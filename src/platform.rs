//! Platform abstraction layer — time, keyboard, and sleep.
//!
//! The [`Platform`] trait decouples the emulator core from the host OS.
//! This lets the same [`crate::emulator::Emulator`] run identically on
//! POSIX systems, Windows, and inside a WebAssembly sandbox — each with its
//! own implementation of this trait.
//!
//! ## Implementations
//!
//! | Target | Type | Notes |
//! |--------|------|-------|
//! | Linux / macOS | `posix_platform::PosixPlatform` | Uses `libc` for time, terminal raw mode, and stdin |
//! | Windows | `windows_platform::WindowsPlatform` | Uses `QueryPerformanceCounter` and `_kbhit`/`_getch` |
//! | WebAssembly | `wasm::WasmPlatform` (private) | Uses `web_sys::Performance`; keyboard fed via JS callbacks |
//!
//! ## Adding a new platform
//!
//! Implement [`Platform`] for your type and pass it to
//! [`crate::emulator::Emulator::run`]. No other changes are needed.

/// Host platform interface used by the emulator for time, I/O, and sleep.
///
/// Every method has a clear contract so implementors know exactly what is
/// expected. The emulator will call these frequently — keep them cheap.
pub trait Platform {
    /// Current time in microseconds.
    ///
    /// The emulator uses this to compute `elapsed_us` between `step` calls,
    /// which drives the CLINT timer. The value does not need to be an absolute
    /// epoch; only the *difference* between two calls matters.
    fn get_time_microseconds(&self) -> u64;

    /// Put the terminal into raw mode (no echo, no line buffering).
    ///
    /// Called once at startup so individual keystrokes reach the emulated
    /// guest without the host OS buffering them.
    fn capture_keyboard(&mut self);

    /// Restore the terminal to its original (canonical) mode.
    ///
    /// Called when the emulator exits, typically via a `Drop` guard.
    fn reset_keyboard(&mut self);

    /// Sleep briefly to avoid burning CPU while the guest is in WFI.
    ///
    /// The duration is intentionally short (< 1 ms) — the goal is to yield
    /// the host scheduler, not to sleep for a fixed wall-clock period.
    fn mini_sleep(&self);

    /// Returns `1` if a key is waiting in stdin, `0` if not, `-1` on EOF.
    fn is_kb_hit(&mut self) -> i32;

    /// Read one byte from stdin.
    ///
    /// Only call this after [`is_kb_hit`] returns `1`. Returns the byte as a
    /// positive `i32`, or `-1` on error/EOF.
    ///
    /// [`is_kb_hit`]: Platform::is_kb_hit
    fn read_kb_byte(&mut self) -> i32;
}

// ─────────────────────────────────────────────────────────────────────────────
// POSIX implementation (Linux / macOS)
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(all(not(target_os = "windows"), not(target_arch = "wasm32")))]
pub mod posix_platform {
    use std::sync::atomic::{AtomicBool, Ordering};

    /// Set to `true` when stdin reaches EOF (e.g. the pipe is closed).
    pub static IS_EOFD: AtomicBool = AtomicBool::new(false);
    /// Set to `true` when SIGINT is received.
    pub static SIGINT_FIRED: AtomicBool = AtomicBool::new(false);

    /// POSIX platform — Linux and macOS.
    pub struct PosixPlatform;

    impl super::Platform for PosixPlatform {
        fn get_time_microseconds(&self) -> u64 {
            let mut tv = libc::timeval {
                tv_sec: 0,
                tv_usec: 0,
            };
            unsafe {
                libc::gettimeofday(&mut tv, std::ptr::null_mut());
            }
            tv.tv_usec as u64 + tv.tv_sec as u64 * 1_000_000
        }

        fn capture_keyboard(&mut self) {
            unsafe {
                let mut term: libc::termios = std::mem::zeroed();
                libc::tcgetattr(0, &mut term);
                term.c_lflag &= !(libc::ICANON | libc::ECHO);
                libc::tcsetattr(0, libc::TCSANOW, &term);
                libc::signal(
                    libc::SIGINT,
                    sigint_handler as *const () as libc::sighandler_t,
                );
            }
        }

        fn reset_keyboard(&mut self) {
            unsafe {
                let mut term: libc::termios = std::mem::zeroed();
                libc::tcgetattr(0, &mut term);
                term.c_lflag |= libc::ICANON | libc::ECHO;
                libc::tcsetattr(0, libc::TCSANOW, &term);
            }
        }

        fn mini_sleep(&self) {
            unsafe {
                libc::usleep(500);
            }
        }

        fn is_kb_hit(&mut self) -> i32 {
            if IS_EOFD.load(Ordering::SeqCst) {
                return -1;
            }
            let mut n: libc::c_int = 0;
            unsafe {
                libc::ioctl(0, libc::FIONREAD, &mut n);
            }
            if n == 0 {
                // A zero-length write is a cheap way to test if the fd is still open.
                let r = unsafe { libc::write(libc::STDIN_FILENO, std::ptr::null(), 0) };
                if r != 0 {
                    IS_EOFD.store(true, Ordering::SeqCst);
                    return -1;
                }
            }
            if n != 0 {
                1
            } else {
                0
            }
        }

        fn read_kb_byte(&mut self) -> i32 {
            if IS_EOFD.load(Ordering::SeqCst) {
                return 0xffff_ffff_u32 as i32;
            }
            let mut c: u8 = 0;
            let r = unsafe {
                libc::read(
                    libc::STDIN_FILENO,
                    &mut c as *mut u8 as *mut libc::c_void,
                    1,
                )
            };
            if r > 0 {
                c as i32
            } else {
                -1
            }
        }
    }

    extern "C" fn sigint_handler(_sig: libc::c_int) {
        SIGINT_FIRED.store(true, Ordering::SeqCst);
        unsafe {
            libc::exit(0);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Windows implementation
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(all(target_os = "windows", not(target_arch = "wasm32")))]
pub mod windows_platform {
    /// Windows platform using `QueryPerformanceCounter` and CRT console APIs.
    ///
    /// Arrow keys arrive from `_getch` as two-byte sequences (`224` + code).
    /// This type translates them to ANSI escape sequences so the guest sees
    /// the standard `ESC [ A` / `ESC [ B` / etc. that terminal apps expect.
    pub struct WindowsPlatform {
        escape_seq: i32,
    }

    impl Default for WindowsPlatform {
        fn default() -> Self {
            Self::new()
        }
    }

    impl WindowsPlatform {
        pub fn new() -> Self {
            WindowsPlatform { escape_seq: 0 }
        }
    }

    impl super::Platform for WindowsPlatform {
        fn get_time_microseconds(&self) -> u64 {
            use windows::Win32::System::Performance::*;
            let mut freq: i64 = 0;
            let mut li: i64 = 0;
            unsafe {
                QueryPerformanceFrequency(&mut freq).unwrap();
                QueryPerformanceCounter(&mut li).unwrap();
            }
            (li as u64 * 1_000_000) / freq as u64
        }

        fn capture_keyboard(&mut self) {}
        fn reset_keyboard(&mut self) {}

        fn mini_sleep(&self) {
            unsafe {
                windows::Win32::System::Threading::Sleep(1);
            }
        }

        fn is_kb_hit(&mut self) -> i32 {
            extern "C" {
                fn _kbhit() -> i32;
            }
            unsafe { _kbhit() }
        }

        fn read_kb_byte(&mut self) -> i32 {
            extern "C" {
                fn _getch() -> i32;
            }

            // Second byte of an arrow-key escape sequence.
            if self.escape_seq == 1 {
                self.escape_seq += 1;
                return '[' as i32;
            }

            let r = unsafe { _getch() };

            if self.escape_seq != 0 {
                self.escape_seq = 0;
                return match r {
                    72 => 'A' as i32, // ↑
                    80 => 'B' as i32, // ↓
                    75 => 'D' as i32, // ←
                    77 => 'C' as i32, // →
                    71 => 'H' as i32, // Home
                    79 => 'F' as i32, // End
                    _ => r,
                };
            }

            match r {
                13 => 10, // CR → LF
                224 => {
                    self.escape_seq = 1;
                    27
                } // start of arrow-key sequence
                _ => r,
            }
        }
    }
}
