//! WebAssembly bindings — runs the emulator in the browser.
//!
//! This module is only compiled when targeting `wasm32`. It exposes a small
//! JavaScript-friendly API via [`wasm_bindgen`] that lets a web page boot a
//! Linux kernel and interact with it through a terminal emulator (e.g.
//! [xterm.js](https://xtermjs.org/)).
//!
//! ## How it works
//!
//! 1. The Linux kernel image is embedded at compile time with `include_bytes!`
//!    so no file I/O is needed in the browser.
//! 2. [`EmulatorWasm::new`] creates the emulator and loads the kernel into
//!    emulated RAM.
//! 3. The JavaScript caller drives execution by calling
//!    [`EmulatorWasm::step_batch`] in a `requestAnimationFrame` loop. Each
//!    call executes `n` instructions and returns whatever the kernel printed to
//!    the UART as a UTF-8 string.
//! 4. Keyboard input is fed via [`EmulatorWasm::send_key`], which pushes bytes
//!    into [`WasmPlatform`]'s keyboard buffer.
//! 5. When the kernel halts ([`EmulatorWasm::is_halted`] returns `true`), the
//!    caller should check [`EmulatorWasm::is_restart`] to decide whether to
//!    recreate the emulator (reboot) or display a halt message.
//!
//! ## UART output buffering
//!
//! In native mode, UART bytes are printed directly to stdout. In WASM mode,
//! [`crate::mmio::handle_store`] writes them into
//! [`crate::emulator::Emulator::output_buf`] instead. `step_batch` drains this
//! buffer and returns the accumulated output to JavaScript after each batch.

use std::collections::VecDeque;
use wasm_bindgen::prelude::*;

use crate::cpu::StepResult;
use crate::emulator::{Emulator, RunConfig};
use crate::platform::Platform;

// The kernel image is embedded at compile time so the browser needs no file I/O.
static KERNEL_IMAGE: &[u8] = include_bytes!("../operational-systems/Image");

/// Browser platform — implements [`Platform`] using Web APIs.
///
/// Time comes from `window.performance.now()`. Sleep is a no-op because the
/// JavaScript event loop controls the execution cadence. Keyboard input
/// arrives via [`EmulatorWasm::send_key`] rather than from stdin.
struct WasmPlatform {
    /// Keyboard bytes queued by JavaScript via [`EmulatorWasm::send_key`].
    kbd_buf: VecDeque<u8>,
}

impl WasmPlatform {
    fn new() -> Self {
        WasmPlatform {
            kbd_buf: VecDeque::new(),
        }
    }
}

impl Platform for WasmPlatform {
    fn get_time_microseconds(&self) -> u64 {
        let window = web_sys::window().expect("no window");
        let perf = window.performance().expect("no performance");
        (perf.now() * 1000.0) as u64
    }

    fn capture_keyboard(&mut self) {}
    fn reset_keyboard(&mut self) {}
    fn mini_sleep(&self) {}

    fn is_kb_hit(&mut self) -> i32 {
        if self.kbd_buf.is_empty() {
            0
        } else {
            1
        }
    }

    fn read_kb_byte(&mut self) -> i32 {
        self.kbd_buf.pop_front().map(|b| b as i32).unwrap_or(-1)
    }
}

/// Reason the emulated CPU stopped executing.
enum HaltReason {
    /// Still running normally.
    Running,
    /// Kernel issued a clean shutdown (SYSCON `0x5555`).
    Poweroff,
    /// Kernel requested a reboot (SYSCON `0x7777`). JavaScript should call
    /// `emulator.free()` and create a new [`EmulatorWasm`] to restart.
    Restart,
    /// Unrecoverable fault.
    Fault,
}

/// The emulator instance exposed to JavaScript.
///
/// Create one with `new EmulatorWasm()` from JS, then call `step_batch` in
/// a loop. See the module documentation for a full usage example.
#[wasm_bindgen]
pub struct EmulatorWasm {
    emu: Emulator,
    plat: WasmPlatform,
    halt: HaltReason,
}

#[wasm_bindgen]
impl EmulatorWasm {
    /// Create a new emulator and load the embedded Linux kernel image.
    ///
    /// This allocates 64 MB of emulated RAM and copies the kernel into it.
    /// On failure (e.g. kernel too large) a JavaScript `Error` is thrown.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Result<EmulatorWasm, JsValue> {
        console_error_panic_hook::set_once();

        let mut emu = Emulator::new(64 * 1024 * 1024);
        emu.load_raw_from_bytes(KERNEL_IMAGE, None)
            .map_err(|e| JsValue::from_str(&format!("load error: {}", e)))?;

        Ok(EmulatorWasm {
            emu,
            plat: WasmPlatform::new(),
            halt: HaltReason::Running,
        })
    }

    /// Execute up to `n` instructions and return any UART output as UTF-8.
    ///
    /// Returns an empty string if the CPU is already halted. The JavaScript
    /// caller should check [`is_halted`] after each call and stop the loop
    /// when it returns `true`.
    ///
    /// [`is_halted`]: EmulatorWasm::is_halted
    pub fn step_batch(&mut self, n: u32) -> String {
        if matches!(self.halt, HaltReason::Running) {
            let cfg = RunConfig {
                instct: n as i64,
                time_divisor: 1,
                fixed_update: false,
                do_sleep: false,
                single_step: false,
                trace: None,
            };

            self.halt = match self.emu.run(&mut { cfg }, &mut self.plat) {
                StepResult::Poweroff => HaltReason::Poweroff,
                StepResult::Restart => HaltReason::Restart,
                StepResult::Fault => HaltReason::Fault,
                _ => HaltReason::Running,
            };
        }

        let output = self.emu.drain_output();
        String::from_utf8_lossy(&output).into_owned()
    }

    /// Send a keyboard byte to the emulator.
    ///
    /// Call this from the xterm.js `onData` callback:
    /// ```js
    /// term.onData(data => {
    ///   for (let i = 0; i < data.length; i++) emu.send_key(data.charCodeAt(i));
    /// });
    /// ```
    pub fn send_key(&mut self, byte: u8) {
        self.plat.kbd_buf.push_back(byte);
    }

    /// Returns `true` if the CPU has stopped for any reason.
    ///
    /// When this returns `true`, check [`is_restart`], [`is_poweroff`], or
    /// [`is_fault`] to find out why.
    ///
    /// [`is_restart`]: EmulatorWasm::is_restart
    /// [`is_poweroff`]: EmulatorWasm::is_poweroff
    /// [`is_fault`]: EmulatorWasm::is_fault
    pub fn is_halted(&self) -> bool {
        !matches!(self.halt, HaltReason::Running)
    }

    /// Returns `true` if the kernel requested a reboot.
    ///
    /// The JavaScript caller should free the current instance and create a new
    /// one: `emu.free(); emu = new EmulatorWasm();`
    pub fn is_restart(&self) -> bool {
        matches!(self.halt, HaltReason::Restart)
    }

    /// Returns `true` if the kernel issued a clean shutdown.
    pub fn is_poweroff(&self) -> bool {
        matches!(self.halt, HaltReason::Poweroff)
    }

    /// Returns `true` if the emulator encountered an unrecoverable fault.
    pub fn is_fault(&self) -> bool {
        matches!(self.halt, HaltReason::Fault)
    }
}
