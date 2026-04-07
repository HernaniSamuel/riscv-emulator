	.file	"gcc_source_test.c"
	.option nopic
	.attribute arch, "rv32i2p1"
	.attribute unaligned_access, 0
	.attribute stack_align, 16
	.text
	.globl	result
	.section	.sbss,"aw",@nobits
	.align	2
	.type	result, @object
	.size	result, 4
result:
	.zero	4
	.text
	.align	2
	.globl	_start
	.type	_start, @function
_start:
	addi	sp,sp,-16
	sw	s0,12(sp)
	addi	s0,sp,16
	li	a5,42
	mv	a0,a5
	lw	s0,12(sp)
	addi	sp,sp,16
	jr	ra
	.size	_start, .-_start
	.ident	"GCC: (13.2.0-11ubuntu1+12) 13.2.0"
