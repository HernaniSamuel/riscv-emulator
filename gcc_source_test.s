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
	addi	sp,sp,-80
	sw	s0,76(sp)
	addi	s0,sp,80
	li	a5,5
	sw	a5,-24(s0)
	li	a5,10
	sw	a5,-28(s0)
	lw	a4,-24(s0)
	lw	a5,-28(s0)
	add	a5,a4,a5
	sw	a5,-32(s0)
	lw	a4,-32(s0)
	lw	a5,-24(s0)
	sub	a5,a4,a5
	sw	a5,-36(s0)
	lw	a4,-32(s0)
	lw	a5,-28(s0)
	and	a5,a4,a5
	sw	a5,-40(s0)
	lw	a4,-32(s0)
	lw	a5,-28(s0)
	or	a5,a4,a5
	sw	a5,-44(s0)
	lw	a4,-32(s0)
	lw	a5,-28(s0)
	xor	a5,a4,a5
	sw	a5,-48(s0)
	lw	a5,-32(s0)
	slli	a5,a5,2
	sw	a5,-52(s0)
	lw	a5,-52(s0)
	srai	a5,a5,1
	sw	a5,-56(s0)
	sw	zero,-20(s0)
	lw	a4,-56(s0)
	li	a5,10
	ble	a4,a5,.L2
	li	a5,1
	sw	a5,-20(s0)
	j	.L3
.L2:
	li	a5,2
	sw	a5,-20(s0)
.L3:
	lw	a5,-24(s0)
	sw	a5,-72(s0)
	lw	a5,-28(s0)
	sw	a5,-68(s0)
	lw	a5,-32(s0)
	sw	a5,-64(s0)
	lw	a4,-72(s0)
	lw	a5,-68(s0)
	add	a5,a4,a5
	sw	a5,-60(s0)
	lw	a4,-60(s0)
	lw	a5,-20(s0)
	add	a4,a4,a5
	lui	a5,%hi(result)
	sw	a4,%lo(result)(a5)
.L4:
	j	.L4
	.size	_start, .-_start
	.ident	"GCC: (13.2.0-11ubuntu1+12) 13.2.0"
