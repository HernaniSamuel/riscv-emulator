	.file	"gcc_source_test.c"
	.option nopic
	.attribute arch, "rv32i2p1"
	.attribute unaligned_access, 0
	.attribute stack_align, 16
	.text
	.align	2
	.globl	mix
	.type	mix, @function
mix:
	addi	sp,sp,-32
	sw	s0,28(sp)
	addi	s0,sp,32
	sw	a0,-20(s0)
	lw	a5,-20(s0)
	slli	a4,a5,3
	lw	a5,-20(s0)
	srli	a5,a5,2
	xor	a4,a4,a5
	li	a5,-1515872256
	addi	a5,a5,1445
	xor	a5,a4,a5
	mv	a0,a5
	lw	s0,28(sp)
	addi	sp,sp,32
	jr	ra
	.size	mix, .-mix
	.align	2
	.globl	main
	.type	main, @function
main:
	addi	sp,sp,-80
	sw	ra,76(sp)
	sw	s0,72(sp)
	addi	s0,sp,80
	sw	zero,-20(s0)
	li	a5,10
	sw	a5,-32(s0)
	li	a5,20
	sw	a5,-36(s0)
	lw	a4,-32(s0)
	lw	a5,-36(s0)
	add	a4,a4,a5
	mv	a5,a4
	slli	a5,a5,1
	add	a5,a5,a4
	sw	a5,-40(s0)
	lw	a0,-40(s0)
	call	mix
	mv	a4,a0
	lw	a5,-20(s0)
	xor	a5,a5,a4
	sw	a5,-20(s0)
	li	a5,-1
	sw	a5,-44(s0)
	li	a5,1
	sw	a5,-48(s0)
	lw	a5,-48(s0)
	lw	a4,-44(s0)
	bge	a4,a5,.L4
	lw	a4,-20(s0)
	li	a5,286330880
	addi	a5,a5,273
	xor	a5,a4,a5
	sw	a5,-20(s0)
.L4:
	lw	a5,-44(s0)
	lw	a4,-48(s0)
	bgeu	a4,a5,.L5
	lw	a4,-20(s0)
	li	a5,572661760
	addi	a5,a5,546
	xor	a5,a4,a5
	sw	a5,-20(s0)
.L5:
	li	a5,-2130739200
	addi	a5,a5,-255
	sw	a5,-60(s0)
	addi	a5,s0,-60
	sw	a5,-52(s0)
	lw	a5,-52(s0)
	addi	a5,a5,1
	lbu	a5,0(a5)
	sb	a5,-53(s0)
	lw	a5,-52(s0)
	lhu	a5,2(a5)
	sh	a5,-56(s0)
	lb	a5,-53(s0)
	lw	a4,-20(s0)
	xor	a5,a4,a5
	sw	a5,-20(s0)
	lh	a5,-56(s0)
	lw	a4,-20(s0)
	xor	a5,a4,a5
	sw	a5,-20(s0)
	sw	zero,-24(s0)
	sw	zero,-28(s0)
	j	.L6
.L7:
	lw	a5,-28(s0)
	slli	a4,a5,1
	lw	a5,-28(s0)
	srli	a5,a5,1
	xor	a5,a4,a5
	lw	a4,-24(s0)
	add	a5,a4,a5
	sw	a5,-24(s0)
	lw	a5,-28(s0)
	addi	a5,a5,1
	sw	a5,-28(s0)
.L6:
	lw	a4,-28(s0)
	li	a5,15
	bleu	a4,a5,.L7
	lw	a4,-20(s0)
	lw	a5,-24(s0)
	xor	a5,a4,a5
	sw	a5,-20(s0)
	li	a5,-559038464
	addi	a5,a5,-273
	sw	a5,-76(s0)
	li	a5,-889274368
	addi	a5,a5,-1346
	sw	a5,-72(s0)
	lw	a4,-76(s0)
	lw	a5,-72(s0)
	xor	a5,a4,a5
	sw	a5,-68(s0)
	lw	a4,-68(s0)
	li	a5,4096
	addi	a5,a5,564
	add	a5,a4,a5
	sw	a5,-64(s0)
	lw	a5,-64(s0)
	lw	a4,-20(s0)
	xor	a5,a4,a5
	sw	a5,-20(s0)
	lw	a5,-20(s0)
	mv	a0,a5
	lw	ra,76(sp)
	lw	s0,72(sp)
	addi	sp,sp,80
	jr	ra
	.size	main, .-main
	.align	2
	.globl	_start
	.type	_start, @function
_start:
	addi	sp,sp,-32
	sw	ra,28(sp)
	sw	s0,24(sp)
	addi	s0,sp,32
	call	main
	sw	a0,-20(s0)
	lw	a5,-20(s0)
 #APP
# 88 "tests/gcc_source_test.c" 1
	mv a0, a5
li a7, 93
ecall

# 0 "" 2
 #NO_APP
.L10:
	j	.L10
	.size	_start, .-_start
	.ident	"GCC: (13.2.0-11ubuntu1+12) 13.2.0"
