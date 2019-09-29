	.syntax unified
	.cpu cortex-m4
	.thumb

.text
	.global main
	.equ N, 46

fib:
	cmp R0, #100
	bgt NERROR
	cmp R0, #1
	blt NERROR
	movs R4, #1  // result = 1
	movs R1, #1  // a = 1
	movs R2, #1  // b = 1

	fib_loop:
		cmp R0, #3
		blt fib_return

		adds R4, R1, R2
		bvs fib_overflow

		movs R3, R2 // temp
		movs R2, R4
		movs R1, R3

		sub R0, R0, #1
		b fib_loop

	NERROR:
		movs R4, #-2
		bx lr
	fib_overflow:
		movs R4, #-1
	fib_return:
		bx lr

main:
	movs R0, #N
	bl fib
L: b L

