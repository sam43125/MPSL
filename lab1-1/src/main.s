	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	result: .byte 0

.text
	.global main
	.equ X, 0x55AA
	.equ Y, 0xAA55

hamm:
	eor R3, R0, R1
	mov R4, #0      // counter
	loop:
		cmp R3, #0
		beq return
		and R5, R3, #1
		lsr R3, #1
		cmp R5, #1
		bne loop
		add R4, R4, #1
		b loop
	return:
		str R4, [R2]
		bx lr

main:
	ldr R0, =X
	ldr R1, =Y
	ldr R2, =result
	bl hamm

L: b L

