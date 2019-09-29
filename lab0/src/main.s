	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	X: .word 5
	Y: .word 10
	Z: .word 0

.text
.global main
.equ AA, 0x55

main:
	ldr r0, =X
	ldr r1, [r0]
	ldr r2, =Y
	ldr r3, [r2]
	mov r4, #10
	mul r4, r1, r4
	add r4, r4, r3
	str r4, [r0]
	sub r4, r3, r4
	ldr r0, =Z
	str r4, [r0]

L:
	b L
	b L
