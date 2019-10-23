	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	leds: .byte 0

.text
.global main
	.equ RCC_AHB2ENR,   0x4002104C
	.equ GPIOB_MODER,   0x48000400
	.equ GPIOB_OTYPER,  0x48000404
	.equ GPIOB_OSPEEDR, 0x48000408
	.equ GPIOB_PUPDR,   0x4800040C
	.equ GPIOB_ODR,     0x48000414
	.equ X, 10000
	.equ Y, 100
	.equ N, 100


main:
	BL GPIO_init
	MOVS R11, #0 // 0: shift left, 1: shift right
	MOVS R12, #1
	LDR R0, =leds
	STRB R12, [R0]

Loop:
	BL DisplayLED
	BL Delay
	BL ChangePattern
	B Loop

//LED on PB3456
GPIO_init:
	// Enable AHB2 clock
	movs r0, #0b11
	ldr r1, =RCC_AHB2ENR
	str r0, [r1]

	//set PB3456 as output mode(01)
	ldr r1, =GPIOB_MODER
	ldr r2, [r1]
	and r2, #0xFFFFC03F
	orr r2, r2, #0x1540
	str r2, [r1]

	//set PB3456 as push-pull(0)
	ldr r1, =GPIOB_OTYPER
	ldr r2, [r1]
	and r2, #0b0000111
	strh r2, [r1]

	//set PB3456 as pull-up(01)
	ldr r1, =GPIOB_PUPDR
	ldr r2, [r1]
	and r2, #0xFFFFC03F
	orr r2, r2, #0x1540
	str r2, [r1]

	//set PB3456 as high speed(10)
	ldr r1, =GPIOB_OSPEEDR
	ldr r2, [r1]
	and r2, #0xFFFFC03F
	orr r2, r2, #0x2A80
	str r2, [r1]

	bx lr

DisplayLED:
	ldr r1, =GPIOB_ODR
	LDR R0, =leds
	LDRB R0, [R0]

	rsb r0, r0, #0xFF
	lsl r0, #3
	and r0, #0b1111000
	strh r0, [r1]

	bx lr

ChangePattern: // r12 is pattern
	cmp r11, #0
	ITE EQ
	lslEQ r12, #1
	lsrNE r12, #1

	cmp r12, #0b0010
	IT EQ
	addEQ r12, #1

	LDR R0, =leds
	AND R1, R12, #0b1111
	STRB R1, [R0]

	cmp r12, #0b11000
	push {lr}
	IT EQ
	blEQ ChangeDirection
	pop {lr}

	cmp r12, #0b0001
	push {lr}
	IT EQ
	blEQ ChangeDirection
	pop {lr}

	bx lr

ChangeDirection:
	cmp r11, #0
	ITE EQ
	movEQ r11, #1
	movNE r11, #0
	bx lr

Delay: //  2+(2+4*Y+3)*X ~= 4*X*Y * 0.25us
	cmp r10, #N
	beq EXIT
	LDR R3, =X
	L1: LDR R4,	=Y
		L2: SUBS R4, #1
		BNE L2
		SUBS R3, #1
		BNE L1

	add R10, #1
	BX LR

EXIT:
	nop
