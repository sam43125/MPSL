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
	.equ GPIOC_MODER,   0x48000800
	.equ GPIOC_IDR,     0x48000810
	.equ N, 1000

main:
	BL GPIO_init
	MOVS R9, #0  // 0: Display, 1: Don't display
	MOVS R11, #0 // 0: shift left, 1: shift right
	MOVS R12, #1 // Pattern
	LDR R0, =leds
	STRB R12, [R0]

Loop:

	MOV R5, #1666 // X
	MOV R6, #100   // Y
	BL Delay
	BL IsPushed
	BL Delay
	BL IsPushed
	BL Delay
	BL IsPushed
	BL Delay
	BL IsPushed
	BL Delay
	BL IsPushed
	CMP r9, #1
	BEQ Loop

	BL DisplayLED
	BL Delay

	BL ChangePattern
	B Loop

//LED on PB3456
GPIO_init:
	// Enable AHB2 clock
	movs r0, #0b111
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

	//Set GPIOC Pin13 as input mode
	ldr r1,	=GPIOC_MODER
	ldr r0,	[r1]
	ldr r2,	=#0xF3FFFFFF
	and r0,	r2
	str r0,	[r1]

	bx lr

IsPushed:

	//Set data register address
	ldr r2,	=GPIOC_IDR
	ldr r3,	[r2]
	movs r4, #1
	lsl r4,	#13
	cmp r3, r4
	bne do_pushed

	bx lr

do_pushed:
	// Wait 10ms and check again
	push {r5, r6, lr}
	MOVS R5, #100 // X
	MOVS R6, #100   // Y
	bl Delay
	pop {r5, r6, lr}

	ldr r3,	[r2]
	cmp r3, r4
	beq return

	cmp r9, #0
	ITE EQ
	movEQ r9, #1
	movNE r9, #0

return:
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
	push {r5, r6}
	cmp r10, #N
	beq EXIT
	mov r0, r6 // backup Y
	L1: mov r6, r0
		L2: SUBS R6, #1
		BNE L2
		SUBS R5, #1
		BNE L1

	add R10, #1
	pop {r5, r6}
	BX LR

EXIT:
	nop
