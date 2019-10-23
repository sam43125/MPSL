	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	password: .byte 0b1100

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

Loop:

	BL IsPushed
	CMP r9, #0
	BEQ Loop

	BL ReadDIP // return r3
	BL Blink // based on r3 and password
	mov r9, #0 // reset pushed state
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

	//turn off PB3456 by default
	ldr r1,	=GPIOB_ODR
	mov r2, #0b1111000
	strh r2, [r1]

	//Set PC0123,13 as input mode
	ldr r1,	=GPIOC_MODER
	ldr r0,	[r1]
	ldr r2,	=#0xF3FFFF00
	and r0,	r2
	str r0,	[r1]

	bx lr

IsPushed:

	//Set data register address
	ldr r2,	=GPIOC_IDR
	ldr r3,	[r2]
	and r3, #0b10000000000000
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

ReadDIP: // return r3
	ldr r2,	=GPIOC_IDR
	ldrh r3, [r2]
	rsb r3, r3, #0xFFFFFFFF
	and r3, r3, #0b1111
	bx lr

Blink:
	push {r5, r6, lr}
	ldr r1,	=GPIOB_ODR
	mov r5, #2500 // Delay 0.25s
	mov r6, #100
	mov r0, #0b1111000 // turn off
	mov r2, #0b0000000 // turn on
	str r0, [r1]
	bl Delay
	str r2, [r1]
	bl Delay
	str r0, [r1]
	bl Delay

	ldr r4, =password
	ldrb r4, [r4]
	cmp r3, r4
	bne Blink_return

	strh r2, [r1]
	bl Delay
	strh r0, [r1]
	bl Delay
	strh r2, [r1]
	bl Delay
	strh r0, [r1]
	bl Delay

	Blink_return:
		pop {r5, r6, lr}
		bx lr

Delay: //  2+(2+4*Y+3)*X ~= 4*X*Y * 0.25us
	push {r0, r5, r6}
	cmp r10, #N
	beq EXIT
	mov r0, r6 // backup Y
	L1: mov r6, r0
		L2: SUBS R6, #1
		BNE L2
		SUBS R5, #1
		BNE L1

	add R10, #1
	pop {r0, r5, r6}
	BX LR

EXIT:
	nop
