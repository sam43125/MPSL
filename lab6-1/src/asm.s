	.syntax unified
	.cpu cortex-m4
	.thumb

.text
.global GPIO_init, Delay1sWith4MHz
	.equ RCC_AHB2ENR,   0x4002104C
	.equ GPIOA_MODER,   0x48000000
	.equ GPIOA_OTYPER,  0x48000004
	.equ GPIOA_OSPEEDR, 0x48000008
	.equ GPIOA_PUPDR,   0x4800000C
	.equ GPIOA_ODR,     0x48000014
	.equ GPIOC_MODER,   0x48000800
	.equ GPIOC_IDR,     0x48000810
	.equ X, 13500
	.equ Y, 100

//LED on PA5
GPIO_init:
	// Enable AHB2 clock
	push {r0-r2,lr}
	movs r0, #0b101
	ldr r1, =RCC_AHB2ENR
	str r0, [r1]

	//set PA5 as output mode
	movs r0, #0x400
	ldr r1, =GPIOA_MODER
	ldr r2, [r1]
	and r2, #0xFFFFF3FF
	orrs r2, r2, r0
	str r2, [r1]
	// Default PA5 is Pull-up output, no need to set

	// Set PA5 as high speed mode
	movs r0, #0x800
	ldr r1, =GPIOA_OSPEEDR
	strh r0, [r1]

	//Set GPIOC Pin13 as input mode
	ldr r1,	=GPIOC_MODER
	ldr r0,	[r1]
	ldr r2,	=#0xF3FFFFFF
	and r0,	r2
	str r0,	[r1]

	pop {r0-r2,pc}

Delay1sWith4MHz: //  2+(2+4*Y+3)*X ~= 4*X*Y * 0.25us
	push {r3,r4,lr}
	LDR R3, =X
	L1: LDR R4,	=Y
		L2: SUBS R4, #1
		BNE L2
		SUBS R3, #1
		BNE L1
	pop {r3,r4,pc}
