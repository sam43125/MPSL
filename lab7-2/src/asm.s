	.syntax unified
	.cpu cortex-m4
	.thumb

.text
.global GPIO_init, Delay1sWith4MHz, Delay20msWith4MHz
	.equ RCC_AHB2ENR,   0x4002104C
	.equ GPIOA_MODER,   0x48000000
	.equ GPIOA_OTYPER,  0x48000004
	.equ GPIOA_OSPEEDR, 0x48000008
	.equ GPIOA_PUPDR,   0x4800000C
	.equ X, 13500
	.equ Y, 50
	.equ P, 13500
	.equ Q, 2

//LED on PA5
GPIO_init:
	push {r0-r2, lr}
	// Enable AHB2 clock
	movs r0, #0b111
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

	pop {r0-r2, pc}

Delay1sWith4MHz: //  2+(2+4*Y+3)*X ~= 4*X*Y * 0.25us
	push {r3,r4,lr}
	LDR R3, =X
	L1: LDR R4,	=Y
		L2: SUBS R4, #1
		BNE L2
		SUBS R3, #1
		BNE L1
	pop {r3,r4,pc}

Delay20msWith4MHz: //  2+(2+4*Y+3)*X ~= 4*X*Y * 0.25us
	push {r3,r4,lr}
	LDR R3, =P
	_L1: LDR R4,	=Q
		_L2: SUBS R4, #1
		BNE _L2
		SUBS R3, #1
		BNE _L1
	pop {r3,r4,pc}
