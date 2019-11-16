	.syntax unified
	.cpu cortex-m4
	.thumb
.data
	arr: .byte 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5b, 0x5F, 0x70, 0x7F, 0x7b, 0x77, 0x1F, 0x4E, 0x3D, 0x4F, 0x47

.text
	.global main
	.equ DATA, 0x20 //PA5
	.equ LOAD, 0x40 //PA6
	.equ CLOCK, 0x80 //PA7
	.equ DECODE_MODE, 0x09
	.equ DISPLAY_TEST, 0x0F
	.equ SCAN_LIMIT, 0x0B
	.equ INTENSITY, 0x0A
	.equ SHUTDOWN, 0x0C
	.equ DIGIT0, 0x01
	.equ RCC_AHB2ENR,   0x4002104C
	.equ GPIOA_BASE,    0x48000000
	.equ GPIOA_MODER,   0x48000000
	.equ GPIOA_OTYPER,  0x48000004
	.equ GPIOA_OSPEEDR, 0x48000008
	.equ GPIOA_PUPDR,   0x4800000C
	.equ GPIOA_ODR,     0x48000014
	.equ GPIO_BSRR_OFFSET, 0x18
	.equ GPIO_BRR_OFFSET, 0x28
	.equ X, 13500
	.equ Y, 100


main:
	BL GPIO_init
	BL max7219_init
	mov r3, #0

loop:
	ldr r2, =arr
	BL Display0toF
	BL Delay
	B loop

GPIO_init:
	// Enable AHB2 clock
	movs r0, #0b1
	ldr r1, =RCC_AHB2ENR
	str r0, [r1]

	//set PA567 as output mode
	movs r0, #0x5400
	ldr r1, =GPIOA_MODER
	ldr r2, [r1]
	and r2, #0xFFFF03FF
	orrs r2, r2, r0
	str r2, [r1]
	BX LR

Display0toF:

	ldr r0, =#0x01
	ldrb r1, [r2, r3]
	push {r2,r3,lr}
	BL MAX7219Send
	pop {r2,r3,lr}
	add r3, #1
	cmp r3, #15
	IT GT
	movGT r3, #0
	BX LR

MAX7219Send:
	//input parameter: r0 is ADDRESS , r1 is DATA
	lsl r0, r0, #8
	add r0, r0, r1
	ldr r1, =#GPIOA_BASE
	ldr r2, =#LOAD
	ldr r3, =#DATA
	ldr r4, =#CLOCK
	ldr r5, =#GPIO_BSRR_OFFSET
	ldr r6, =#GPIO_BRR_OFFSET
	mov r7, #16  //r7 = i
	.max7219send_loop:
		mov r8, #1
		sub r9, r7, #1
		lsl r8, r8, r9 // r8 = mask
		str r4, [r1,r6] //HAL_GPIO_WritePin(GPIOA, CLOCK, 0);
		tst r0, r8
		beq .bit_not_set//bit not set
		str r3, [r1,r5] //HAL_GPIO_WritePin(GPIOA, DATA, 1);
		b .if_done
	.bit_not_set:
		str r3, [r1,r6] //HAL_GPIO_WritePin(GPIOA, DATA, 0);
	.if_done:
		str r4, [r1,r5] //HAL_GPIO_WritePin(GPIOA, CLOCK, 1);
		subs r7, r7, #1
		bgt .max7219send_loop
		str r2, [r1,r6] //HAL_GPIO_WritePin(GPIOA, LOAD, 0);
		str r2, [r1,r5] //HAL_GPIO_WritePin(GPIOA, LOAD, 1);

	BX LR

max7219_init:

	push {r0, r1, r2, lr}
	ldr r0, =#DECODE_MODE
	ldr r1, =#0x0
	BL MAX7219Send
	ldr r0, =#DISPLAY_TEST
	ldr r1, =#0x1
	BL MAX7219Send
	ldr r0, =#DISPLAY_TEST
	ldr r1, =#0x0
	BL MAX7219Send
	ldr r0, =#SCAN_LIMIT
	ldr r1, =0x0
	BL MAX7219Send
	ldr r0, =#INTENSITY
	ldr r1, =#0xA
	BL MAX7219Send
	ldr r0, =#SHUTDOWN
	ldr r1, =#0x1
	BL MAX7219Send
	pop {r0, r1, r2, pc}

Delay:
	push {r3,r4,r10,lr}
	LDR R3, =X
	L1: LDR R4,	=Y
		L2: SUBS R4, #1
		BNE L2
		SUBS R3, #1
		BNE L1

	add R10, #1
	pop {r3,r4,r10,pc}
