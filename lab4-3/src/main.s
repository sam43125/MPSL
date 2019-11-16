	.syntax unified
	.cpu cortex-m4
	.thumb

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
	.equ GPIOC_MODER,   0x48000800
	.equ GPIOC_IDR,     0x48000810
	.equ GPIO_BSRR_OFFSET, 0x18
	.equ GPIO_BRR_OFFSET, 0x28
	.equ X, 1250
	.equ Y, 100



main:
	BL GPIO_init
	BL max7219_init
	mov r10, #0 // a
	mov r11, #1 // b
	mov r12, #0 // isOverflow

Loop:
	mov r9, #0
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

	CMP r9, #10
	IT EQ
	BLEQ ResetFib
	BEQ Loop

	CMP r9, #2
	BLT Loop
	CMP r12, #0
	BNE Loop // if isOverflow, do nothing

	.loop_delay: // again

		CMP r9, #10
		IT GE
		BLGE ResetFib
		BGE Loop

		ldr r2,	=GPIOC_IDR
		ldr r3,	[r2]
		movs r4, #1
		lsl r4,	#13
		cmp r3, r4
		IT NE
		addNE r9, #1
		bne .loop_delay

	BL CalculateFib
	BL DisplayFib

	B Loop



GPIO_init:
	//TODO: Initialize three GPIO pins as output for max7219 DIN, CS and CLK
	// Enable AHB2 clock
	movs r0, #0b101
	ldr r1, =RCC_AHB2ENR
	str r0, [r1]

	//set PA567 as output mode
	movs r0, #0x5400
	ldr r1, =GPIOA_MODER
	ldr r2, [r1]
	and r2, #0xFFFF03FF
	orrs r2, r2, r0
	str r2, [r1]

	//Set GPIOC Pin13 as input mode
	ldr r1,	=GPIOC_MODER
	ldr r0,	[r1]
	ldr r2,	=#0xF3FFFFFF
	and r0,	r2
	str r0,	[r1]

	BX LR

MAX7219Send:
	//input parameter: r0 is ADDRESS , r1 is DATA
	//TODO: Use this function to send a message to max7219
	push {r0-r9}
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

	pop {r0-r9}
	BX LR

max7219_init:
	//TODO: Initialize max7219 registers
	push {r0, r1, r2, lr}
	ldr r0, =#DECODE_MODE
	ldr r1, =#0xFF
	BL MAX7219Send
	ldr r0, =#DISPLAY_TEST
	ldr r1, =#0x1
	BL MAX7219Send
	ldr r0, =#DISPLAY_TEST
	ldr r1, =#0x0
	BL MAX7219Send
	ldr r0, =#SCAN_LIMIT
	ldr r1, =0x07
	BL MAX7219Send
	ldr r0, =#INTENSITY
	ldr r1, =#0xA
	BL MAX7219Send
	ldr r0, =#SHUTDOWN
	ldr r1, =#0x1
	BL MAX7219Send
	BL ClearDisplay
	mov r0, #0x1
	mov r1, #0x0
	BL MAX7219Send // Display 0 on digit 0
	pop {r0, r1, r2, pc}

ClearDisplay:
	push {r0-r1, lr}
	mov r0, #0x1
	mov r1, #0b1111 // Empty
	.clear_loop:
		BL MAX7219Send
		add r0, #1
		cmp r0, #0x8
		ble .clear_loop
	pop {r0-r1, pc}

IsPushed:
	ldr r2,	=GPIOC_IDR
	ldr r3,	[r2]
	movs r4, #1
	lsl r4,	#13
	cmp r3, r4
	IT NE
	addNE r9, #1
	bx lr

ResetFib:
	push {lr}
	BL ClearDisplay
	mov r10, #0 // a
	mov r11, #1 // b
	mov r12, #0 // isOverflow = 0
	mov r0, #0x1
	mov r1, #0x0
	BL MAX7219Send // Display 0 on digit 0

	.reset_delay:
		ldr r2,	=GPIOC_IDR
		ldr r3,	[r2]
		movs r4, #1
		lsl r4,	#13
		cmp r3, r4
		bne .reset_delay

	pop {pc}

CalculateFib:
	mov r0, r11			// temp = b
	add r11, r10, r11	// b += a
	mov r10, r0			// a = temp
	bx lr

DisplayFib:
	mov r0, #1 // opcode of digit i
	mov r2, #10
	mov r3, r11 // backup b
	.loop:
		cmp r3, #10
		mov r4, r3 // old r3
		ITTEE GE
		udivGE r3, r3, r2 // r3 = r3//r2
		mlsGE r1, r2, r3, r4 // r1 = r4 - r2*r3
		movLT r1, r3
		movLT r3, #0

		push {r0-r4, lr}
		bl MAX7219Send
		pop {r0-r4, lr}
		add r0, #1
		cbz r3, .return
		cmp r0, 0x8
		bgt .error
		b .loop

	.error:
		mov r12, #1 // isOverflow = 1
		push {r0-r1, lr}
		BL ClearDisplay
		mov r0, #0x1
		mov r1, #0x1
		BL MAX7219Send // Display 1 on digit 0
		mov r0, #0x2
		mov r1, #0xa
		BL MAX7219Send // Display - on digit 1
		pop {r0-r1, lr}
	.return:
		bx lr

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

