	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	arr1: .byte 0x19, 0x34, 0x14, 0x32, 0x52, 0x23, 0x61, 0x29
	arr2: .byte 0x18, 0x17, 0x33, 0x16, 0xFA, 0x20, 0x55, 0xAC

.text
.global main

do_sort:
	mov r1, #-1 // i = -1
	outer_loop:
		add r1, r1, #1
		cmp r1, #7
		bge return
		mov r2, #-1 // j = -1
		inner_loop:
			add r2, r2, #1
			rsb r3, r1, #7      // r3 = 7 - i
			cmp r2, r3
			bge outer_loop

			add r6, r2, #1      // r6 = j + 1
			ldrb r4, [r0, r2]   // r4 = arr[j]
			ldrb r5, [r0, r6]   // r5 = arr[j+1]
			cmp r4, r5
			ble inner_loop

			strb r5, [r0, r2]   // arr[j] = r5
			strb r4, [r0, r6]   // arr[j+1] = r4
			b inner_loop

	return:
		bx lr

main:
	ldr r0, =arr1
	bl do_sort
	ldr r0, =arr2
	bl do_sort

L: b L

