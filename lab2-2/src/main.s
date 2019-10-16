	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	result: .word 0
	max_size: .word 0

.text
	.global main
	m: .word 0x5E
	n: .word 0x60

GCD: // input r0, r1, return r2
	eor r2, r0, r1 // if(m == 0 || n == 0)
	cbz r0, return //     return m ^ n;
	cbz r1, return

	and r3, r0, #1
	and r4, r1, #1
	orr r5, r3, r4
	cbnz r5, else_if_m_is_even

	lsr r0, #1
	lsr r1, #1
	add r6, r6, #4 // max_stack_size
	push {lr}
	bl GCD
	mul r2, r2, r12
	pop {pc}

	else_if_m_is_even:
		cbnz r3, else_if_n_is_even
		lsr r0, #1
		add r6, r6, #4
		push {lr}
		bl GCD
		pop {pc}

	else_if_n_is_even:
		cbnz r4, else
		lsr r1, #1
		add r6, r6, #4
		push {lr}
		bl GCD
		pop {pc}

	else:
		cmp r0, r1
		ITEEE GE
		subGE r0, r0, r1
		movLT r11, r0    // backup m
		subLT r0, r1, r0
		movLT r1, r11
		add r6, r6, #4
		push {lr}
		bl GCD
		pop {pc}

	return: // m == 0 || n == 0
		bx lr

main:
	mov r12, #2
	mov r6, #0
	// r0 = m, r1 = n
	ldr r0, =m
	ldr r0, [r0]
	ldr r1, =n
	ldr r1, [r1]

	bl GCD

	// get return val and store into result
	ldr r0, =result
	str r2, [r0]

	ldr r0, =max_size
	str r6, [r0]
	nop
