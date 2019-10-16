	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	user_stack_bottom: .zero 128
	expr_result: .word 0

.text
	.global main
	postfix_expr: .asciz "-100 10 20 + - 10 +"

main:
	ldr r0, =user_stack_bottom
	add r0, r0, #128
	msr MSP, r0
	mov r4, #-1 // i = -1
	ldr r0, =postfix_expr
	loop:
		add r4, r4, #1
		ldrb r1, [r0, r4] // r1 = postfix_expr[i]
		cbz r1, program_end
		cmp r1, ' '       // " "
		beq loop
		cmp r1, '+'       // "+"
		beq add
		cmp r1, '-'       // "-?"
		bne atoi
		add r5, r4, #1
		ldrb r6, [r0, r5] // r6 = postfix_expr[i+1]
		cbz r6, sub 	  // "-\0"
		cmp r6, ' '       // "- "
		beq sub
		b atoi            // "-[0-9]"

	program_end:
		pop {r2}
		ldr r3, =expr_result
		str r2, [r3]
		b program_end+6

add:
	pop {r2, r3}
	add r2, r2, r3
	push {r2}
	b loop

sub:
	pop {r2, r3}
	sub r2, r3, r2
	push {r2}
	b loop

atoi:
	mov r8, #10
	cmp r1, '-'
	ITTTE EQ
	moveq r9, #-1
	addeq r4, r4, #1
	ldrbeq r1, [r0, r4]
	movne r9, #1
	mov r2, #0

	atoi_loop:
		sub r1, r1, '0'
		mla r2, r2, r8, r1 // r2 = r2 * 10 + r1
		add r4, r4, #1
		ldrb r1, [r0, r4]
		cbz r1, atoi_return
		cmp r1, ' '
		beq atoi_return
		b atoi_loop

	atoi_return:
		mul r2, r2, r9
		push {r2}
		b loop
