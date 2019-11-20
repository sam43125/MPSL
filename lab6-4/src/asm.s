	.syntax unified
	.cpu cortex-m4
	.thumb

.text
.global Delay1sWith4MHz
	.equ X, 1350
	.equ Y, 100

Delay1sWith4MHz: //  2+(2+4*Y+3)*X ~= 4*X*Y * 0.25us
	push {r3,r4,lr}
	LDR R3, =X
	L1: LDR R4,	=Y
		L2: SUBS R4, #1
		BNE L2
		SUBS R3, #1
		BNE L1
	pop {r3,r4,pc}
