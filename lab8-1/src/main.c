#include "stm32l476xx.h"
#include <string.h>
#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}
#define GPIO_AF_USART1 ((uint8_t)0x07)

void GPIO_Init(void) {
	// PA9 PA10 PC13
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN;
	// UART
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE9, 0b10 << 18);
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE10, 0b10 << 20);
	SET_REG(GPIOA->AFR[1], GPIO_AFRL_AFRL1, GPIO_AF_USART1 << 4);
	SET_REG(GPIOA->AFR[1], GPIO_AFRL_AFRL2, GPIO_AF_USART1 << 8);
	// BUTTON
	SET_REG(GPIOC->MODER, GPIO_MODER_MODE13, 0b00 << 26);
}

void USART1_Init(void) {
	/* Enable clock for USART1 */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// CR1
	// 1 Start bit, 8 data bits, n stop bits
	// Even parity
	// Parity control disabled
	// Transmitter is enabled
	// Receiver is enabled and begins searching for a start bit
	// Oversampling by 16
	SET_REG(USART1->CR1, USART_CR1_M | USART_CR1_PS | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE |
	USART_CR1_OVER8, USART_CR1_TE | USART_CR1_RE);
	// CR2
	SET_REG(USART1->CR2, USART_CR2_STOP, 0x0 << 12); // 1-bit stop
	// CR3
	// CTS hardware flow control disabled
	// RTS hardware flow control disabled
	// Three sample bit method
	SET_REG(USART1->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT), 0x0); // none hwflowctl
	// Baudrate = 9600 bits per second
	SET_REG(USART1->BRR, 0xFF, 4000000 / 9600);
	/* In asynchronous mode, the following bits must be kept cleared:
	- LINEN and CLKEN bits in the USART_CR2 register,
	- SCEN, HDSEL and IREN bits in the USART_CR3 register.*/
	// LIN mode disabled
	// CK pin disabled
	USART1->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
	// Smartcard Mode disabled
	// Half duplex mode is not selected
	// IrDA disabled
	USART1->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
	// Enable UART
	USART1->CR1 |= (USART_CR1_UE);
}

void UART_Transmit(char *arr, uint32_t size) {
	for (int i = 0; i < size; i++) {
		while (!(USART1->ISR & USART_ISR_TXE)); // TXE: wait until data is transferred to the shift register (TDR)
		USART1->TDR = arr[i];
	}
	while (!(USART1->ISR & USART_ISR_TC));     // TC: wait until transmission is complete
}

int main() {
	GPIO_Init();
	USART1_Init();
	char *msg = "Hello World!";
	while (1) {
		while (GPIOC->IDR & 1 << 13);
		UART_Transmit(msg, strlen(msg));
		while (!(GPIOC->IDR & 1 << 13));
	}
	return 0;
}
