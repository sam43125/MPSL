#include "stm32l476xx.h"
#include <string.h>
#include <stdlib.h>
#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}
#define GPIO_AF_USART1 ((uint8_t)0x07)

void GPIO_Init(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN;
	// UART (PA9 PA10)
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE9, 0b10 << 18);
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE10, 0b10 << 20);
	SET_REG(GPIOA->AFR[1], GPIO_AFRL_AFRL1, GPIO_AF_USART1 << 4);
	SET_REG(GPIOA->AFR[1], GPIO_AFRL_AFRL2, GPIO_AF_USART1 << 8);
	// LED (PA5)
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE5, 0b01 << 10);
	// LED on breadboard (PC1)
	SET_REG(GPIOC->MODER, GPIO_MODER_MODE13, 0b00 <<26 );
	//GPIOC->BRR = 1 << 1;
	// Photoresistor ADC (PC0)
	SET_REG(GPIOC->MODER, GPIO_MODER_MODE0, 0b11 << 0);
	GPIOC->ASCR |= 1 << 0;
}

void ADC_Config(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

	// Clear Deep Sleep
	SET_REG(ADC1->CR, ADC_CR_DEEPPWD,0<<29);
	// Turn on Voltage Regulator
	SET_REG(ADC1->CR, ADC_CR_ADVREGEN,1<<28);
	SET_REG(ADC1->CR, ADC_CR_ADCAL,1<<31);
	SET_REG(ADC1->CFGR,ADC_CFGR_RES,0b00<<3);//00 for 12bit ,01 for 10bit
	//delay_us(200);
	// Prescaler
	SET_REG(ADC1->IER, ADC_IER_EOSIE ,1<<3);//End of regular sequence of conversions interrupt enable
	SET_REG(ADC123_COMMON->CCR, ADC_CCR_PRESC|ADC_CCR_CKMODE, ADC_CCR_CKMODE_0);
	SET_REG(ADC1->CFGR, ADC_CFGR_CONT,1<<13 );//continous mode
	SET_REG(ADC1->CFGR2, ADC_CFGR2_ROVSE,0<<0);//ADC oversampler disable
	SET_REG(ADC1->SQR1, ADC_SQR1_L,0); //1 conversion
	SET_REG(ADC1->SQR1, (0xF)|(0x1F<<24)|(0x1F<<18)|(0x1F<<12)|(0x1F<<6), (0x1<<6)); // Channel 1, Rank 1
	SET_REG(ADC1->SMPR1, (0x3FFFFFFFFF), (0x6<<3)); // Channel 1, Sampling Time: 247.5 ADC cycles
}
void startADC() {
	while (!(ADC1->ISR & ADC_ISR_ADRDY)) //wait for conversion start
		ADC1->CR |= ADC_CR_ADEN; // TURN ON
	//delay_us(5000);//go into interrupt handler
	SET_REG(ADC1->ISR , ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR,0b111<<2); // Clear flags
	SET_REG(ADC1->CR, ADC_CR_ADSTART,1<<2); // START CONV }
}
void ADC1_IRQHandler(){
	int value=0;
	char buffer [4]="";
	SET_REG(ADC1->CR, ADC_CR_ADCAL,1<<31);
	value=ADC1->DR;
	itoa (value,buffer,10);
	if(!(GPIOC->IDR & 1 << 13)){
		UART_Transmit(buffer,strlen(buffer));
		UART_Transmit("\r\n",2);
		while(!(GPIOC->IDR & 1 << 13));
	}


	SET_REG(ADC1->ISR , ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR,ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR); // Clear flags
}
void NVIC_config(){
	SET_REG(*(uint32_t *)0xE000E100, 0x0,1<<18 );
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

void UART_Receive(char *c) {
	while (!(USART1->ISR & USART_ISR_RXNE)); // RXNE: wait until received data is ready to be read
	*c = USART1->RDR;
}

int UART_Receive_Cmd(char *c, int i) {
	while (1) {
		UART_Receive(c + i);
		if (c[i] == '\r') {
			UART_Transmit("\r\n", 2);
			break;
		}
		else if (c[i] == '\177' && i == 0)
			continue;
		else
			UART_Transmit(c+i, 1);

		if(c[i] == '\177')
			i--;
		else
			i++;
	}
	return i;
}

int main() {

	GPIO_Init();
	USART1_Init();
	NVIC_config();
	ADC_Config();
	startADC();

	while (1);
	return 0;
}
