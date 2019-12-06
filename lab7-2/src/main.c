#include "stm32l476xx.h"
#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}
extern void GPIO_init();
extern void Delay1sWith4MHz();
extern void Delay20msWith4MHz();
int rows[] = {8,9,10,12};
int cols[] = {5,6,7,9};
unsigned int Table[4][4]={{1,2,3,10},{4,5,6,11},{7,8,9,12},{15,0,14,13}};

int i = 0;

void EXTI_config(){
	SET_REG(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN, 1);

	SET_REG(SYSCFG->EXTICR[1], SYSCFG_EXTICR1_EXTI1, 0b001<<4);
	SET_REG(SYSCFG->EXTICR[1], SYSCFG_EXTICR1_EXTI2, 0b001<<8);
	SET_REG(SYSCFG->EXTICR[1], SYSCFG_EXTICR1_EXTI3, 0b001<<12);
	SET_REG(SYSCFG->EXTICR[2], SYSCFG_EXTICR1_EXTI1, 0b001<<4);

	SET_REG(EXTI->IMR1, EXTI_IMR1_IM5, 1<<5);
	SET_REG(EXTI->IMR1, EXTI_IMR1_IM6, 1<<6);
	SET_REG(EXTI->IMR1, EXTI_IMR1_IM7, 1<<7);
	SET_REG(EXTI->IMR1, EXTI_IMR1_IM9, 1<<9);

	SET_REG(EXTI->EMR1, EXTI_EMR1_EM5, 0<<5);
	SET_REG(EXTI->EMR1, EXTI_EMR1_EM6, 0<<6);
	SET_REG(EXTI->EMR1, EXTI_EMR1_EM7, 0<<7);
	SET_REG(EXTI->EMR1, EXTI_EMR1_EM9, 0<<9);

	SET_REG(EXTI->RTSR1, EXTI_RTSR1_RT5, 1<<5);
	SET_REG(EXTI->RTSR1, EXTI_RTSR1_RT6, 1<<6);
	SET_REG(EXTI->RTSR1, EXTI_RTSR1_RT7, 1<<7);
	SET_REG(EXTI->RTSR1, EXTI_RTSR1_RT9, 1<<9);

	SET_REG(EXTI->FTSR1, EXTI_FTSR1_FT5, 0<<5);
	SET_REG(EXTI->FTSR1, EXTI_FTSR1_FT6, 0<<6);
	SET_REG(EXTI->FTSR1, EXTI_FTSR1_FT7, 0<<7);
	SET_REG(EXTI->FTSR1, EXTI_FTSR1_FT9, 0<<9);
}

void NVIC_config(){
	SET_REG(*(uint32_t *)0xE000E100, 0x0,1<<23 );
}

void EXTI9_5_IRQHandler(void){
	for(int j=0;j<4;j++){
		if((EXTI->PR1>>cols[j])&1){
			for(int k=0;k<Table[j][i];k++){
				GPIOA->BRR = 1 << 5;
				Delay1sWith4MHz();
				GPIOA->BSRR = 1 << 5;
				Delay1sWith4MHz();
			}
			SET_REG(EXTI->PR1, 1<<cols[j],1<<cols[j] );
		}
	}
}

void keypad_init() { // SET keypad gpio OUTPUT //
	RCC->AHB2ENR = RCC->AHB2ENR|0x7;
	//Set PA8,9,10,12 as output mode
	GPIOA->MODER= GPIOA->MODER&0xFDD5FFFF;
	//set PA8,9,10,12 is Pull-up output
	GPIOA->PUPDR=GPIOA->PUPDR|0x1150000;
	//Set PA8,9,10,12 as medium speed mode
	GPIOA->OSPEEDR=GPIOA->OSPEEDR|0x1150000;
	//Set PA8,9,10,12 as high
	GPIOA->ODR=GPIOA->ODR|0b10111<<8;
	// SET keypad gpio INPUT //
	//Set PB5,6,7,9 as INPUT mode
	GPIOB->MODER=GPIOB->MODER&0xFFF303FF;
	//set PB5,6,7,9 is Pull-down input
	GPIOB->PUPDR=GPIOB->PUPDR|0x8A800;
	//Set PB5,6,7,9 as medium speed mode
	GPIOB->OSPEEDR=GPIOB->OSPEEDR|0x45400;
}

int main(){
	keypad_init();
	GPIO_init();
	EXTI_config();
	NVIC_config();
	TIM2->CR1 |= TIM_CR1_CEN;
	GPIOA->BSRR = 1 << 5;
	while(1){
		for( i=0; i<4; i++) {
			GPIOA->ODR=(GPIOA->ODR&0xFFFFE8FF)|1<<rows[i];
			Delay20msWith4MHz(); //Debounce
		}
	}
	return 0;
}
