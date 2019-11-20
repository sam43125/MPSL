#include "stm32l476xx.h"
#include <unistd.h>
extern void Delay1sWith4MHz();

#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}

//extern void GPIO_init();
int TableFreq[4][4] = {{382, 340, 303, 0}, {286, 255, 227, 0}, {202, 191, 0, 0}, {-1, 0, 1, 0}};
int rows[] = {8,9,10,12};
int cols[] = {5,6,7,9};
#define PRESS 1
#define NOT_PRESS 0

void GPIO_init_AF(){
	//TODO: Initial GPIO pin as alternate function for buzzer. You can choose to use C or assembly to finish this function.
	RCC->AHB2ENR |= 0b111;
	SET_REG(GPIOA->MODER, 0b11 << 0, 0b10 << 0);
	SET_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL0, 1);
}
void Timer_init(int CCR){
	//TODO: Initialize timer
	TIM2->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	SET_REG(TIM2->CR1, TIM_CR1_DIR, 0 << 4);// Edge-aligned mode, up counter
	SET_REG(TIM2->CCMR1, TIM_CCMR1_OC1M, 0b110 << 4);/*!<OC1M[2:0] bits (Output Compare 1 Mode) */
	SET_REG(TIM2->CCMR1, TIM_CCMR1_OC1PE, 0b1 << 3);/*!<Output Compare 1 Preload enable */
	SET_REG(TIM2->CR1, TIM_CR1_ARPE, 0b1 << 7);/*!<Auto-reload preload enable */
	SET_REG(TIM2->CCER, TIM_CCER_CC1P, 0b1 << 1); /*!<Capture/Compare 1 output Polarity */
	SET_REG(TIM2->CCER, TIM_CCER_CC1E, 0b1);/*!<Capture/Compare 1 output enable */
	SET_REG(TIM2->CCR1, TIM_CCR1_CCR1, CCR);
	TIM2->ARR = (uint32_t)100;//Reload value
	TIM2->PSC = (uint32_t)0;//Prescaler
	TIM2->EGR = TIM_EGR_UG;//Reinitialize the counter
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

void SystemClock_Config() {
	RCC->CR |= RCC_CR_MSION;// turn on MSI oscillator
	while((RCC->CR & RCC_CR_MSIRDY) == 0);//check MSI ready
	SET_REG(RCC->CR, RCC_CR_MSIRGSEL, 1 << 3);// 100Khz
	SET_REG(RCC->CR, RCC_CR_MSIRANGE, RCC_CR_MSIRANGE_0);
	SET_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
	SET_REG(RCC->CFGR, RCC_CFGR_MCOSEL, 0b010);
	SET_REG(RCC->CFGR, RCC_CFGR_SW, 0b00);
	while((RCC->CR & RCC_CR_MSIRDY) == 0);
	return;
}

int main(){
	GPIO_init_AF();
	keypad_init();
	//SystemClock_Config();
	//TODO: Scan the keypad and use PWM to send the corresponding frequency square wave to buzzer.
	int press_stat = NOT_PRESS, befoer_val = -1, CCR = 50;
	Timer_init(CCR);
	TIM2->CR1 |= TIM_CR1_CEN;

	while(1){
		for(int i=0; i<4; i++) {
			GPIOA->ODR=(GPIOA->ODR&0xFFFFE8FF)|1<<rows[i];
			for(int j = 0; j < 4; j++) {
				int flag_keypad_r=GPIOB->IDR&1<<cols[j];
				if(flag_keypad_r != 0) {
					if(TableFreq[j][i]==-1) {
						CCR = CCR >= 15 ? CCR - 5 : CCR;
						Timer_init(CCR);
						TIM2->CR1 |= TIM_CR1_CEN;
					}
					if(TableFreq[j][i]==1) {
						CCR = CCR <= 85 ? CCR + 5 : CCR;
						Timer_init(CCR);
						TIM2->CR1 |= TIM_CR1_CEN;
					}
				}
			}
		}/*
		if (press_stat == NOT_PRESS) {
			TIM2->CR1 &= ~TIM_CR1_CEN;
			befoer_val = -1;
		}
		press_stat = NOT_PRESS;*/
		Delay1sWith4MHz();
	}
	return 0;
}
