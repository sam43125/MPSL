#include "stm32l476xx.h"
#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}
extern void GPIO_init();
extern void Delay1sWith4MHz();
void SysTick_Handler() {
	if(GPIOA->ODR &1<<5)
		GPIOA->BRR = 1 << 5;
	else
		GPIOA->BSRR = 1 << 5;
	/*SET_REG(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN, 1);
	SET_REG(SYSCFG->EXTICR[1], SYSCFG_EXTICR1_EXTI3, 0);
	SET_REG(EXTI->IMR1, EXTI_IMR1_IM3, 1<<3);
	SET_REG(EXTI->EMR1, EXTI_EMR1_EM3, 0<<3);
	SET_REG(EXTI->RTSR1, EXTI_RTSR1_RT3, 1<<3);
	SET_REG(EXTI->FTSR1, EXTI_FTSR1_FT3, 0<<3);*/
}
void SystemClock_Config(void) {
	RCC->CR &= ~RCC_CR_PLLON;
	while(RCC_CR_PLLRDY == 0);
	SET_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN, 0);

	RCC->CR |= RCC_CR_HSION;// turn on MSI oscillator
	while((RCC->CR & RCC_CR_HSIRDY) == 0);//check MSI ready
	//SET_REG(RCC->CR, RCC_CR_MSIRGSEL, 1 << 3);// 16Mhz
	SET_REG(RCC->CR, RCC_CR_HSIKERON, 1<<9);
	//SET_REG(RCC->BDCR, RCC_BDCR_LSEON, 1);
	//SET_REG(RCC->CSR, RCC_CSR_LSION, 1);
	//SET_REG(RCC->CR, RCC_CR_MSIPLLEN, 1 << 2); // Set the PLL part of the clock source
	SET_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV16);//SYSCLK = 1Mhz or 16 Mhz
	SET_REG(RCC->CFGR, RCC_CFGR_MCOSEL, 0b011);
	SET_REG(RCC->CFGR, RCC_CFGR_SW, 0b01);
	SET_REG(*(uint32_t *)0xE000E010U, 0x7, 0b111);
	int reload_val=1000000/0.33 -1;
	SET_REG(*(uint32_t *)0xE000E014U, 0x7FFFFF, reload_val);
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	return;
}

int main(){
	SystemClock_Config();
	GPIO_init();
	while(1){

	}
}
