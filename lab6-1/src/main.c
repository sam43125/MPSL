#include "stm32l476xx.h"
#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}

extern void GPIO_init();
extern void Delay1sWith4MHz();
void SystemClock_Config(int freq){
	int M, N, R;
	if (freq == 1 || freq == 16) {
		//TODO: Change the SYSCLK source and set the corresponding Prescaler value.
		RCC->CR &= ~RCC_CR_PLLON;
		while(RCC_CR_PLLRDY == 0);
		SET_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN, 0);

		RCC->CR |= RCC_CR_MSION;// turn on MSI oscillator
		while((RCC->CR & RCC_CR_MSIRDY) == 0);//check MSI ready
		SET_REG(RCC->CR, RCC_CR_MSIRGSEL, 1 << 3);// 16Mhz
		SET_REG(RCC->CR, RCC_CR_MSIRANGE, RCC_CR_MSIRANGE_8);
		//SET_REG(RCC->BDCR, RCC_BDCR_LSEON, 1);
		//SET_REG(RCC->CSR, RCC_CSR_LSION, 1);
		//SET_REG(RCC->CR, RCC_CR_MSIPLLEN, 1 << 2); // Set the PLL part of the clock source
		SET_REG(RCC->CFGR, RCC_CFGR_HPRE, freq == 1 ? RCC_CFGR_HPRE_DIV16 : RCC_CFGR_HPRE_DIV1);//SYSCLK = 1Mhz or 16 Mhz
		SET_REG(RCC->CFGR, RCC_CFGR_MCOSEL, 0b010);
		SET_REG(RCC->CFGR, RCC_CFGR_SW, 0b00);
		while((RCC->CR & RCC_CR_MSIRDY) == 0);
		return;
	}
	else if (freq == 6) {
		M = 0b011; // /4
		N = 12;    // *12
		R = 0b11;  // /8
	}
	else if (freq == 10) {
		SystemClock_Config(1); // to deal with bug
		M = 0b011; // /4
		N = 10;    // *10
		R = 0b01;  // /4
	}
	else if (freq == 40) {
		M = 0b011; // /4
		N = 40;    // *40
		R = 0b01;  // /4
	}
	RCC->CR &= ~RCC_CR_PLLON;
	while(RCC_CR_PLLRDY == 0);
	SET_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN, 0);
	//SET_REG(RCC->CR, RCC_CR_MSIPLLEN, 1 << 2); // Set the PLL part of the clock source
	SET_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, RCC_PLLCFGR_PLLSRC_MSI); // Source 16Mhz
	SET_REG(RCC->CFGR, RCC_CFGR_MCOSEL, 0b101 << 24);
	SET_REG(RCC->CFGR, RCC_CFGR_SW, 0b11 << 0);
	SET_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
	SET_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, M << 4); // Divided by 4
	SET_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, N << 8); // *12
	SET_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLR, R << 25); // /8
	SET_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN, 1 << 24);
	RCC->CR |= RCC_CR_PLLON;
	while(RCC_CR_PLLRDY == 0);
}

int isPushed() {
	int ret = 0;
	while (!(GPIOC->IDR & 1 << 13))
		ret = 1;
	return ret;
}

int main(){
	int freqs[] = {1, 6, 10, 16, 40};
	int index = 0;
	SystemClock_Config(freqs[(index++) % 5]);
	GPIO_init();
	while(1){
		// make LED light
		GPIOA->BSRR = 1 << 5;

		if (isPushed())
			SystemClock_Config(freqs[(index++) % 5]);

		Delay1sWith4MHz();
		// make LED dark
		GPIOA->BRR = 1 << 5;

		if (isPushed())
			SystemClock_Config(freqs[(index++) % 5]);

		Delay1sWith4MHz();
	}
}
