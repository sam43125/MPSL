#include "stm32l476xx.h"
#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}

// You can use your way to store TIME_SEC. Maybe it is `int` or `float` or any you want
#define TIME_SEC 16.8752
extern void GPIO_init();
extern void max7219_init();
extern void max7219_send(unsigned char address, unsigned char data);
static char digits[] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5b, 0x5F, 0x70, 0x7F, 0x7b};

int Display(int data) {
	// No decode mode
	unsigned char digit = 0;
	int i = 0;
	for (;; i++) {
		if (data == 0)
			break;
		digit = digits[data % 10];
		if (i == 2)
			digit |= 1 << 7;
		data /= 10;
		max7219_send(i+1, digit);
	}
	if (i < 1)
		max7219_send(2, digits[0]);
	if (i < 2)
		max7219_send(3, digits[0] | (1 << 7));
	return 0;
}

void Timer_init() {
	//TODO: Initialize timer
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	SET_REG(TIM2->CR1, TIM_CR1_DIR , 0 << 4);//up counter
	TIM2->ARR = (uint32_t)(TIME_SEC*100);//Reload value, [0, 1000000]
	TIM2->PSC = (uint32_t)(4000000/100 - 1);//Prescalser, CK_CNT = 100 Hz
	TIM2->EGR = TIM_EGR_UG;//Reinitialize the counter
}
void Timer_start() {
	TIM2->CR1 |= TIM_CR1_CEN;//start timer
}
int main() {
	GPIO_init();
	max7219_init();
	Timer_init();
	Timer_start();

	for (int i=1; i <= 8; i++)
		max7219_send(i, 0);

	int pre_val = 0;
	while(1) {
		int timerValue = TIM2->CNT;//polling the counter value
		if(pre_val > timerValue){  //check if overflow
			TIM2->CR1 &= ~TIM_CR1_CEN;
			return 0;
		}
		pre_val = timerValue;
		int dis_val = timerValue;//timerValue = actual seconds * 100
		Display(dis_val);//display the time on the 7-SEG LED
	}
	return 0;
}
