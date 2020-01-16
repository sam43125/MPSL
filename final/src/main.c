#include "stm32l476xx.h"
#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
extern void Delay1sWith4MHz();
void speed_init() {
	RCC->AHB2ENR = RCC->AHB2ENR|0x7;
	//Set PB5 as INPUT mode
	SET_REG(GPIOB->MODER,GPIO_MODER_MODE5 ,0b00<<10);
	//set PB5 is Pull-down inputGPIO_PUPDR_PUPD5
	SET_REG(GPIOB->PUPDR,GPIO_PUPDR_PUPD5 ,0b10<<10);
	//Set PB5 as high speed mode
	SET_REG(GPIOB->OSPEEDR,GPIO_OSPEEDR_OSPEED5 ,0b11<<10);
}
void led_test(){
	SET_REG(GPIOA->MODER,GPIO_MODER_MODE5 ,0b01<<10);
}
void EXTI_config(){
	SET_REG(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN, 1);

	SET_REG(SYSCFG->EXTICR[1], SYSCFG_EXTICR1_EXTI1, 0b001<<4);

	SET_REG(EXTI->IMR1, EXTI_IMR1_IM5, 1<<5);

	SET_REG(EXTI->EMR1, EXTI_EMR1_EM5, 0<<5);

	SET_REG(EXTI->RTSR1, EXTI_RTSR1_RT5, 1<<5);

	SET_REG(EXTI->FTSR1, EXTI_FTSR1_FT5, 0<<5);
}
void EXTI9_5_IRQHandler(void){
	TIM2->CR1 &= ~TIM_CR1_CEN;
	double time=TIM2->CNT/1000000.0;
	int speed=3.45/time *18/5; //meter per sec
	for(int i=1;i<4;i++){
		int data = speed%10;
		max7219_send(i,(char)data);
		speed=speed/10;
	}
	while(GPIOB->IDR&1<<5);
	Delay1sWith4MHz();
	Timer2_init();
	Timer2_start();
	SET_REG(EXTI->PR1, 1<<5,1<<5 );
}
int counter=0;
/*void EXTI9_5_IRQHandler(void){
	counter++;
	while(GPIOB->IDR&1<<5);
	Delay1sWith4MHz();
	SET_REG(EXTI->PR1, 1<<5,1<<5 );
}*/
void NVIC_config(){
	SET_REG(*(uint32_t *)0xE000E100, 0x0,1<<23 );
	SET_REG(*(uint32_t *)0xE000E100, 0x0,1<<28 );
	SET_REG(*(uint32_t *)0xE000E100, 0x0,1<<29 );
}

void Timer2_init(void){
	//TODO: Initialize timer
	TIM2->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	/*SET_REG(TIM2->CR1, TIM_CR1_UDIS, 0 << 1);    //set timer interrupt
	SET_REG(TIM2->CR1, TIM_CR1_URS, 1 << 2);	//set timer interrupt
	SET_REG(TIM2->DIER, TIM_DIER_UDE, 1 << 8);  //set timer interrupt
	SET_REG(TIM2->DIER, TIM_DIER_UIE, 1 << 0);  //set timer interrupt*/
	SET_REG(TIM2->CR1, TIM_CR1_DIR, 0 << 4);// Edge-aligned mode, up counter
	SET_REG(TIM2->CCMR1, TIM_CCMR1_OC1M, 0b110 << 4);/*!<OC1M[2:0] bits (Output Compare 1 Mode) */
	SET_REG(TIM2->CCMR1, TIM_CCMR1_OC1PE, 0b1 << 3);/*!<Output Compare 1 Preload enable */
	SET_REG(TIM2->CR1, TIM_CR1_ARPE, 0b1 << 7);/*!<Auto-reload preload enable */
	SET_REG(TIM2->CCER, TIM_CCER_CC1P, 0b1 << 1); /*!<Capture/Compare 1 output Polarity */
	SET_REG(TIM2->CCER, TIM_CCER_CC1E, 0b1);/*!<Capture/Compare 1 output enable */
	SET_REG(TIM2->CCR1, TIM_CCR1_CCR1, 20000000/ 2);
	TIM2->ARR = (uint32_t)20000000;//Reload value 10s explosion
	TIM2->PSC = (uint32_t)1;//Prescaler /2 = 2MHz
	//TIM2->ARR = (uint32_t)4000000;//Reload value 1s explosion
	//TIM2->PSC = (uint32_t)0;//Prescaler  = 4MHz
	TIM2->EGR = TIM_EGR_UG;//Reinitialize the counter
}
void Timer3_init(void){
	//TODO: Initialize timer
	TIM3->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
	SET_REG(TIM3->CR1, TIM_CR1_UDIS, 0 << 1);    //set timer interrupt
	SET_REG(TIM3->CR1, TIM_CR1_URS, 1 << 2);	//set timer interrupt
	SET_REG(TIM3->DIER, TIM_DIER_UDE, 1 << 8);  //set timer interrupt
	SET_REG(TIM3->DIER, TIM_DIER_UIE, 1 << 0);  //set timer interrupt
	SET_REG(TIM3->CR1, TIM_CR1_DIR, 0 << 4);// Edge-aligned mode, up counter
	SET_REG(TIM3->CCMR1, TIM_CCMR1_OC1M, 0b110 << 4);/*!<OC1M[2:0] bits (Output Compare 1 Mode) */
	SET_REG(TIM3->CCMR1, TIM_CCMR1_OC1PE, 0b1 << 3);/*!<Output Compare 1 Preload enable */
	SET_REG(TIM3->CR1, TIM_CR1_ARPE, 0b1 << 7);/*!<Auto-reload preload enable */
	SET_REG(TIM3->CCER, TIM_CCER_CC1P, 0b1 << 1); /*!<Capture/Compare 1 output Polarity */
	SET_REG(TIM3->CCER, TIM_CCER_CC1E, 0b1);/*!<Capture/Compare 1 output enable */
	//SET_REG(TIM3->CCR1, TIM_CCR1_CCR1, 20000000/ 2);
	//TIM3->ARR = (uint32_t)20000000;//Reload value 10s explosion
	//TIM3->PSC = (uint32_t)1;//Prescaler /2 = 2MHz
	TIM3->ARR = (uint32_t)10000;//Reload value 1s explosion
	TIM3->PSC = (uint32_t)399;//Prescaler  = 4MHz/400
	TIM3->EGR = TIM_EGR_UG;//Reinitialize the counter
}
void Direction_light_init(void){
	RCC->AHB2ENR = RCC->AHB2ENR|0x7;
	//Set PC01 as INPUT mode
	SET_REG(GPIOC->MODER,GPIO_MODER_MODE0 ,0b00<<0);
	SET_REG(GPIOC->MODER,GPIO_MODER_MODE1 ,0b00<<2);
	//set PC01 is Pull-down inputGPIO_PUPDR_PUPD5
	SET_REG(GPIOC->PUPDR,GPIO_PUPDR_PUPD0 ,0b10<<0);
	SET_REG(GPIOC->PUPDR,GPIO_PUPDR_PUPD1 ,0b10<<2);
	//Set PC01 as high speed mode
	SET_REG(GPIOC->OSPEEDR,GPIO_OSPEEDR_OSPEED0 ,0b11<0);
	SET_REG(GPIOC->OSPEEDR,GPIO_OSPEEDR_OSPEED1 ,0b11<2);
	//set PC23 output mode
	SET_REG(GPIOC->MODER,GPIO_MODER_MODE2 ,0b01<<4);
	SET_REG(GPIOC->MODER,GPIO_MODER_MODE3 ,0b01<<6);
}
void Direction_light_detect(void){
	if(!(TIM3->SR &1))
		if((GPIOC->IDR&1<<0)||(GPIOC->IDR&1<<1))
			Timer3_start();
		else
			GPIOC->BRR|=0b11<<2;
}
/*void TIM2_IRQHandler(void){
	int speed = counter*3.45*18/5;
	for(int i=1;i<4;i++){
		int data = speed%10;
		max7219_send(i,(char)data);
		speed=speed/10;
	}
	counter=0;
	SET_REG(TIM2->SR,0x1, 0); //clear interrupt pending
}*/
void TIM3_IRQHandler(void){
	TIM3->CR1 &= ~TIM_CR1_CEN;
	if(GPIOC->ODR&1<<2||GPIOC->ODR&1<<3)
		GPIOC->BRR|=0b11<<2;//turn off the light
	else if(GPIOC->IDR&1<<0)
		GPIOC->BSRR|=1<<2;
	else if(GPIOC->IDR&1<<1)
		GPIOC->BSRR|=1<<3;
	Timer3_init();
	SET_REG(TIM3->SR,0x1, 0); //clear interrupt pending

}
void Timer2_start(void){
	TIM2->CR1 |= TIM_CR1_CEN;
}
void Timer3_start(void){
	TIM3->CR1 |= TIM_CR1_CEN;
}
void GPIO_init_AF(){
	//TODO: Initial PB7 as alternate function for buzzer.
	RCC->AHB2ENR |= 0b111;
	SET_REG(GPIOB->MODER, 0b11<<14, 0b10<<14);
	SET_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL7, 0x02<<28);
}
void Timer4_init(){
	//TODO: Initialize timer
	TIM4->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
	SET_REG(TIM4->CR1, TIM_CR1_DIR, 0 << 4);// Edge-aligned mode, up counter
	SET_REG(TIM4->CCMR1, TIM_CCMR1_OC2M, 0b110 << 12);/*!<OC1M[2:0] bits (Output Compare 1 Mode) */
	SET_REG(TIM4->CCMR1, TIM_CCMR1_OC2PE, 0b1 << 11);/*!<Output Compare 1 Preload enable */
	SET_REG(TIM4->CR1, TIM_CR1_ARPE, 0b1 << 7);/*!<Auto-reload preload enable */
	SET_REG(TIM4->CCER, TIM_CCER_CC2P, 0b1 << 5); /*!<Capture/Compare 1 output Polarity */
	SET_REG(TIM4->CCER, TIM_CCER_CC2E, 0b1<<4);/*!<Capture/Compare 1 output enable */
	//SET_REG(TIM4->CCR1, TIM_CCR1_CCR1, 382 / 2);
	SET_REG(TIM4->CCR2, TIM_CCR2_CCR2, 382 / 2);
	TIM4->ARR = (uint32_t)382;//Reload value
	TIM4->PSC = (uint32_t)40;//Prescaler
	TIM4->EGR = TIM_EGR_UG;//Reinitialize the counter
}
int main(){
	speed_init();
	//led_test();
	EXTI_config();
	NVIC_config();
	Timer2_init();
	Timer2_start();
	max7219_init();
	Timer3_init();
	Timer4_init();
	GPIO_init_AF();
	Direction_light_init();
	TIM4->CR1 |= TIM_CR1_CEN;
	while(1){

		Direction_light_detect();
	}
	return 0;
}
