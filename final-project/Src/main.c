/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sd_hal_mpu6050.h"
#include "stm32l476xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT))) | (VAL));}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
// SD_MPU6050 mpu1;
// int16_t a_x, a_y, a_z;
// unsigned long long total;
// uint32_t previous_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
extern void Delay1sWith4MHz();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void speed_init() {
	// 光遮斷器
	RCC->AHB2ENR = RCC->AHB2ENR|0x7;
	//Set PB5 as INPUT mode
	SET_REG(GPIOB->MODER,GPIO_MODER_MODE5 ,0b00<<10);
	//set PB5 is Pull-down input
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
void EXTI9_5_IRQHandler(void) {
	// 光遮斷器被遮斷時
	TIM2->CR1 &= ~TIM_CR1_CEN;
	double time=TIM2->CNT/1000000.0;
	/*if (previous_cnt - TIM2->CNT < -500000 && !(GPIOB->IDR&1<<0) && !(GPIOC->IDR&1<<0) && !(GPIOC->IDR&1<<1))
		GPIOC->BSRR = 0b11 << 2;
	else if (previous_cnt = TIM2->CNT && !(GPIOB->IDR&1<<0) && !(GPIOC->IDR&1<<0) && !(GPIOC->IDR&1<<1))
		GPIOC->BRR = 0b11 << 2;
	previous_cnt = TIM2->CNT;*/
	int speed=3.45/time *18/5; //meter per sec
	for(int i=1;i<3;i++){
		int data = speed%10;
		max7219_send(i,(char)data);
		speed=speed/10;
	}
	max7219_send(3, 15);
	max7219_send(4, 15);
	max7219_send(6, 15);
	max7219_send(7, 15);
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
	// 時速計時
	//Initialize timer
	TIM2->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	SET_REG(TIM2->CR1, TIM_CR1_UDIS, 0 << 1);    //set timer interrupt
	SET_REG(TIM2->CR1, TIM_CR1_URS, 1 << 2);	//set timer interrupt
	SET_REG(TIM2->DIER, TIM_DIER_UDE, 1 << 8);  //set timer interrupt
	SET_REG(TIM2->DIER, TIM_DIER_UIE, 1 << 0);  //set timer interrupt
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
	// 方向燈每秒閃一次
	//Initialize timer
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
	// PC01 方向燈三段式按鈕
	// PC23 左右方向燈
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
	if(!(TIM3->SR &1)) {
		if((GPIOC->IDR&1<<0)||(GPIOC->IDR&1<<1)||(GPIOB->IDR&1<<0))
			Timer3_start();
		else
			GPIOC->BRR|=0b11<<2;
	}
}
void TIM2_IRQHandler(void){
	/*int speed = counter*3.45*18/5;
	for(int i=1;i<4;i++){
		int data = speed%10;
		max7219_send(i,(char)data);
		speed=speed/10;
	}
	counter=0;*/
	for(int i=1;i<=2;i++){
		max7219_send(i,(char)0);
	}
	SET_REG(TIM2->SR,0x1, 0); //clear interrupt pending
}
void TIM3_IRQHandler(void){
	// 判斷哪個方向燈要閃
	TIM3->CR1 &= ~TIM_CR1_CEN;
	if(GPIOC->ODR&1<<2||GPIOC->ODR&1<<3)
		GPIOC->BRR|=0b11<<2;//turn off the light
	else if(GPIOB->IDR&1<<0) {
		GPIOC->BSRR|=0b11<<2;
		max7219_send(5, 10);
		max7219_send(8, 10);
	}
	else if(GPIOC->IDR&1<<0) {
		GPIOC->BSRR|=1<<2;
		max7219_send(5, 15);
		max7219_send(8, 10);
	}
	else if(GPIOC->IDR&1<<1) {
		GPIOC->BSRR|=1<<3;
		max7219_send(5, 10);
		max7219_send(8, 15);
	}
	else {
		max7219_send(5, 15);
		max7219_send(8, 15);
	}
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
	//Initial PB7 as alternate function for buzzer.
	RCC->AHB2ENR |= 0b111;
	SET_REG(GPIOB->MODER, 0b11<<14, 0b10<<14);
	SET_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL7, 0x02<<28);
}
void Timer4_init(){
	// 蜂鳴器
	//Initialize timer
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
void double_light_init() {
	// 雙黃燈
	RCC->AHB2ENR = RCC->AHB2ENR|0x7;
	//Set PB0 as INPUT mode
	SET_REG(GPIOB->MODER,GPIO_MODER_MODE0 ,0b00<<0);
	//set PB0 is Pull-down input
	SET_REG(GPIOB->PUPDR,GPIO_PUPDR_PUPD0 ,0b10<<0);
	//Set PB0 as high speed mode
	SET_REG(GPIOB->OSPEEDR,GPIO_OSPEEDR_OSPEED0 ,0b11<<0);
}
void GPIO_init_ButtonANDBuzzer() {
	RCC->AHB2ENR = RCC->AHB2ENR|0x1;
	//Set PA10 as INPUT mode
	SET_REG(GPIOA->MODER,GPIO_MODER_MODE10 ,0b00<<20);
	//set PA10 is Pull-down input
	SET_REG(GPIOA->PUPDR,GPIO_PUPDR_PUPD10 ,0b10<<20);
}
/*
void ReadAcc(int16_t *x, int16_t *y, int16_t *z, unsigned long long *total) {
	SD_MPU6050_ReadAccelerometer(&hi2c2,&mpu1);
	*x = mpu1.Accelerometer_X - _x;
	*y = mpu1.Accelerometer_Y - _y;
	*z = mpu1.Accelerometer_Z - _z;
	*total = (*x)*(*x) + (*y)*(*y) + (*z)*(*z);
}
*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // SD_MPU6050_Result result;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
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
  double_light_init();
  GPIO_init_ButtonANDBuzzer();
  TIM4->CR1 |= TIM_CR1_CEN;
  // result = SD_MPU6050_Init(&hi2c2,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_4G,SD_MPU6050_Gyroscope_250s);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(10);
    max7219_send(3, 15);
	max7219_send(4, 15);
	max7219_send(6, 15);
	max7219_send(7, 15);
    /* USER CODE END WHILE */
	// SD_MPU6050_ReadTemperature(&hi2c2,&mpu1);
	// float temper = mpu1.Temperature;
    /*
	SD_MPU6050_ReadGyroscope(&hi2c2,&mpu1);
	int16_t g_x = mpu1.Gyroscope_X;
	int16_t g_y = mpu1.Gyroscope_Y;
	int16_t g_z = mpu1.Gyroscope_Z;
	*/
    /* USER CODE BEGIN 3 */
    // SD_MPU6050_ReadAccelerometer(&hi2c2,&mpu1);
    // int16_t a_x = mpu1.Accelerometer_X;
 	// int16_t a_y = mpu1.Accelerometer_Y;
    // int16_t a_z = mpu1.Accelerometer_Z;
	// ReadAcc(&a_x, &a_y, &a_z, &total);
	// if (a_y > -1000 && a_y < 1000)
	// 	a_y = 0;
    if (GPIOA->IDR & 1<<10) {
    	SET_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL7, 0x02<<28);
    }
    else {
    	SET_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL7, 0x00<<28);
    }
	Direction_light_detect();
	/*
	if (a_y < 0)
		max7219_send(8, (char)10); // -
	else
		max7219_send(8, (char)15); // blank
	int i;
	for(i=3;i<8;i++){
		int16_t data = a_y % 10;
		data = data < 0 ? -data : data;
		max7219_send(i,(char)data);
		a_y /= 10;
	}
	*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000004;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
