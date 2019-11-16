#include "stm32l476xx.h"

extern void GPIO_init();
extern void max7219_init();
extern void max7219_send(unsigned char address, unsigned char data);

#define X0 8 // PA
#define X1 9
#define X2 10
#define X3 12
#define Y0 5 // PB
#define Y1 6
#define Y2 7
#define Y3 9
unsigned int x_pin[4] = {X0, X1, X2, X3};
unsigned int y_pin[4] = {Y0, Y1, Y2, Y3};
unsigned int Table[4][4] = {
		{1, 2, 3, 10},
		{4, 5, 6, 11},
		{7, 8, 9, 12},
		{15, 0, 14, 13}
};
unsigned int isPushed[4][4];

int display(int data, int num_digs) {
	if (data > 99999999)
		return -1;
	for (int i=0; i < num_digs; i++) {
		unsigned char digit = data % 10;
		data /= 10;
		max7219_send(i+1, digit);
	}
	return 0;
}

void clear_display() {
	for (int i=1; i <= 8; i++)
		max7219_send(i, 15);
}

void isPushed_init() {
	for (int i=0; i < 4; i++)
		for (int j=0; j < 4; j++)
			isPushed[i][j] = 0;
}

/* TODO: initial keypad gpio pin, X as output and Y as input */
void keypad_init() {
	RCC->AHB2ENR |= 0b11; // Port AB
	// Set keypad gpio OUTPUT
	// Set PA8,9,10,12 as output mode(01)
	GPIOA->MODER &= 0xFDD5FFFF;
	// Set PA8,9,10,12 as Pull-down output(00)
	GPIOA->PUPDR |= /*0x1150000*/ 0x22A0000;
	// Set PA8,9,10,12 as medium speed mode
	GPIOA->OSPEEDR |= 0x1150000;
	// Set PA8,9,10,12 as high
	GPIOA->ODR=GPIOA->ODR|0b10111<<8;

	// Set keypad gpio INPUT
	// Set PB5,6,7,9 as INPUT mode(00)
	GPIOB->MODER &= 0xFFF303FF;
	// Set PB5,6,7,9 is Pull-down input(10)
	GPIOB->PUPDR |= 0x8A800;
	// Set PB5,6,7,9 as medium speed mode(01)
	GPIOB->OSPEEDR |= 0x45400;
}

char keypad_scan() {
	int sum = 255;
	GPIOA->BSRR = 0b10111 << 8; // Set PA 8 9 10 12 as high
	int flag_keypad=GPIOB->IDR&0b10111<<5;
	int flag_debounce;
	if(flag_keypad!=0){
		int k=45000;
		while(k!=0){
			flag_debounce=GPIOB->IDR&0b10111<<5;
			k--;
		}
		if(flag_debounce!=0){
			for(int i=0;i<4;i++){ //scan keypad from first column
				//set PA8,9,10,12(column) low and set pin high from PA8
				GPIOA->ODR=(GPIOA->ODR&0xFFFFE8FF)|1<<x_pin[i];
				for(int j=0;j<4;j++){ //read input from first row
					int flag_keypad_r=GPIOB->IDR&1<<y_pin[j];
					if(flag_keypad_r!=0)
						isPushed[j][i] = 1;
				}
			}
			GPIOA->BRR = 0b10111 << 8;
			GPIOA->MODER &= 0xFCC0FFFF; // Set PA8 9 10 12 as input
			GPIOB->MODER = (GPIOB->MODER & 0xFFF303FF) | (0b0100010101 << 10);
			GPIOB->BRR = 0b10111 << 5;
			for(int i=0;i<4;i++){
				GPIOB->BSRR = 1 << y_pin[i];
				for(int j=0;j<4;j++){
					int flag_keypad_r=GPIOA->IDR&1<<x_pin[j];
					if(flag_keypad_r!=0)
						isPushed[i][j] = 1;
				}
				GPIOB->BRR = 1 << y_pin[i];
			}
			GPIOA->MODER = (GPIOA->MODER & 0xFCC0FFFF) | (0b0100010101 << 16);
			GPIOB->MODER &= 0xFFF303FF;
		}
	}
	for (int i=0; i < 4; i++)
		for (int j=0; j < 4; j++)
			if (isPushed[i][j]) {
				if (sum == 255)
					sum = Table[i][j];
				else
					sum += Table[i][j];
				isPushed[i][j] = 0;
			}

	return sum;
}

int main() {
	GPIO_init();
	max7219_init();
	keypad_init();
	isPushed_init();
	while (1) {
		char sum = keypad_scan();
		clear_display();
		if (sum != 255)
			display(sum, sum > 9 ? 2 : 1);
	}
	return 0;
}
