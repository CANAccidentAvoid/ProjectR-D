#include "stm32f4xx.h"
#define ARM_MATH_CM4

void GPIO_Init(void);
void TIM2_us_Delay(uint32_t delay); //TIM2 for generating 10us pulse for trig pin


uint32_t data;
double time,dist;

void GPIO_Init(){
	//Configuring PA5 for generating pulse sent to trig pin
	RCC->AHB1ENR |= 1; //Enable GPIOA clock (pn:242)
	GPIOA->MODER |= 1<<10; //Set the PA5 pin to output mode

	//Configuring output from echo pin to be sent to the board (PA6 pin)
	GPIOA->MODER &= ~(0x00003000); //Set PA6 to input mode
}


void TIM2_us_Delay(uint32_t delay){
	RCC->APB1ENR |=1; //Start the clock for the timer peripheral
	TIM2->ARR = (int)(delay/0.0625); // Total period of the timer
	TIM2->CNT = 0;
	TIM2->CR1 |= 1; //Start the Timer
	while(!(TIM2->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
	TIM2->SR &= ~(0x0001); //Reset the update interrupt flag
}

int main(){
	RCC->CFGR |= 0<<10; // set APB1 = 16 MHz
	GPIO_Init();
	GPIOA->BSRR = 0x00000000;// Setting trig pin to low to initialize the module

	while(1){
		//1. Sending 10us pulse to
		GPIOA->BSRR &= 0x00000000; //PA5 is low
		TIM2_us_Delay(2);
		GPIOA->BSRR |= 0x00000020;//PA5 set to High
		TIM2_us_Delay(10);// wait for 10us
		GPIOA->BSRR |= 0x00200000;// Make PA5 low again

		//2. Measure the pulse width of the pulse sent from the echo pin by polling IDR for port A
		while (GPIOA->IDR & 64){
			data = data+1;
		}

		//3.Converting the gathered data into distance in cm
		if (data>0){
			time = data*(0.0625*0.000001);
			dist = ((time*340)/2)*100;
		}

		TIM2_us_Delay(4);
		data = 0;
	}

}
