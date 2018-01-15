
#include "stm32f0xx.h"


uint32_t prescaler_ms;


void first_motor(int ml){
	TIM3->CCR2 = 100;
//	for(int i = 0; i < 10000000; i++);
	TIM6delay_ms(ml*48/125 * 1000);
	TIM3->CCR2 = 0;
}
void second_motor(int ml){
	TIM3->CCR1 = 100;
	TIM6delay_ms(ml*48/125 * 1000);
	TIM3->CCR1 = 0;
}

void third_motor(int ml){
	TIM3->CCR3 = 100;
	TIM6delay_ms(ml*48/125 * 1000);
	TIM3->CCR3 = 0;
}
void fourth_motor(int ml){
	TIM3->CCR4 = 100;
	TIM6delay_ms(ml*48/125 * 1000);
	TIM3->CCR4 = 0;
}


void coctail_gilbert(){
	first_motor(80);
	fourth_motor(40);
	third_motor(40);
}

void coctail_maple_nut_shot(){
	second_motor(120);
}
void coctail_screw_driver(){
	first_motor(18);
	third_motor(50);
}
void coctail_vodka_tonic(){
	first_motor(20);
	fourth_motor(60);
}
void coctail_rum_rickie(){
	second_motor(20);
	fourth_motor(60);
}
void coctail_snake_bite(){
	second_motor(30);
	third_motor(40);
}
void coctail_water(){
	fourth_motor(50);
}
void coctail_juice(){
	third_motor(50);
}
void clean(){
	first_motor(15);
	second_motor(15);
	third_motor(15);
	fourth_motor(15);
}



void InitDelayTIM6(void)
	{
		prescaler_ms = SystemCoreClock / 1000; //prescaler for ms
		TIM6->PSC = prescaler_ms;
		RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // TIM6 clock enable
	}

void TIM6delay_ms(uint16_t value) //the argument is the value after which timer will stop
	{
		TIM6->PSC = prescaler_ms; //set of prescaler for ms
		TIM6->ARR = value; //after this value timer will stop
		TIM6->CNT = 0; //SET ZERO TO COUNT REGISER
		TIM6->SR &=~ TIM_SR_UIF;//reset flag (~ is not for bits)
		TIM6->CR1 |= TIM_CR1_CEN;//enable timer
		while((TIM6->SR & TIM_SR_UIF)==0){}
		TIM6->SR &=~ TIM_SR_UIF;//reset flag (~ is not for bits)
		TIM6->CR1 &= ~TIM_CR1_CEN;
	}



// USART1 interrupt request handler
// if something received through USART then call functions and turn on/off the LED
void USART1_IRQHandler(void){
	char chartoreceive = 0;
	if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		chartoreceive = (char)(USART1->RDR);
		switch(chartoreceive) {
			case 'g':
				coctail_gilbert();
				break;
			case 'm':
				coctail_maple_nut_shot();
				break;
			case 's':
					coctail_screw_driver();
					break;
			case 'v':
				coctail_vodka_tonic();
				break;
			case 'r':
				coctail_rum_rickie();
				break;
			case 'b':
				coctail_snake_bite();
				break;
			case 'w':
				coctail_water();
				break;
			case 'j':
				coctail_juice();
				break;
			case 'c':
				clean();
				break;
			default:
				break;
		}
	}
	else {
		NVIC_DisableIRQ(USART1_IRQn);
	}
}

// USART2 interrupt handler
// if something received through Bluetooth then call functions and turn on/off the LED
void USART2_IRQHandler(void){
	uint8_t chartoreceive = 0;
	if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		chartoreceive = (uint8_t)(USART2->RDR);
		USART2->TDR=chartoreceive;
		switch(chartoreceive) {
		case 'O':
			first_motor(10);
			second_motor(20);
			break;
		case 'B':
			GPIOC->ODR &= ~(1 << 6);
			break;
			default:
				break;
		}
	}
	else {
		NVIC_DisableIRQ(USART2_IRQn);
	}
}

// configures USART1
void init_USART_1(){
	//Setting AF for USART1 signals
		GPIOA->AFR[1] = (GPIOA->AFR[1] &~(GPIO_AFRH_AFRH1 |
					GPIO_AFRH_AFRH2)) | (1 << (1*4)) | (1 << (2 * 4));

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	// enable RCC USART1
	USART1->BRR = 480000 / 96;	// oversampling
	USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;	// USART RX ON, TX ON, USART ON,
	while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART1->ICR |= USART_ICR_TCCF;	// clear TC flag
	USART1->CR1 |= USART_CR1_RXNEIE;	// enable USART1 receive interruptions
	NVIC_SetPriority(USART1_IRQn, 0);	// set priority for USART1_IRQn
	NVIC_EnableIRQ(USART1_IRQn);	// enable USART1_IRQn
}

// configures USART2 (USART2 is connected to PA14 and PA15)
void init_USART_2(){
	//Setting AF for USART2 signals
		GPIOA->AFR[1] = (GPIOA->AFR[1] &~(GPIO_AFRH_AFRH6 |
				GPIO_AFRH_AFRH7)) | (1 << (6*4)) | (1 << (7 * 4));

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE |		// USART2 ON, TX ON, RX ON
					 USART_CR1_RXNEIE;
	USART2->BRR = 480000 / 384;
	while((USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
	USART2->ICR |= USART_ICR_TCCF;
}




int main(void) {
	// BLUETOOTH init

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;

	//	Select AF mode on pins
	GPIOA->MODER = (GPIOA->MODER &
			~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10| GPIO_MODER_MODER14 | GPIO_MODER_MODER15))\
			| (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1| GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);



	init_USART_1();
	init_USART_2();

	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART2_IRQn, 0);
	NVIC_EnableIRQ(USART2_IRQn);





	SystemInit();
	SystemCoreClockUpdate();
    InitDelayTIM6();

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;

	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 |
				GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;

	//Setting AF for USART 2
	GPIOA->MODER = (GPIOA->MODER &
				~(GPIO_MODER_MODER4|GPIO_MODER_MODER14 | GPIO_MODER_MODER15 | GPIO_MODER_MODER7))\
				| (GPIO_MODER_MODER4_1 |GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1 | GPIO_MODER_MODER7_1 );


	RCC ->APB1ENR |= RCC_APB1ENR_TIM3EN;


	//p166 manual Enabling alternative mode for pa7
	GPIOC->MODER |=  (1<<15);
	GPIOC->MODER &= ~(1<<14);
	//AF for PC6
	GPIOC->MODER |=  (1<<13);
	GPIOC->MODER &= ~(1<<12);
	//AF for PC8
	GPIOC->MODER |=  (1<<17);
	GPIOC->MODER &= ~(1<<16);

	//AF for PC9
	GPIOC->MODER |=  (1<<19);
	GPIOC->MODER &= ~(1<<18);

	/* Configure the Timer Channel 2 as PWM */
	/* (1) Configure the Timer x Channel 2 waveform (TIM1_CCMR1 register)
	is in PWM mode 1 (write OC2M = 110) */
	/* (2) Set TIMx prescaler to 2 */
	/* (3) Set TIMx Autoreload to 99 in order to get an overflow (so an UEV)
	each 10ms */
	/* (4) Set capture compare register to a value between 0 and 99 */

	//TIM3 CH2 PC7
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; /* (1) */
	TIM3->PSC = 0; /* (2) */
	TIM3->ARR = 100; /* (3) */
	TIM3->CCER |= (TIM_CCER_CC2E); // Capture/Compare 2 output enable -- PC7
	TIM3->CCER &= ~TIM_CCER_CC2P;

	TIM3->CR1  |= TIM_CR1_CEN; //Turn on the counter

//TIM3 CH1 PC6
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; /* (1) */
	TIM3->CCER |= (TIM_CCER_CC1E); // Capture/Compare 2 output enable -- PA7
	TIM3->CCER &= ~TIM_CCER_CC1P;

	//TIM3 CH3 PC8
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; /* (1) */
	TIM3->CCER |= (TIM_CCER_CC3E); // Capture/Compare 2 output enable -- PA7
	TIM3->CCER &= ~TIM_CCER_CC3P;

	//TIM3 CH4 PC8
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; /* (1) */
	TIM3->CCER |= (TIM_CCER_CC4E); // Capture/Compare 2 output enable -- PA7
	TIM3->CCER &= ~TIM_CCER_CC4P;

	int pow = 0;

	TIM3->CCR2 = pow	; /* (4) */
	TIM3->CCR1 = pow	; /* (4) */
	TIM3->CCR3 = pow;
	TIM3->CCR4 = pow;
	TIM6delay_ms(20);
//
//	first_motor(10);
//	second_motor(10);
//	third_motor(10);
//	fourth_motor(10);

	unsigned char i=0;


 int k = 1;
	while(1){

	}
}
