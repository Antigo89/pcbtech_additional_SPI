/*
File    : main.c
Software "Kurs STM32 PCBtech"
Lesson 6: SPI2 connect to SPI1.
Student: antigo1989@gmail.com
*/

#include "main.h"

/***************************inline function*******************************/


//global values
volatile uint16_t rx_buf = 0x00;
//volatile uint16_t tx_buf = 0x00;

/*********************************main************************************/
int main(void) {
  //Values
  
  //System init
  SystemInit();
  RCC_Init();
  __enable_irq();
  //GPIO init
  key_led_Init();
  //Connections init
  SPI1_Init();
  SPI2_Init();
 

  while(1){
    if(rx_buf > 0){
      switch(rx_buf){
        case LED1:
          GPIOE->BSRR |= GPIO_BSRR_BR13|GPIO_BSRR_BS14|GPIO_BSRR_BS15;
          break;
        case LED2:
          GPIOE->BSRR |= GPIO_BSRR_BR14|GPIO_BSRR_BS13|GPIO_BSRR_BS15;
          break;
        case LED3:
          GPIOE->BSRR |= GPIO_BSRR_BR15|GPIO_BSRR_BS13|GPIO_BSRR_BS14;
          break;
        case CLEAR:
          GPIOE->BSRR |= GPIO_BSRR_BS13|GPIO_BSRR_BS14|GPIO_BSRR_BS15;
          break;
      }
      rx_buf = 0;
    }
  __NOP();
  }
}

/***********************interrupts function**************************/
//keys
void EXTI15_10_IRQHandler(void){
  switch(EXTI->PR & (EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12)){
      case EXTI_PR_PR10:
        send_SPI2(LED1);
        break;
      case EXTI_PR_PR11:
        send_SPI2(LED2);
        break;
      case EXTI_PR_PR12:
        send_SPI2(LED3);
        break;
  } 
  EXTI->PR |= EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12;
}

void SPI1_IRQHandler(void){
  if((SPI1->SR & SPI_SR_RXNE) == 1){
    rx_buf = SPI1->DR;
  }
  NVIC_ClearPendingIRQ(SPI1_IRQn);
}
/****************************** function**********************************/
void send_SPI2(uint16_t data){
  SPI2->DR = (data & 0xFFFF);
  while((SPI2->SR & SPI_SR_TXE) == 0){}
  while((SPI2->SR & SPI_SR_BSY) == 1){}
}

void key_led_Init(void){
  // Clock BUS Initial
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PE|SYSCFG_EXTICR3_EXTI11_PE;
  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PE;
  //GPIO 
  GPIOE->MODER |= GPIO_MODER_MODE13_0|GPIO_MODER_MODE14_0|GPIO_MODER_MODE15_0;
  GPIOE->MODER &= ~(GPIO_MODER_MODE10|GPIO_MODER_MODE11|GPIO_MODER_MODE12);
  //Interrupts keys
  EXTI->PR |= EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12;
  EXTI->FTSR |= EXTI_FTSR_TR10|EXTI_FTSR_TR11|EXTI_FTSR_TR12;
  EXTI->IMR |= EXTI_IMR_IM10|EXTI_IMR_IM11|EXTI_IMR_IM12;
  //Interrupt NVIC Enable
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  //LED turn off
  GPIOE->BSRR |= GPIO_BSRR_BS13|GPIO_BSRR_BS14|GPIO_BSRR_BS15;
}

void SPI2_Init(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN;
  //MOSI Pin
  GPIOC->MODER |= GPIO_MODER_MODER3_1;
  GPIOC->AFR[0] |= (5<<GPIO_AFRL_AFSEL3_Pos);
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD3_1;
  //SCK Pin
  GPIOB->MODER |= GPIO_MODER_MODER10_1;
  GPIOB->AFR[1] |= (5<<GPIO_AFRH_AFSEL10_Pos);
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0;

  //SPI
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  SPI2->CR1 |= (0b101<<SPI_CR1_BR_Pos)|SPI_CR1_MSTR|SPI_CR1_DFF; //42MHz/64 = 656,25kHz 16bit Master
  SPI2->CR1 |= SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_LSBFIRST; //mode3 LSB first 1wire transmitt
  SPI2->CR1 |= SPI_CR1_SPE;
}

void SPI1_Init(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  //MISO (A6) SCK (A5) Pins
  GPIOA->MODER |= GPIO_MODER_MODE6_1|GPIO_MODER_MODE5_1;
  GPIOA->AFR[0] |= (5<<GPIO_AFRL_AFSEL6_Pos)|(5<<GPIO_AFRL_AFSEL5_Pos);
  //GPIOA->PUPDR |= GPIO_PUPDR_PUPD6|GPIO_PUPDR_PUPD6;
  //SPI
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  SPI1->CR1 |= (0b110<<SPI_CR1_BR_Pos)|SPI_CR1_DFF; //84MHz/128 = 656,25kHz 16bit
  SPI1->CR1 |= SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_LSBFIRST|SPI_CR1_SSM|SPI_CR1_RXONLY; //mode3 LSB first RXonly 
  SPI1->CR1 &= ~(SPI_CR1_BIDIMODE|SPI_CR1_SSI|SPI_CR1_MSTR); //slave
  SPI1->CR2 |= SPI_CR2_RXNEIE; //RX Interrupt
  SPI1->CR1 |= SPI_CR1_SPE;
  NVIC_EnableIRQ(SPI1_IRQn);
}
/*************************** End of file ****************************/
