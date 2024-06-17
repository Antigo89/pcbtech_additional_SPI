#ifndef H_MAIN
#define H_MAIN 1

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"

//Constants
#define LED1 401
#define LED2 402
#define LED3 403
#define CLEAR 0xFF

//prototype function
void send_SPI2(uint16_t data);

void RCC_Init(void);
void key_led_Init(void);
void SPI2_Init(void);
void SPI1_Init(void);

#endif