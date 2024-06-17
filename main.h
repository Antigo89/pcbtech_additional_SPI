#ifndef H_MAIN
#define H_MAIN 1

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"

//Settings

//Description connect

//Constants
#define LED1 401
#define LED2 402
#define LED3 403
#define CLEAR 0xFF

#define SPI_F8B 0x00
#define SPI_F16B 0x01

//prototype function
void send_SPI2(uint16_t data);

void RCC_Init(void);
void key_led_Init(void);
void SPI2_Init(void);
void SPI1_Init(void);

#endif