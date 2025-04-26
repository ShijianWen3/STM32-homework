#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"

#include <stdio.h>
#include <string.h>

void USART_Init_Config(void);
void GPIO_Init_Config(void);
void LED_Control(uint8_t led, uint8_t state);
void USART_SendString(USART_TypeDef* USARTx, const char* str);
void USART_ReceiveCommand(void);


extern char command[10];
extern int command_index;


#endif