#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"

#include <stdio.h>
#include <string.h>


void DMA_Initial(void);
void ADC1_Init(void);
void USART1_Init(void);
void USART_SendString(USART_TypeDef* USARTx, char* str);



#endif