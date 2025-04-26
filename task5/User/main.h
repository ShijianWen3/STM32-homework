#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"

#include <stdio.h>
#include <string.h>

// PWM���ںͳ�ʼռ�ձ�
#define PWM_PERIOD 1000  // PWM���� (1000Ϊ��)
#define PWM_MAX_DUTY 1000  // ���ռ�ձȣ�������
#define PWM_MIN_DUTY 0     // ��Сռ�ձȣ���




void ADC1_Init(void);
void USART1_Init(void);
void USART_SendString(USART_TypeDef* USARTx, char* str);



#endif