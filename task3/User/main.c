/**
  ******************************************************************************
  * @file     
  * @author  wenshijian.site
  * @version v1.0
  * @date    24/3/2025
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 wenshijian.site.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#include <stdio.h>
#include <string.h>
#include "main.h"

char command[10];
int command_index = 0;

int main(void)
{
    // ����USART1��GPIO
    USART_Init_Config();
    GPIO_Init_Config();

    // Ĭ��LEDΪOFF
    LED_Control(1, 0);  // LED1 OFF
    LED_Control(2, 0);  // LED2 OFF
	USART_SendString(USART1, "start");
    while (1)
    {
        // ��ѭ�����ݿ�������������������Ҫ�����ж�����������
////		USART_SendString("Running");
//		USART_SendString("Running\r\n");
//		USART_SendString(USART1, "start\n");
//		for (volatile int i = 0; i < 800000; i++);  // ����ʱ
    }
}

void USART_Init_Config(void)
{
    // �������ʱ��
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;  // �ؼ�������AFIO

    // ����PA9Ϊ�������������PA10Ϊ��������
    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1);  // TX: AF�������

    GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0;  // RX: ��������

    // ���ò����ʣ�115200 @72MHz��
    USART1->BRR = 0x1D4C;

    // ʹ��USART1�����ͽ��չ���
    USART1->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

    // ���������ж�
    USART1->CR1 |= USART_CR1_RXNEIE;

    // �����ж����ȼ�������
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}


void GPIO_Init_Config(void)
{
    // ����PA1��PA2Ϊ���������LED���ƣ�
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1 | GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOA->CRL |= GPIO_CRL_MODE1_1 | GPIO_CRL_MODE2_1;  // �������
}

void LED_Control(uint8_t led, uint8_t state)
{
    if (led == 1) {
        if (state) {
            GPIOA->ODR |= GPIO_ODR_ODR1;  // LED1 ON
        } else {
            GPIOA->ODR &= ~GPIO_ODR_ODR1; // LED1 OFF
        }
    } else if (led == 2) {
        if (state) {
            GPIOA->ODR |= GPIO_ODR_ODR2;  // LED2 ON
        } else {
            GPIOA->ODR &= ~GPIO_ODR_ODR2; // LED2 OFF
        }
    }
}

void USART_SendString(USART_TypeDef* USARTx, const char* str)
{
    while (*str) // �����ַ���ֱ��ĩβ
    {
        USART_SendData(USARTx, (uint8_t)(*str)); // ���͵�ǰ�ַ�
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) != SET); // �ȴ��������
        str++; // ָ����һ���ַ�
    }
}

