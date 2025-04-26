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
    // 配置USART1和GPIO
    USART_Init_Config();
    GPIO_Init_Config();

    // 默认LED为OFF
    LED_Control(1, 0);  // LED1 OFF
    LED_Control(2, 0);  // LED2 OFF
	USART_SendString(USART1, "start");
    while (1)
    {
        // 主循环内容可以做其他任务，这里主要依赖中断来接收命令
////		USART_SendString("Running");
//		USART_SendString("Running\r\n");
//		USART_SendString(USART1, "start\n");
//		for (volatile int i = 0; i < 800000; i++);  // 简单延时
    }
}

void USART_Init_Config(void)
{
    // 开启相关时钟
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;  // 关键：开启AFIO

    // 配置PA9为复用推挽输出，PA10为浮空输入
    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1);  // TX: AF推挽输出

    GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0;  // RX: 浮空输入

    // 设置波特率（115200 @72MHz）
    USART1->BRR = 0x1D4C;

    // 使能USART1，发送接收功能
    USART1->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

    // 开启接收中断
    USART1->CR1 |= USART_CR1_RXNEIE;

    // 设置中断优先级并开启
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}


void GPIO_Init_Config(void)
{
    // 配置PA1、PA2为推挽输出（LED控制）
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1 | GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOA->CRL |= GPIO_CRL_MODE1_1 | GPIO_CRL_MODE2_1;  // 推挽输出
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
    while (*str) // 遍历字符串直到末尾
    {
        USART_SendData(USARTx, (uint8_t)(*str)); // 发送当前字符
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) != SET); // 等待发送完成
        str++; // 指向下一个字符
    }
}

