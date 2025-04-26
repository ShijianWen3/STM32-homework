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
#include "main.h"

int main()
{
	gpio_init();
	exit_init();
	
	
	
	
	
	while(1)
	{
		
	}
	return 0;
}


void gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure_PA0;
    // 1. 启用 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 2. 配置 PA0 为输入模式
    GPIO_InitStructure_PA0.GPIO_Pin = GPIO_Pin_0; // PA0 引脚
    GPIO_InitStructure_PA0.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入模式
    GPIO_Init(GPIOA, &GPIO_InitStructure_PA0); // 初始化 PA0
	
	// 假设 LED 接在 PC13 上，初始化 GPIOC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure_PC13;
    GPIO_InitStructure_PC13.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure_PC13.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    GPIO_InitStructure_PC13.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure_PC13);
	

	
}

void exit_init()
{
	// 1. 启用外部中断控制器时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // 2. 配置外部中断线路 0 (PA0)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    // 3. 配置 EXTI0 中断线
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;               // 使用 EXTI0（PA0）
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;       // 设置为中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;   // 触发条件为下降沿（按键按下时）
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                 // 使能中断线
    EXTI_Init(&EXTI_InitStructure);                           // 初始化 EXTI0

    // 4. 配置 NVIC 中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;         // 外部中断 0 通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 最高优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // 子优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          // 使能中断
    NVIC_Init(&NVIC_InitStructure);                           // 初始化 NVIC
}
