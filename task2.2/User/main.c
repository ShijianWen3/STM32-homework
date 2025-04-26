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


volatile uint32_t button_press_time = 0;  // 记录按键按下的时刻
volatile uint8_t button_state = 0;         // 0表示未按下，1表示按下，2表示长按


void ShortPressAction(void)
{
    // 短按时执行的操作
    // 例如：点亮LED
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void LongPressAction(void)
{
    // 长按时执行的操作
    // 例如：熄灭LED
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void exit_init()
{
	GPIO_InitTypeDef GPIO_InitStructure_G;
    
    // 使能PA0端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 配置PA0为输入浮空
    GPIO_InitStructure_G.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure_G.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure_G);
	
	EXTI_InitTypeDef EXTI_InitStructure;
    // 使能外部中断线路0的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // 将PA0引脚映射到外部中断0
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    // 配置外部中断触发方式：按下和松开都触发
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  // 上升沿和下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    // 使能外部中断0的中断请求
    NVIC_EnableIRQ(EXTI0_IRQn);
	
	// 配置SysTick定时器，定时1ms中断
    SysTick_Config(SystemCoreClock / 1000);
}

int main()
{
	exit_init();
	
	while(1)
	{
		
	}
	return 0;
}

