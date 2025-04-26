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



volatile uint16_t pwm_duty = PWM_MIN_DUTY;  // 初始占空比
volatile int pwm_direction = 1;  // 1表示从暗到亮，-1表示从亮到暗

int main(void)
{
    // 初始化定时器和GPIO
    TIM3_PWM_Init();
    GPIO_Init_Config();

    // 启动定时器3
    TIM3->CR1 |= TIM_CR1_CEN;

    while (1)
    {
        // 不断调整占空比
        TIM3->CCR2 = pwm_duty;  // 更新占空比
    }
}

void TIM3_PWM_Init(void)
{
    // 开启定时器3和GPIOB时钟
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // 配置PB5为定时器3的PWM输出（通道2）
    GPIOB->CRL &= ~GPIO_CRL_MODE5;  // 清除PB5的配置
    GPIOB->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;  // PB5为复用推挽输出

    // 配置定时器3为PWM模式
    TIM3->PSC = 71;   // 设置预分频器，APB1 时钟频率为72MHz，所以预分频器设置为71，计数器时钟为1MHz
    TIM3->ARR = PWM_PERIOD - 1;  // 设置自动重载寄存器，定义PWM周期
    TIM3->CCR2 = pwm_duty;  // 初始占空比为最小值，LED从暗开始

    // 配置PWM模式和输出比较通道
    TIM3->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // 设置为PWM模式
    TIM3->CCER |= TIM_CCER_CC2E;  // 使能PWM输出

    // 配置定时器3中断（如果需要自动调整占空比）
    NVIC_EnableIRQ(TIM3_IRQn);
}

void GPIO_Init_Config(void)
{
    // 这里可以初始化其他GPIO配置（如LED控制）
    // 目前只需要PB5的配置即可
}


