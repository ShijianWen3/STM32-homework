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



volatile uint16_t pwm_duty = PWM_MIN_DUTY;  // ��ʼռ�ձ�
volatile int pwm_direction = 1;  // 1��ʾ�Ӱ�������-1��ʾ��������

int main(void)
{
    // ��ʼ����ʱ����GPIO
    TIM3_PWM_Init();
    GPIO_Init_Config();

    // ������ʱ��3
    TIM3->CR1 |= TIM_CR1_CEN;

    while (1)
    {
        // ���ϵ���ռ�ձ�
        TIM3->CCR2 = pwm_duty;  // ����ռ�ձ�
    }
}

void TIM3_PWM_Init(void)
{
    // ������ʱ��3��GPIOBʱ��
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // ����PB5Ϊ��ʱ��3��PWM�����ͨ��2��
    GPIOB->CRL &= ~GPIO_CRL_MODE5;  // ���PB5������
    GPIOB->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;  // PB5Ϊ�����������

    // ���ö�ʱ��3ΪPWMģʽ
    TIM3->PSC = 71;   // ����Ԥ��Ƶ����APB1 ʱ��Ƶ��Ϊ72MHz������Ԥ��Ƶ������Ϊ71��������ʱ��Ϊ1MHz
    TIM3->ARR = PWM_PERIOD - 1;  // �����Զ����ؼĴ���������PWM����
    TIM3->CCR2 = pwm_duty;  // ��ʼռ�ձ�Ϊ��Сֵ��LED�Ӱ���ʼ

    // ����PWMģʽ������Ƚ�ͨ��
    TIM3->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // ����ΪPWMģʽ
    TIM3->CCER |= TIM_CCER_CC2E;  // ʹ��PWM���

    // ���ö�ʱ��3�жϣ������Ҫ�Զ�����ռ�ձȣ�
    NVIC_EnableIRQ(TIM3_IRQn);
}

void GPIO_Init_Config(void)
{
    // ������Գ�ʼ������GPIO���ã���LED���ƣ�
    // Ŀǰֻ��ҪPB5�����ü���
}


