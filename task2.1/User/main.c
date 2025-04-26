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
    // 1. ���� GPIOA ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 2. ���� PA0 Ϊ����ģʽ
    GPIO_InitStructure_PA0.GPIO_Pin = GPIO_Pin_0; // PA0 ����
    GPIO_InitStructure_PA0.GPIO_Mode = GPIO_Mode_IN_FLOATING; // ��������ģʽ
    GPIO_Init(GPIOA, &GPIO_InitStructure_PA0); // ��ʼ�� PA0
	
	// ���� LED ���� PC13 �ϣ���ʼ�� GPIOC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure_PC13;
    GPIO_InitStructure_PC13.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure_PC13.GPIO_Mode = GPIO_Mode_Out_PP;  // �������
    GPIO_InitStructure_PC13.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure_PC13);
	

	
}

void exit_init()
{
	// 1. �����ⲿ�жϿ�����ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // 2. �����ⲿ�ж���· 0 (PA0)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    // 3. ���� EXTI0 �ж���
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;               // ʹ�� EXTI0��PA0��
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;       // ����Ϊ�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;   // ��������Ϊ�½��أ���������ʱ��
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                 // ʹ���ж���
    EXTI_Init(&EXTI_InitStructure);                           // ��ʼ�� EXTI0

    // 4. ���� NVIC �ж����ȼ�
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;         // �ⲿ�ж� 0 ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ������ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // �����ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          // ʹ���ж�
    NVIC_Init(&NVIC_InitStructure);                           // ��ʼ�� NVIC
}
