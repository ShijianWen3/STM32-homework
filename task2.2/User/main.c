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


volatile uint32_t button_press_time = 0;  // ��¼�������µ�ʱ��
volatile uint8_t button_state = 0;         // 0��ʾδ���£�1��ʾ���£�2��ʾ����


void ShortPressAction(void)
{
    // �̰�ʱִ�еĲ���
    // ���磺����LED
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void LongPressAction(void)
{
    // ����ʱִ�еĲ���
    // ���磺Ϩ��LED
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void exit_init()
{
	GPIO_InitTypeDef GPIO_InitStructure_G;
    
    // ʹ��PA0�˿�ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // ����PA0Ϊ���븡��
    GPIO_InitStructure_G.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure_G.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure_G);
	
	EXTI_InitTypeDef EXTI_InitStructure;
    // ʹ���ⲿ�ж���·0��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // ��PA0����ӳ�䵽�ⲿ�ж�0
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    // �����ⲿ�жϴ�����ʽ�����º��ɿ�������
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  // �����غ��½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    // ʹ���ⲿ�ж�0���ж�����
    NVIC_EnableIRQ(EXTI0_IRQn);
	
	// ����SysTick��ʱ������ʱ1ms�ж�
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

