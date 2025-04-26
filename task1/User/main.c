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
#include "delay.h"

//#define USE_ODR
#define USE_BSRR
int main()
{
	//ʹ��GPIOCʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//����PC13Ϊ�������
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//��������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;// �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// �ٶ� 50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	delay_init();
	#ifdef USE_ODR
	GPIOC->ODR |= 1 << 13;//PC13�ø�
	delay_ms(1000);
	GPIOC->ODR &= ~(1 << 13);//PC13�õ�
	delay_ms(1000);
	GPIOC->ODR |= 1 << 13;//PC13�ø�
	delay_ms(1000);
	GPIOC->ODR &= ~(1 << 13);//PC13�õ�
	delay_ms(1000);
	GPIOC->ODR |= 1 << 13;//PC13�ø�
	delay_ms(1000);
	GPIOC->ODR &= ~(1 << 13);//PC13�õ�
	delay_ms(1000);
	GPIOC->ODR |= 1 << 13;//PC13�ø�
	#endif
	/*
	ODR��д���Ӱ�������˿ڣ�ͬʱ���Ŷ�-λ����-д�Ĺ��̣�Ч�ʵ�
	*/
	#ifdef USE_BSRR
	GPIOC->BSRR = (1 << 13);//PC13�ø�
	delay_ms(1000);
	GPIOC->BSRR = (1 << (13 + 16));//PC13�õ�
	delay_ms(1000);
	GPIOC->BSRR = (1 << 13);//PC13�ø�
	delay_ms(1000);
	GPIOC->BSRR = (1 << (13 + 16));//PC13�õ�
	delay_ms(1000);
	GPIOC->BSRR = (1 << 13);//PC13�ø�
	delay_ms(1000);
	GPIOC->BSRR = (1 << (13 + 16));//PC13�õ�
	delay_ms(1000);
	GPIOC->BSRR = (1 << 13);//PC13�ø�
	
	#endif
	/*
	BSRR��16λд1��Ϊ�ߵ�ƽ����16λд1Ϊ�͵�ƽ��д0������
	*/
	while(1)
	{
		GPIOC->BSRR = (1 << 13);//PC13�ø�
		delay_ms(50);
		GPIOC->BSRR = (1 << (13 + 16));//PC13�õ�
		delay_ms(50);
	}
	return 0;
}

