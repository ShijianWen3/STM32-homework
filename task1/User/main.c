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
	//使能GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//配置PC13为推挽输出
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//配置引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;// 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// 速度 50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	delay_init();
	#ifdef USE_ODR
	GPIOC->ODR |= 1 << 13;//PC13置高
	delay_ms(1000);
	GPIOC->ODR &= ~(1 << 13);//PC13置低
	delay_ms(1000);
	GPIOC->ODR |= 1 << 13;//PC13置高
	delay_ms(1000);
	GPIOC->ODR &= ~(1 << 13);//PC13置低
	delay_ms(1000);
	GPIOC->ODR |= 1 << 13;//PC13置高
	delay_ms(1000);
	GPIOC->ODR &= ~(1 << 13);//PC13置低
	delay_ms(1000);
	GPIOC->ODR |= 1 << 13;//PC13置高
	#endif
	/*
	ODR的写入会影响整个端口，同时有着读-位运算-写的过程，效率低
	*/
	#ifdef USE_BSRR
	GPIOC->BSRR = (1 << 13);//PC13置高
	delay_ms(1000);
	GPIOC->BSRR = (1 << (13 + 16));//PC13置低
	delay_ms(1000);
	GPIOC->BSRR = (1 << 13);//PC13置高
	delay_ms(1000);
	GPIOC->BSRR = (1 << (13 + 16));//PC13置低
	delay_ms(1000);
	GPIOC->BSRR = (1 << 13);//PC13置高
	delay_ms(1000);
	GPIOC->BSRR = (1 << (13 + 16));//PC13置低
	delay_ms(1000);
	GPIOC->BSRR = (1 << 13);//PC13置高
	
	#endif
	/*
	BSRR低16位写1则为高电平，高16位写1为低电平，写0无作用
	*/
	while(1)
	{
		GPIOC->BSRR = (1 << 13);//PC13置高
		delay_ms(50);
		GPIOC->BSRR = (1 << (13 + 16));//PC13置低
		delay_ms(50);
	}
	return 0;
}

