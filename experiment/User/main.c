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
#include "main.h"

volatile float temperature_threshold;

float current_temperature;


int main(void) 
{
	USART_Config();
	
	ADC_Config();
	
	EXTI_Config();
	
	LED_Init();
	
	delay_init();
	
	PWM_Init();

	TIM3_Init();//1ms
	temperature_threshold = 25.00;
	
	
	
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	delay_ms(500);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	delay_ms(500);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	delay_ms(500);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	delay_ms(500);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	

	
	
	while(1)
	{
		
		if (current_temperature > temperature_threshold)  // 超温
		{
			Update_PWM_Duty();  // 每次调用更新占空比
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);
		}
		else  // 温度正常
		{
			TIM_SetCompare2(TIM2, 0);  // 停止 PWM，关闭 LED
		}
		delay_ms(100);
	}
    
}


void ADC_Config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE); // 开启 GPIOA 和 ADC1 时钟

    // 配置 PA7 为模拟输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 模拟输入模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ADC 初始化配置
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;              // 独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                   // 禁用扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;             // 单次转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;          // 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;                          // 转换通道数1
    ADC_Init(ADC1, &ADC_InitStructure);

    // 配置 ADC 通道（PA7 -> ADC1_IN7）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_55Cycles5);

    // 启用 ADC
    ADC_Cmd(ADC1, ENABLE);

    // 校准 ADC（上电后只需要一次）
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

void USART_Config(void)
{
    // 1. 启用 USART1 和相关 GPIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // PA9 为 USART1 TX，PA10 为 USART1 RX

    // 2. 配置 GPIO 引脚（PA9 为 TX, PA10 为 RX）
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 配置 PA9（USART1 TX）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置 PA10（USART1 RX）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. 配置 USART1 参数
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;  // 波特率 115200
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;  // 8 位数据位
    USART_InitStruct.USART_StopBits = USART_StopBits_1;  // 1 个停止位
    USART_InitStruct.USART_Parity = USART_Parity_No;  // 无奇偶校验
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 使用硬件流控制
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  // 使能发送和接收功能

    // 4. 初始化 USART1
    USART_Init(USART1, &USART_InitStruct);

    // 5. 使能 USART1
    USART_Cmd(USART1, ENABLE);
    
    // 6. 配置 USART1 接收中断（如果需要使用中断接收）
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // 使能接收中断
    NVIC_EnableIRQ(USART1_IRQn);  // 使能 USART1 中断
}


void EXTI_Config(void)
{
    // 1. 配置 GPIO 引脚 PA0 为外部中断线（EXTI Line0）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // 使能 GPIOA 时钟

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;  // PA0
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入模式
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. 配置外部中断控制器（EXTI）线
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // 使能 AFIO 时钟

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);  // 配置 PA0 为 EXTI0 输入

    EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;  // 使用 EXTI Line0
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  // 中断模式
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;  // 上升沿触发中断
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;  // 使能中断线
    EXTI_Init(&EXTI_InitStruct);

    // 3. 配置中断优先级和使能中断
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;  // 使用 EXTI0 中断
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 中断抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;  // 中断响应优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;  // 使能中断
    NVIC_Init(&NVIC_InitStruct);
}

void LED_Init(void)
{
    // 初始化 LED 引脚 PA2
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
}


void PWM_Init(void)
{
    // 1. 配置 GPIO 引脚 PA1 为输出模式（PWM 输出）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // 使能 GPIOA 时钟

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;  // PA1 为 PWM 输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. 配置定时器 TIM2 产生 PWM 信号
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // 使能 TIM2 时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;  // 设置频率，1000 为周期
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;  // 1ms 周期
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    TIM_Cmd(TIM2, ENABLE);  // 启动定时器

    // 3. 配置 PWM 输出通道
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);  // 使用 TIM2 通道 2
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 4. 启用 PWM 输出
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}




void USART_SendString(USART_TypeDef* USARTx, uint8_t *str)
{
    while (*str)
    {
        USART_SendData(USARTx, *str);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);

		
        str++;
    }
}

float Temperature_Sensor(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); // 开始转换

    // 等待转换完成
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    uint16_t ADC_Value = ADC_GetConversionValue(ADC1); // 返回 12 位 ADC 值（0~4095）
   
    float voltage = (ADC_Value / 4096.0) * 3.3;
    float temperature = voltage * 100;  // LM35 输出为 10mV/°C
	
	temperature = voltage;
	
	return temperature;
}

void send_temperature(float temperature)
{
	char buffer[32];
    sprintf(buffer, "Temp: %.1f C\n", temperature);
    USART_SendString(USART1, (uint8_t*)buffer);
}


void Update_PWM_Duty(void)
{
	static uint16_t pwm_duty = 0;  // 初始占空比	
	static int pwm_direction = 1;  // 1: 增加, -1: 减少
    // 增加或减少占空比
    if (pwm_direction == 1) 
    {
        pwm_duty += pwm_direction * 100;
        if (pwm_duty >= 1000)
		{
			pwm_direction = -1;// 达到最大值，开始减小 
			
		}			
		
    } 
    else 
    {
        pwm_duty += pwm_direction * 100;
        if (pwm_duty == 0)  
            pwm_direction = 1;// 达到最小值，开始增加
    }
    // 更新 PWM 占空比
    TIM_SetCompare2(TIM2, pwm_duty);  // 修改 PWM 占空比
}



void TIM3_Init(void)
{
    // 开启 TIM3 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 配置 TIM3 基本参数
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = 35999;          // ARR = 35999 (1ms)
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;           // PSC = 0, 时钟频率 36MHz
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 计数器向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    
    // 配置定时器中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  // 开启更新中断
    
    // 启动定时器
    TIM_Cmd(TIM3, ENABLE);
    
    // 配置 NVIC 中断
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn; // 中断源是 TIM3
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}




