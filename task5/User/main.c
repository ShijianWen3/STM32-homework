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
#include "delay.h"
// 定义采集值
uint16_t ADC_Buffer[2];



int main(void) {
    SystemInit();  // 初始化系统
    ADC1_Init();   // 初始化 ADC
    USART1_Init(); // 初始化串口
	delay_init();
    // 启动 ADC 转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (1) {
        // 等待转换完成
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

        // 读取转换结果
        ADC_Buffer[0] = ADC_GetConversionValue(ADC1); // 获取 PA0 电压值
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);  // 清除转换完成标志

        // 继续进行下一次转换
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        
        // 等待转换完成
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        
        // 获取 PA1 电压值
        ADC_Buffer[1] = ADC_GetConversionValue(ADC1);
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

        // 格式化并输出结果
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "PA0: %d, PA1: %d\r\n", ADC_Buffer[0], ADC_Buffer[1]);
        USART_SendString(USART1, buffer);

        // 延时 1 秒
        delay_ms(1000);
    }
}


void ADC1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // 配置 PA0 和 PA1 为模拟输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ADC 配置
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;  // 扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // 连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 不使用外部触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 2;  // 2 个通道（PA0 和 PA1）
    ADC_Init(ADC1, &ADC_InitStructure);

    // 配置通道 0 和 1 的采集顺序
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_1Cycles5); // PA0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_1Cycles5); // PA1

    // 启动 ADC
    ADC_Cmd(ADC1, ENABLE);

    // 校准 ADC
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

void USART1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    // 开启 GPIOA 和 USART1 的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    // 配置 PA9 为 USART1 TX，PA10 为 USART1 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置 USART1
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    
    // 启用 USART1
    USART_Cmd(USART1, ENABLE);
}


void USART_SendString(USART_TypeDef* USARTx, char* str) {
    while (*str) {
        USART_SendData(USARTx, *str++);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    }
}

