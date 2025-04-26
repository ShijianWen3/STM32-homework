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
    DMA_Initial();    // 初始化 DMA
    USART1_Init(); // 初始化串口
	delay_init();
    // 启动 ADC 转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (1) {
        // 等待 DMA 传输完成
        if (DMA_GetFlagStatus(DMA1_FLAG_TC1) == SET) {
            // 清除 DMA 传输完成标志
            DMA_ClearFlag(DMA1_FLAG_TC1);

            // 格式化并输出结果
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "PA0: %d, PA1: %d\r\n", ADC_Buffer[0], ADC_Buffer[1]);
            USART_SendString(USART1, buffer);
        }

        // 延时 1 秒
        delay_ms(1000);
    }
}



// DMA 配置
void DMA_Initial() 
{
    DMA_InitTypeDef DMA_InitStructure;

    // 开启 DMA 和 ADC 的时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 配置 DMA 通道 1（与 ADC1 相关联）
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;  // ADC 数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_Buffer;      // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // 从外设到内存
    DMA_InitStructure.DMA_BufferSize = 2;                               // 两个通道（PA0 和 PA1）
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;            // 内存地址增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16-bit 数据大小
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 16-bit 数据大小
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     // 循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 // 优先级设置
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // 不使用内存到内存传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);                        // 配置 DMA1 通道 1

    // 启动 DMA
    DMA_Cmd(DMA1_Channel1, ENABLE);
}


void ADC1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // 开启 GPIOA 和 ADC1 的时钟
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

    // 启动 ADC 使用 DMA
    ADC_DMACmd(ADC1, ENABLE);
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

