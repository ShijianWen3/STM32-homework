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
// ����ɼ�ֵ
uint16_t ADC_Buffer[2];



int main(void) {
    SystemInit();  // ��ʼ��ϵͳ
    ADC1_Init();   // ��ʼ�� ADC
    DMA_Initial();    // ��ʼ�� DMA
    USART1_Init(); // ��ʼ������
	delay_init();
    // ���� ADC ת��
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (1) {
        // �ȴ� DMA �������
        if (DMA_GetFlagStatus(DMA1_FLAG_TC1) == SET) {
            // ��� DMA ������ɱ�־
            DMA_ClearFlag(DMA1_FLAG_TC1);

            // ��ʽ����������
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "PA0: %d, PA1: %d\r\n", ADC_Buffer[0], ADC_Buffer[1]);
            USART_SendString(USART1, buffer);
        }

        // ��ʱ 1 ��
        delay_ms(1000);
    }
}



// DMA ����
void DMA_Initial() 
{
    DMA_InitTypeDef DMA_InitStructure;

    // ���� DMA �� ADC ��ʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // ���� DMA ͨ�� 1���� ADC1 �������
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;  // ADC ���ݼĴ�����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_Buffer;      // �ڴ��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  // �����赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = 2;                               // ����ͨ����PA0 �� PA1��
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // �����ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;            // �ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16-bit ���ݴ�С
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 16-bit ���ݴ�С
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     // ѭ��ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 // ���ȼ�����
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                        // ��ʹ���ڴ浽�ڴ洫��
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);                        // ���� DMA1 ͨ�� 1

    // ���� DMA
    DMA_Cmd(DMA1_Channel1, ENABLE);
}


void ADC1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // ���� GPIOA �� ADC1 ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // ���� PA0 �� PA1 Ϊģ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ADC ����
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;  // ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // ����ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // ��ʹ���ⲿ����
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // �Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 2;  // 2 ��ͨ����PA0 �� PA1��
    ADC_Init(ADC1, &ADC_InitStructure);

    // ����ͨ�� 0 �� 1 �Ĳɼ�˳��
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_1Cycles5); // PA0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_1Cycles5); // PA1

    // ���� ADC
    ADC_Cmd(ADC1, ENABLE);

    // У׼ ADC
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // ���� ADC ʹ�� DMA
    ADC_DMACmd(ADC1, ENABLE);
}

void USART1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    // ���� GPIOA �� USART1 ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    // ���� PA9 Ϊ USART1 TX��PA10 Ϊ USART1 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // ���� USART1
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    
    // ���� USART1
    USART_Cmd(USART1, ENABLE);
}

void USART_SendString(USART_TypeDef* USARTx, char* str) {
    while (*str) {
        USART_SendData(USARTx, *str++);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    }
}

