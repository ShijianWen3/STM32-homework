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
		
		if (current_temperature > temperature_threshold)  // ����
		{
			Update_PWM_Duty();  // ÿ�ε��ø���ռ�ձ�
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);
		}
		else  // �¶�����
		{
			TIM_SetCompare2(TIM2, 0);  // ֹͣ PWM���ر� LED
		}
		delay_ms(100);
	}
    
}


void ADC_Config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE); // ���� GPIOA �� ADC1 ʱ��

    // ���� PA7 Ϊģ������
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // ģ������ģʽ
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ADC ��ʼ������
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;              // ����ģʽ
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                   // ����ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;             // ����ת��
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // �������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;          // �����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 1;                          // ת��ͨ����1
    ADC_Init(ADC1, &ADC_InitStructure);

    // ���� ADC ͨ����PA7 -> ADC1_IN7��
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_55Cycles5);

    // ���� ADC
    ADC_Cmd(ADC1, ENABLE);

    // У׼ ADC���ϵ��ֻ��Ҫһ�Σ�
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

void USART_Config(void)
{
    // 1. ���� USART1 ����� GPIO ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // PA9 Ϊ USART1 TX��PA10 Ϊ USART1 RX

    // 2. ���� GPIO ���ţ�PA9 Ϊ TX, PA10 Ϊ RX��
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // ���� PA9��USART1 TX��
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // �������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ���� PA10��USART1 RX��
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // ��������
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. ���� USART1 ����
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;  // ������ 115200
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;  // 8 λ����λ
    USART_InitStruct.USART_StopBits = USART_StopBits_1;  // 1 ��ֹͣλ
    USART_InitStruct.USART_Parity = USART_Parity_No;  // ����żУ��
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ʹ��Ӳ��������
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  // ʹ�ܷ��ͺͽ��չ���

    // 4. ��ʼ�� USART1
    USART_Init(USART1, &USART_InitStruct);

    // 5. ʹ�� USART1
    USART_Cmd(USART1, ENABLE);
    
    // 6. ���� USART1 �����жϣ������Ҫʹ���жϽ��գ�
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // ʹ�ܽ����ж�
    NVIC_EnableIRQ(USART1_IRQn);  // ʹ�� USART1 �ж�
}


void EXTI_Config(void)
{
    // 1. ���� GPIO ���� PA0 Ϊ�ⲿ�ж��ߣ�EXTI Line0��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // ʹ�� GPIOA ʱ��

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;  // PA0
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;  // ��������ģʽ
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. �����ⲿ�жϿ�������EXTI����
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // ʹ�� AFIO ʱ��

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);  // ���� PA0 Ϊ EXTI0 ����

    EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;  // ʹ�� EXTI Line0
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  // �ж�ģʽ
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;  // �����ش����ж�
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;  // ʹ���ж���
    EXTI_Init(&EXTI_InitStruct);

    // 3. �����ж����ȼ���ʹ���ж�
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;  // ʹ�� EXTI0 �ж�
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // �ж���ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;  // �ж���Ӧ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;  // ʹ���ж�
    NVIC_Init(&NVIC_InitStruct);
}

void LED_Init(void)
{
    // ��ʼ�� LED ���� PA2
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
}


void PWM_Init(void)
{
    // 1. ���� GPIO ���� PA1 Ϊ���ģʽ��PWM �����
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // ʹ�� GPIOA ʱ��

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;  // PA1 Ϊ PWM ���
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // �����������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. ���ö�ʱ�� TIM2 ���� PWM �ź�
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // ʹ�� TIM2 ʱ��

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;  // ����Ƶ�ʣ�1000 Ϊ����
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;  // 1ms ����
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    TIM_Cmd(TIM2, ENABLE);  // ������ʱ��

    // 3. ���� PWM ���ͨ��
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);  // ʹ�� TIM2 ͨ�� 2
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 4. ���� PWM ���
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
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); // ��ʼת��

    // �ȴ�ת�����
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    uint16_t ADC_Value = ADC_GetConversionValue(ADC1); // ���� 12 λ ADC ֵ��0~4095��
   
    float voltage = (ADC_Value / 4096.0) * 3.3;
    float temperature = voltage * 100;  // LM35 ���Ϊ 10mV/��C
	
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
	static uint16_t pwm_duty = 0;  // ��ʼռ�ձ�	
	static int pwm_direction = 1;  // 1: ����, -1: ����
    // ���ӻ����ռ�ձ�
    if (pwm_direction == 1) 
    {
        pwm_duty += pwm_direction * 100;
        if (pwm_duty >= 1000)
		{
			pwm_direction = -1;// �ﵽ���ֵ����ʼ��С 
			
		}			
		
    } 
    else 
    {
        pwm_duty += pwm_direction * 100;
        if (pwm_duty == 0)  
            pwm_direction = 1;// �ﵽ��Сֵ����ʼ����
    }
    // ���� PWM ռ�ձ�
    TIM_SetCompare2(TIM2, pwm_duty);  // �޸� PWM ռ�ձ�
}



void TIM3_Init(void)
{
    // ���� TIM3 ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // ���� TIM3 ��������
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = 35999;          // ARR = 35999 (1ms)
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;           // PSC = 0, ʱ��Ƶ�� 36MHz
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // ���������ϼ���
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    
    // ���ö�ʱ���ж�
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  // ���������ж�
    
    // ������ʱ��
    TIM_Cmd(TIM3, ENABLE);
    
    // ���� NVIC �ж�
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn; // �ж�Դ�� TIM3
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}




