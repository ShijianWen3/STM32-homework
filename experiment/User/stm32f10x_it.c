/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.6.0
  * @date    20-September-2021
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2011 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "delay.h"
#include <stdio.h>



#define BUFFER_SIZE 32

char rx_buffer[BUFFER_SIZE];  // 用于存储接收到的数据
uint8_t rx_index = 0;         // 当前缓冲区索引

u8 counter_5ms = 0;
u8 counter_200ms = 0;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 





// 外部中断处理函数
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // 按键切换阈值
        if (temperature_threshold == 25.0)
		{
			 temperature_threshold = 30.0;
		}
           
        else if (temperature_threshold == 30.0)
		{
			temperature_threshold = 35.0;
		}
        else
		{
			temperature_threshold = 25.0;
		} 
        
        // 发送当前温度阈值
        char buffer[32];
        sprintf(buffer, "New Threshold: %.1f C\n", temperature_threshold);
        USART_SendString(USART1, (uint8_t*)buffer);

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}



void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = USART_ReceiveData(USART1);

        // 如果接收到换行符（\n），则表示一条消息的结束
        if (received_byte == '\n')
        {
            rx_buffer[rx_index] = '\0';  // 在缓冲区末尾添加字符串结束符
            rx_index = 0;  // 重置缓冲区索引

            // 判断接收到的命令是否是 "07"
            if (strcmp(rx_buffer, "07") == 0)
            {
                char buffer[32];
                sprintf(buffer, "Threshold: %.1f C\n", temperature_threshold);
                USART_SendString(USART1, (uint8_t*)buffer);
            }
            else
            {
                USART_SendString(USART1, (uint8_t*)"invalid instruction.\n");
            }
        }
        else
        {
            // 如果接收到的字符不是换行符，保存字符到缓冲区
            if (rx_index < BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = received_byte;
            }
            else
            {
                // 缓冲区已满，丢弃当前字符
                rx_index = 0;
            }
        }
    }
}


// 定时器 TIM3 中断处理函数
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        counter_5ms++;
		counter_200ms++;
		if(5 == counter_5ms)
		{
			current_temperature = Temperature_Sensor();
			counter_5ms = 0;
		}
		else if(200 == counter_200ms)
		{
			send_temperature(current_temperature);
			counter_200ms = 0;
		}
        
    }
}