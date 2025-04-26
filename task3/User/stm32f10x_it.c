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
    // 检查中断标志
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
	{
        // 按键按下（低电平），执行相关操作
       

        // 清除中断标志
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

// USART1中断处理程序
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)  // 判断是否是接收中断
    {
        char received_char = USART1->DR;  // 获取接收到的数据
		LED_Control(2, 1);
        // 如果接收到回车或换行，认为命令结束
        if (received_char == '\r' || received_char == '\n') {
            command[command_index] = '\0';  // 添加字符串结束符
            command_index = 0;  // 重置索引，准备接收下一个命令
			USART_SendString(USART1, "hello");
//            // 处理命令
			LED_Control(2, 1);
            if (strcmp(command, "ON1") == 0) {
                LED_Control(1, 1);
                USART_SendString(USART1,"LED1 ON\r\n");
            } else if (strcmp(command, "OFF1") == 0) {
                LED_Control(1, 0);
                USART_SendString(USART1,"LED1 OFF\r\n");
            } else if (strcmp(command, "ON2") == 0) {
                LED_Control(2, 1);
                USART_SendString(USART1,"LED2 ON\r\n");
            } else if (strcmp(command, "OFF2") == 0) {
                LED_Control(2, 0);
                USART_SendString(USART1,"LED2 OFF\r\n");
            } else if (strcmp(command, "STATUS") == 0) {
                if (GPIOA->ODR & GPIO_ODR_ODR1) {
                    USART_SendString(USART1,"LED1 ON\r\n");
                } else {
                    USART_SendString(USART1,"LED1 OFF\r\n");
                }

                if (GPIOA->ODR & GPIO_ODR_ODR2) {
                    USART_SendString(USART1,"LED2 ON\r\n");
                } else {
                    USART_SendString(USART1,"LED2 OFF\r\n");
                }
            } else {
                USART_SendString(USART1,"Unknown command\r\n");
            }
        } else {
            // 如果不是回车或换行，继续接收字符
            if (command_index < sizeof(command) - 1) {
                command[command_index++] = received_char;  // 存储接收到的字符
            }
        }
    }
}