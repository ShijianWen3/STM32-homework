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


// �ⲿ�жϴ�����
void EXTI0_IRQHandler(void) 
{
    // ����жϱ�־
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
	{
        // �������£��͵�ƽ����ִ����ز���
       

        // ����жϱ�־
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

// USART1�жϴ������
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)  // �ж��Ƿ��ǽ����ж�
    {
        char received_char = USART1->DR;  // ��ȡ���յ�������
		LED_Control(2, 1);
        // ������յ��س����У���Ϊ�������
        if (received_char == '\r' || received_char == '\n') {
            command[command_index] = '\0';  // ����ַ���������
            command_index = 0;  // ����������׼��������һ������
			USART_SendString(USART1, "hello");
//            // ��������
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
            // ������ǻس����У����������ַ�
            if (command_index < sizeof(command) - 1) {
                command[command_index++] = received_char;  // �洢���յ����ַ�
            }
        }
    }
}