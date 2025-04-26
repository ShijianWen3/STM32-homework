#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>




extern volatile float temperature_threshold;
extern float current_temperature;

void ADC_Config(void);
void USART_Config(void);
void EXTI_Config(void);
void LED_Init(void);
void PWM_Init(void);
void USART_SendString(USART_TypeDef* USARTx, uint8_t *str);
float Temperature_Sensor(void);
void send_temperature(float temperature);
void Update_PWM_Duty(void);
void TIM3_Init(void);
#endif