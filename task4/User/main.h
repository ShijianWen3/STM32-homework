#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"

#include <stdio.h>
#include <string.h>

// PWM���ںͳ�ʼռ�ձ�
#define PWM_PERIOD 1000  // PWM���� (1000Ϊ��)
#define PWM_MAX_DUTY 1000  // ���ռ�ձȣ�������
#define PWM_MIN_DUTY 0     // ��Сռ�ձȣ���


extern volatile uint16_t pwm_duty;  // ��ʼռ�ձ�
extern volatile int pwm_direction;

void TIM3_PWM_Init(void);
void GPIO_Init_Config(void);



#endif