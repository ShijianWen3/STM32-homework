#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"

#include <stdio.h>
#include <string.h>

// PWM周期和初始占空比
#define PWM_PERIOD 1000  // PWM周期 (1000为例)
#define PWM_MAX_DUTY 1000  // 最大占空比（满亮）
#define PWM_MIN_DUTY 0     // 最小占空比（灭）


extern volatile uint16_t pwm_duty;  // 初始占空比
extern volatile int pwm_direction;

void TIM3_PWM_Init(void);
void GPIO_Init_Config(void);



#endif