#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void delay_init(void);       // ��ʼ�� SysTick
void delay_us(uint32_t us);  // ΢����ʱ
void delay_ms(uint32_t ms);  // ������ʱ

#endif
