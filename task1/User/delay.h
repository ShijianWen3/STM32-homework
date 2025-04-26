#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void delay_init(void);       // ≥ı ºªØ SysTick
void delay_us(uint32_t us);  // Œ¢√Î—” ±
void delay_ms(uint32_t ms);  // ∫¡√Î—” ±

#endif
