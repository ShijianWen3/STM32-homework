#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"

#include <stdio.h>

extern volatile uint32_t button_press_time;  // 记录按键按下的时刻
extern volatile uint8_t button_state;         // 0表示未按下，1表示按下，2表示长按

void ShortPressAction(void);
void LongPressAction(void);



#endif