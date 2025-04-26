#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"

#include <stdio.h>

extern volatile uint32_t button_press_time;  // ��¼�������µ�ʱ��
extern volatile uint8_t button_state;         // 0��ʾδ���£�1��ʾ���£�2��ʾ����

void ShortPressAction(void);
void LongPressAction(void);



#endif