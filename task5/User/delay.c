#include "delay.h"



#include "delay.h"

static uint8_t  fac_us = 0;  // us��ʱ������
static uint16_t fac_ms = 0;  // ms��ʱ������

// ��ʼ�� SysTick
void delay_init(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // �ر� SysTick
    fac_us = SystemCoreClock / 1000000;        // 72MHz -> 72
    fac_ms = fac_us * 1000;                    // 72 * 1000 = 72000
}

// ΢����ʱ����
void delay_us(uint32_t us)
{
    uint32_t temp;
    SysTick->LOAD = us * fac_us;              // ������װ��ֵ
    SysTick->VAL = 0x00;                      // ��յ�ǰ������
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;  // ����������ʹ���ں�ʱ��

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & SysTick_CTRL_COUNTFLAG_Msk) == 0); // �ȴ�����0

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // �ر�SysTick
}

// ������ʱ����
void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);
    }
}
