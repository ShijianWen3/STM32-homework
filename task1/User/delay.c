#include "delay.h"



#include "delay.h"

static uint8_t  fac_us = 0;  // us延时倍乘数
static uint16_t fac_ms = 0;  // ms延时倍乘数

// 初始化 SysTick
void delay_init(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // 关闭 SysTick
    fac_us = SystemCoreClock / 1000000;        // 72MHz -> 72
    fac_ms = fac_us * 1000;                    // 72 * 1000 = 72000
}

// 微秒延时函数
void delay_us(uint32_t us)
{
    uint32_t temp;
    SysTick->LOAD = us * fac_us;              // 设置重装载值
    SysTick->VAL = 0x00;                      // 清空当前计数器
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;  // 启动计数，使用内核时钟

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & SysTick_CTRL_COUNTFLAG_Msk) == 0); // 等待到达0

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // 关闭SysTick
}

// 毫秒延时函数
void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);
    }
}
