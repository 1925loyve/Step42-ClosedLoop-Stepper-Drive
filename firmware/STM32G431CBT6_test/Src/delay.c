#include "delay.h"
#include "stm32G4xx_hal.h"

void DWT_Init(void)
{
    // 使能 DWT 外设
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  
    
    // 复位计数器
    DWT->CYCCNT = 0;                                
    
    // 使能 cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            
}

void delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000L) * us; // 换算成 CPU 周期数
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);              // 等待
}

void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}


