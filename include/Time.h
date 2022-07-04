#pragma once
#include "stm32f417xx.h"
#include <Clock.h>

#define TIMER_COUNT 14

namespace STM32F417VG
{
    namespace TIME
    {
        typedef void (*TimerISR)(void);

        int InitGlobalTimer(TIM_TypeDef *timer);

        inline int StartTimer(TIM_TypeDef *timer);
        inline int StopTimer(TIM_TypeDef *timer);
        int EnableTimer(TIM_TypeDef *timer);
        int EnableTimerInterrupt(TIM_TypeDef * timer);
        int RegisterISR(TIM_TypeDef *timer, TimerISR isr);

        uint64_t micros();
        uint64_t millis();
        uint32_t seconds();

        inline int IsTimer(TIM_TypeDef *timer);
        uint32_t GetTimerBaseFreq(TIM_TypeDef *timer);
    } // namespace TIME
    
} // namespace STM32F417VG
