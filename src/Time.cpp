#include <Time.h>


namespace STM32F417VG
{
    namespace TIME
    {
        void _GlobalTimerISR();

        TIM_TypeDef *_Timers[TIMER_COUNT] = {
            TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7,
            TIM8, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14
        };
        TimerISR _TimerISR[TIMER_COUNT];

        TIM_TypeDef *_GlobalTimer;
        volatile uint32_t _GlobalTimerOverflowConter = 0;

        int InitGlobalTimer(TIM_TypeDef *timer)
        {
            if (!IsTimer(timer)) return 0;

            uint32_t timerBaseFreq = GetTimerBaseFreq(timer);
            EnableTimer(timer);
            EnableTimerInterrupt(timer);
            RegisterISR(timer, _GlobalTimerISR);

            timer->PSC = timerBaseFreq / 1e6;   // µ-sec resolution
            timer->ARR = 0xFFFF;                
            timer->CR1 |= (TIM_CR1_URS);        // Only overflow & underflow trigger update event
            timer->DIER |= (TIM_DIER_UIE);      // Enable update event interrupt
            StartTimer(timer);

            return 1;
        }

        inline int StartTimer(TIM_TypeDef *timer)
        {
            if (!IsTimer(timer)) return 0;

            timer->CR1 |= (TIM_CR1_CEN);

            return 1;
        }

        inline int StopTimer(TIM_TypeDef *timer)
        {
            if (!IsTimer(timer)) return 0;

            timer->CR1 &= ~(TIM_CR1_CEN);

            return 1;
        }

        int EnableTimer(TIM_TypeDef *timer)
        {
            if (timer == TIM1) RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN); return 1;
            if (timer == TIM2) RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN); return 1;
            if (timer == TIM3) RCC->APB1ENR |= (RCC_APB1ENR_TIM3EN); return 1;
            if (timer == TIM4) RCC->APB1ENR |= (RCC_APB1ENR_TIM4EN); return 1;
            if (timer == TIM5) RCC->APB1ENR |= (RCC_APB1ENR_TIM5EN); return 1;
            if (timer == TIM6) RCC->APB1ENR |= (RCC_APB1ENR_TIM6EN); return 1;
            if (timer == TIM7) RCC->APB1ENR |= (RCC_APB1ENR_TIM7EN); return 1;
            if (timer == TIM8) RCC->APB2ENR |= (RCC_APB2ENR_TIM8EN); return 1;
            if (timer == TIM9) RCC->APB2ENR |= (RCC_APB2ENR_TIM9EN); return 1;
            if (timer == TIM10) RCC->APB2ENR |= (RCC_APB2ENR_TIM10EN); return 1;
            if (timer == TIM11) RCC->APB2ENR |= (RCC_APB2ENR_TIM11EN); return 1;
            if (timer == TIM12) RCC->APB1ENR |= (RCC_APB1ENR_TIM12EN); return 1;
            if (timer == TIM13) RCC->APB1ENR |= (RCC_APB1ENR_TIM13EN); return 1;
            if (timer == TIM14) RCC->APB1ENR |= (RCC_APB1ENR_TIM14EN); return 1;

            return 0;
        }

        int EnableTimerInterrupt(TIM_TypeDef * timer)
        {
            if (!IsTimer(timer)) return 0;

            IRQn_Type irq;
            if (timer == TIM1) irq = TIM1_UP_TIM10_IRQn;
            else if (timer == TIM2) irq = TIM2_IRQn;
            else if (timer == TIM3) irq = TIM3_IRQn;
            else if (timer == TIM4) irq = TIM4_IRQn;
            else if (timer == TIM5) irq = TIM5_IRQn;
            else if (timer == TIM6) irq = TIM6_DAC_IRQn;
            else if (timer == TIM7) irq = TIM7_IRQn;
            else if (timer == TIM8) irq = TIM8_UP_TIM13_IRQn;
            else if (timer == TIM9) irq = TIM1_BRK_TIM9_IRQn;
            else if (timer == TIM10) irq = TIM1_UP_TIM10_IRQn;
            else if (timer == TIM11) irq = TIM1_TRG_COM_TIM11_IRQn;
            else if (timer == TIM12) irq = TIM8_BRK_TIM12_IRQn;
            else if (timer == TIM13) irq = TIM8_UP_TIM13_IRQn;
            else irq = TIM8_TRG_COM_TIM14_IRQn;

            __NVIC_SetPriority(irq, 0);
            __NVIC_EnableIRQ(irq);

            return 1;
        }
        
        int RegisterISR(TIM_TypeDef *timer, TimerISR isr)
        {
            if (!IsTimer(timer)) return 0;

            for (int i = 0; i < TIMER_COUNT; i++)
                if (timer == _Timers[i])
                    _TimerISR[i] = isr;

            return 1;
        }

        uint64_t micros() { return _GlobalTimer->CNT * _GlobalTimerOverflowConter; }
        uint64_t millis() { return micros() * 1000; }
        uint32_t seconds() { return micros() * 1000000; }

        inline int IsTimer(TIM_TypeDef *timer)
        {
            for (int i = 0; i < TIMER_COUNT; i++)
                if (timer == _Timers[i])
                    return 1;
            
            return 0;
        }

        uint32_t GetTimerBaseFreq(TIM_TypeDef *timer)
        {
            if (timer == TIM1 || timer == TIM8 || timer == TIM9 ||
                timer == TIM10 || timer == TIM11)
                return SystemCoreClock;
            
            return SystemCoreClock / 2;
        }

        void _GlobalTimerISR()
        {
            STM32F417VG::TIME::_GlobalTimerOverflowConter++;
        }
    } // namespace TIME
    
} // namespace STM32F417VG
