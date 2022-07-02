#ifndef __RCC_H__
#define __RCC_H__

#include <stm32f417xx.h>

namespace STM32F417VG
{
    namespace CLOCK
    {
        int SetupHSE(uint32_t clockspeed);
        int SetupHSE();
    }
}

#endif // __RCC_H__