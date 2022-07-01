#ifndef __RCC_H__
#define __RCC_H__

#include "stm32f417xx.h"

namespace STM32F417VG
{
    namespace RCC
    {
        int SetupHSE(uint32_t clockspeed);
    }
}

#endif // __RCC_H__