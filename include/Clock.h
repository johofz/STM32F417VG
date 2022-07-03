#ifndef __RCC_H__
#define __RCC_H__

#include <stm32f417xx.h>

namespace STM32F417VG
{
    namespace CLOCK
    {
        int SetupHSE(uint32_t clockspeed);
        int SetupHSE();

        int SetupPLL(uint8_t m, uint8_t n, uint8_t p);
        int SetExternalCrystal(uint32_t crystalSpeed);
        uint32_t GetExternalCrystalSpeed();
    }
}

#endif // __RCC_H__