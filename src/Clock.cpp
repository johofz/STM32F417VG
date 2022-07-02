#include <Clock.h>

namespace STM32F417VG
{
    namespace CLOCK
    {
        uint32_t _maxClockSpeed = 168000000;
        uint32_t _externalCrystalSpeed = 16000000;

        int SetupHSE(uint32_t clockspeed)
        {
            if (clockspeed > _maxClockSpeed)
                return 0;

            return 1;
        }

        int SetupHSE()  // Setup for the HSE-Clock with max speed (168MHz)
        {
            RCC->CR |= (RCC_CR_HSEON);  // Enable HSE Clock
            while (!(RCC->CR & (RCC_CR_HSERDY))); // Wait for HSE to be ready
            RCC->APB1ENR |= (RCC_APB1ENR_PWREN); // Enable Power
            FLASH->ACR |= ((FLASH_ACR_DCEN) | (FLASH_ACR_ICEN) | (FLASH_ACR_PRFTEN) | (FLASH_ACR_LATENCY_0WS)); // Internal Flash configuration

            RCC->CFGR |= ((RCC_CFGR_PPRE1_DIV4) | (RCC_CFGR_PPRE2_DIV2));
            SetupPLL(8, 168, 2);

            RCC->CFGR |= (RCC_CFGR_SW_PLL);
            while (!(RCC->CFGR & (RCC_CFGR_SWS_PLL)));

            return 1;
        }

        int SetupPLL(uint8_t m, uint8_t n, uint8_t p)
        {
            uint8_t p_val;
            switch (p)
            {
                case 2: p_val = 0; break;
                case 4: p_val = 1; break;
                case 6: p_val = 2; break;
                case 8: p_val = 3; break;

                default: SetupPLL(8, 168, 2); return 0;
            }

            RCC->PLLCFGR |= ((m << RCC_PLLCFGR_PLLM_Pos) |
                             (n << RCC_PLLCFGR_PLLN_Pos) |
                             (p_val << RCC_PLLCFGR_PLLN_Pos) |
                             (RCC_PLLCFGR_PLLSRC_HSE));
                             
            RCC->CR |= (RCC_CR_PLLON);  // Enable PLL
            while (!(RCC->CR & (RCC_CR_PLLRDY))); // Wait for PLL to be ready

            return 1;
        }
    }
}