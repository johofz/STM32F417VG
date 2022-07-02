#include <Clock.h>

namespace STM32F417VG
{
    namespace CLOCK
    {
        /*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
        1. ENABLE HSE and wait for the HSE to become Ready
        2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
        3. Configure the FLASH PREFETCH and the LATENCY Related Settings
        4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
        5. Configure the MAIN PLL
        6. Enable the PLL and wait for it to become ready
        7. Select the Clock Source and wait for it to be set
        ********************************************************/
        int SetupHSE()  // Setup for the HSE-Clock with max speed (168MHz)
        {
            RCC->CR |= (RCC_CR_HSEON);  // Enable HSE Clock
            while (!(RCC->CR & (RCC_CR_HSERDY))); // Wait for HSE to be ready
            RCC->APB1ENR |= (RCC_APB1ENR_PWREN); // Enable Power
            FLASH->ACR |= ((FLASH_ACR_DCEN) | (FLASH_ACR_ICEN) | (FLASH_ACR_PRFTEN) | (FLASH_ACR_LATENCY_0WS)); // Internal Flash configuration

            RCC->CFGR |= ((RCC_CFGR_PPRE1_DIV4) | (RCC_CFGR_PPRE2_DIV2));
            RCC->PLLCFGR |= ((6 << RCC_PLLCFGR_PLLM_Pos) | (168 << RCC_PLLCFGR_PLLN_Pos) | (RCC_PLLCFGR_PLLSRC_HSE));
            RCC->CR |= (RCC_CR_PLLON);  // Enable PLL
            while (!(RCC->CR & (RCC_CR_PLLRDY))); // Wait for PLL to be ready

            RCC->CFGR |= (RCC_CFGR_SW_PLL);
            while (!(RCC->CFGR & (RCC_CFGR_SWS_PLL)));

            return 1;
        }
    }
}