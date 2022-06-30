#include "Usart.h"

namespace STM32F417VG
{
    namespace USART
    {
        int EnablePeripheral(USART_TypeDef *usart)
        {
            if (!IsUsart(usart)) return 0;

            if (usart == USART1) RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);
            else if (usart == USART2) RCC->APB1ENR |= (RCC_APB1ENR_USART2EN);
            else if (usart == USART2) RCC->APB1ENR |= (RCC_APB1ENR_USART3EN);
            else if (usart == USART2) RCC->APB2ENR |= (RCC_APB2ENR_USART6EN);
            else if (usart == USART2) RCC->APB1ENR |= (RCC_APB1ENR_UART4EN);
            else if (usart == USART2) RCC->APB1ENR |= (RCC_APB1ENR_UART5EN);

            return 1;
        }

        int ConfigureGpios(USART_TypeDef *usart, uint8_t alternatePins)
        {
            if (!IsUsart(usart)) return 0;

            if (usart == USART1)
            {
                if (!alternatePins)
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral
                    GPIOA->MODER |= ((2 << 18) | (2 << 20));  // PA9+10 (TX/RX) mode: alternate function
                    GPIOA->AFR[1] |= ((7 << 4) | (7 << 8));  // PA9+10 (TX/RX) alternate function: AF7 (USART1)
                }
                else
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
                    GPIOB->MODER |= ((2 << 12) | (2 << 14));  // PB6+7 (TX/RX) mode: alternate function                    
                    GPIOB->AFR[0] |= ((7 << 24) | (7 << 28));  // PB6+7 (TX/RX) alternate function: AF7 (USART1)                    
                }
            }
            else if (usart == USART2)
            {
                if (!alternatePins)
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral
                    GPIOA->MODER |= ((2 << 4) | (2 << 6));  // PA2+3 (TX/RX) mode: alternate function
                    GPIOA->AFR[0] |= ((7 << 8) | (7 << 12));  // PA2 (TX) alternate function: AF7 (USART2)                    
                }
                else
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
                    GPIOB->MODER |= ((2 << 10) | (2 << 12));  // PB5+6 (TX/RX) mode: alternate function
                    GPIOB->AFR[0] |= ((7 << 20) | (7 << 24));  // PB5+6 (TX/RX) alternate function: AF7 (USART2)
                }
            }
            else if (usart == USART3)
            {
                if (!alternatePins)
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);  // enable GPIO peripheral
                    GPIOB->MODER |= ((2 << 20) | (2 << 22));  // PB10+11 (TX/RX) mode: alternate function                    
                    GPIOB->AFR[1] |= ((7 << 8) | (7 << 12));  // PB10+11 (TX/RX) alternate function: AF7 (USART3)                    
                }
                else
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);  // enable GPIO peripheral
                    GPIOD->MODER |= ((2 << 16) | (2 << 18));  // PD8+9 (TX/RX) mode: alternate function                    
                    GPIOD->AFR[1] |= ((7 << 8) | (7 << 12));  // PD8 (TX/RX) alternate function: AF7 (USART3)                    
                }
            }
            else if (usart == USART6)   // Only one Pin-Option
            {
                RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);  // enable GPIO peripheral
                GPIOC->MODER |= ((2 << 12) | (2 << 14));  // PC6+7 (TX/RX) mode: alternate function                
                GPIOC->AFR[0] |= ((8 << 24) | (8 << 28)); // PC6 (TX) alternate function: AF8 (USART6)
            }
            else if (usart == UART4)
            {
                RCC->APB1ENR |= (RCC_APB1ENR_UART4EN); // enable UART peripheral

                if (!alternatePins)
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral
                    GPIOA->MODER |= ((2 << 0) | (2 << 2));  // PA0+1 (TX/RX) mode: alternate function                    
                    GPIOA->AFR[0] |= ((8 << 0) | (8 << 4));  // PA0+1 (TX/RX) alternate function: AF8 (UART4)                    
                }
                else
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);  // enable GPIO peripheral
                    GPIOC->MODER |= ((2 << 20) | (2 << 22));  // PC10+11 (TX/RX) mode: alternate function
                    GPIOC->AFR[1] |= ((8 << 8) | (8 << 12));  // PC10+11 (TX/RX) alternate function: AF8 (USART4)
                }
            }
            else if (usart == UART5)
            {
                RCC->APB1ENR |= (RCC_APB1ENR_UART5EN); // enable UART peripheral
                RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);  // enable GPIO peripheral
                RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);  // enable GPIO peripheral

                GPIOC->MODER |= (2 << 24);  // PC12 (TX) mode: alternate function
                GPIOD->MODER |= (2 << 4);  // PD2 (RX) mode: alternate function
                GPIOC->AFR[1] |= (8 << 16);  // PC12 (TX) alternate function: AF8 (UART5)
                GPIOD->AFR[0] |= (8 << 8);  // PD2 (RX) alternate function: AF8 (UART5_RX)            
            }

            return 1;
        }

        int SetupAsync(USART_TypeDef *usart, uint32_t baud, uint8_t alternatePins)
        {
            if (!IsUsart(usart)) return 0;

            EnablePeripheral(usart);
            ConfigureGpios(usart, alternatePins);
            
            if (!SetBaud(usart, baud)) return 0;

            return 1;
        }

        int SetBaud(USART_TypeDef *usart, uint32_t baud)
        {
            if (!IsUsart(usart)) return 0;

            switch (baud) 
            {
                case 9600:
                case 19200:
                case 28800:
                case 38400:
                case 57600:
                case 76800:
                case 115200:
                    break;

                default:
                    return 0;
            }

            float usartDiv = (float)SystemCoreClock / ((float)baud * 16.0f);
            uint16_t mentissa = (uint16_t)usartDiv;
            uint16_t fraction = (uint16_t)((usartDiv - mentissa) * 16.0f);
            usart->BRR |= (mentissa << 4);
            usart->BRR |= (fraction << 0);
            
            return 1;
        }

        int SetupRcvDma(USART_TypeDef *usart)
        {
            if (!IsUsart(usart)) return 0;

            DMA_Stream_TypeDef *stream = GetDmaStreamRcv(usart);
            DMA::EnablePeriperal(stream);
            DMA::EnableInterrupt(stream);

            stream->CR = 0x00;
            stream->CR |= (DMA_SxCR_EN);            // Disable DMA stream
            stream->CR |= ((1 << DMA_SxCR_PL_Pos) | // Priority level: meduim
                (DMA_SxCR_MINC) |                   // Memory increment mode: memory address pointer is incremented
                (DMA_SxCR_TCIE) |                   // Transfer complete interrupt: enabled
                (DMA_SxCR_HTIE) |                   // Half transfer interrupt: enabled
                (DMA_SxCR_TEIE) |                   // Transfer error interrupt: enabled
                (DMA_SxCR_DMEIE));                  // Direct mode error interrupt: enabled

            return 1;
        }

        int RcvDma(USART_TypeDef *usart, uint8_t *rxBuf, uint32_t size)
        {
            if (!IsUsart(usart)) return 0;

            DMA_Stream_TypeDef *stream = GetDmaStreamRcv(usart);
            DMA::SetupBuffer(stream, (uint32_t)&usart->DR, (uint32_t)rxBuf, size);

            return 1;
        }

        DMA_Stream_TypeDef* GetDmaStreamRcv(USART_TypeDef *usart)
        {
            DMA_Stream_TypeDef *dma = nullptr;

            if (usart == USART1) dma = DMA2_Stream2;
            else if (usart == USART2) dma = DMA1_Stream5;
            else if (usart == USART3) dma = DMA1_Stream1;
            else if (usart == USART6) dma = DMA2_Stream1;
            else if (usart == UART4) dma = DMA1_Stream2;
            else if (usart == UART5) dma = DMA1_Stream0;

            return dma;
        }

        int IsUsart(USART_TypeDef *usart)
        {
            if (usart == USART1 || usart == USART2 || usart == USART3 ||
                usart == USART6 || usart == UART4 || usart == UART5)
                return 1;

            return 0;
        }
    }
}