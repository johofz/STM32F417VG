#include "Usart.h"

namespace STM32F417VG
{
    namespace USART
    {
        int SetupAsync(USART_TypeDef *usart, uint32_t baud, uint8_t alternativPins)
        {
            if (!IsUsart(usart)) return 0;

            if (usart == USART1)
            {
                RCC->APB2ENR |= (RCC_APB2ENR_USART1EN); // enable USART peripheral

                if (!alternativPins)
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral
                    GPIOA->MODER |= (2 << 18);  // PA9 (TX) mode: alternate function
                    GPIOA->MODER |= (2 << 20);  // PA10 (RX) mode: alternate function
                    GPIOA->AFR[1] |= (7 << 4);  // PA9 (TX) alternate function: AF7 (USART1_TX)
                    GPIOA->AFR[1] |= (7 << 8);  // PA10 (RX) alternate function: AF7 (USART1_RX)
                }
                else
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
                    GPIOB->MODER |= (2 << 12);  // PB6 (TX) mode: alternate function
                    GPIOB->MODER |= (2 << 14);  // PB7 (RX) mode: alternate function
                    GPIOB->AFR[0] |= (7 << 24);  // PB6 (TX) alternate function: AF7 (USART1_TX)
                    GPIOB->AFR[0] |= (7 << 28);  // PB7 (RX) alternate function: AF7 (USART1_RX)
                }

            }
            else if (usart == USART2)
            {
                RCC->APB1ENR |= (RCC_APB1ENR_USART2EN); // enable USART peripheral

                if (!alternativPins)
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral
                    GPIOA->MODER |= (2 << 4);  // PA2 (TX) mode: alternate function
                    GPIOA->MODER |= (2 << 6);  // PA3 (RX) mode: alternate function
                    GPIOA->AFR[0] |= (7 << 8);  // PA2 (TX) alternate function: AF7 (USART2_TX)
                    GPIOA->AFR[0] |= (7 << 12);  // PA3 (RX) alternate function: AF7 (USART2_RX)
                }
                else
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
                    GPIOB->MODER |= (2 << 10);  // PB5 (TX) mode: alternate function
                    GPIOB->MODER |= (2 << 12);  // PB6 (RX) mode: alternate function
                    GPIOB->AFR[0] |= (7 << 20);  // PB5 (TX) alternate function: AF7 (USART2_TX)
                    GPIOB->AFR[0] |= (7 << 24);  // PB6 (RX) alternate function: AF7 (USART2_RX)
                }
            }
            else if (usart == USART3)
            {
                RCC->APB1ENR |= (RCC_APB1ENR_USART3EN); // enable USART peripheral

                if (!alternativPins)
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);  // enable GPIO peripheral
                    GPIOB->MODER |= (2 << 20);  // PB10 (TX) mode: alternate function
                    GPIOB->MODER |= (2 << 22);  // PB11 (RX) mode: alternate function
                    GPIOB->AFR[1] |= (7 << 8);  // PB10 (TX) alternate function: AF7 (USART3_TX)
                    GPIOB->AFR[1] |= (7 << 12); // PB11 (RX) alternate function: AF7 (USART3_RX)
                }
                else
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);  // enable GPIO peripheral
                    GPIOD->MODER |= (2 << 16);  // PD8 (TX) mode: alternate function
                    GPIOD->MODER |= (2 << 18);  // PD9 (RX) mode: alternate function
                    GPIOD->AFR[1] |= (7 << 8);  // PD8 (TX) alternate function: AF7 (USART3_TX)
                    GPIOD->AFR[1] |= (7 << 12); // PD9 (RX) alternate function: AF7 (USART3_RX)
                }

            }
            else if (usart == USART6)   // Only one Pin-Option
            {
                RCC->APB2ENR |= (RCC_APB2ENR_USART6EN); // enable USART peripheral
                RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);  // enable GPIO peripheral
                GPIOC->MODER |= (2 << 12);  // PC6 (TX) mode: alternate function
                GPIOC->MODER |= (2 << 14);  // PC7 (RX) mode: alternate function
                GPIOC->AFR[0] |= (8 << 24); // PC6 (TX) alternate function: AF8 (USART6_TX)
                GPIOC->AFR[0] |= (8 << 28); // PC7 (RX) alternate function: AF8 (USART6_RX)
            }
            else if (usart == UART4)
            {
                RCC->APB1ENR |= (RCC_APB1ENR_UART4EN); // enable UART peripheral

                if (!alternativPins)
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral
                    GPIOA->MODER |= (2 << 0);  // PA0 (TX) mode: alternate function
                    GPIOA->MODER |= (2 << 2);  // PA1 (RX) mode: alternate function
                    GPIOA->AFR[0] |= (8 << 0);  // PA0 (TX) alternate function: AF8 (UART4_TX)
                    GPIOA->AFR[0] |= (8 << 4);  // PA1 (RX) alternate function: AF8 (UART4_RX)
                }
                else
                {
                    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);  // enable GPIO peripheral
                    GPIOC->MODER |= (2 << 20);  // PC10 (TX) mode: alternate function
                    GPIOC->MODER |= (2 << 22);  // PC11 (RX) mode: alternate function
                    GPIOC->AFR[1] |= (8 << 8);  // PC10 (TX) alternate function: AF8 (USART4_TX)
                    GPIOC->AFR[1] |= (8 << 12); // PC11 (RX) alternate function: AF8 (USART4_RX)
                }
            }
            else if (usart == UART5)
            {
                RCC->APB1ENR |= (RCC_APB1ENR_UART5EN); // enable UART peripheral
                RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);  // enable GPIO peripheral
                RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);  // enable GPIO peripheral

                GPIOC->MODER |= (2 << 24);  // PC12 (TX) mode: alternate function
                GPIOD->MODER |= (2 << 4);  // PD2 (RX) mode: alternate function
                GPIOC->AFR[1] |= (8 << 16);  // PC12 (TX) alternate function: AF8 (UART5_TX)
                GPIOD->AFR[0] |= (8 << 8);  // PD2 (RX) alternate function: AF8 (UART5_RX)            
            }

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

        int SetupDma(USART_TypeDef *usart)
        {
            if (!IsUsart(usart)) return 0;

            DMA_Stream_TypeDef *stream = GetDmaStreamRcv(usart);
            DMA::EnablePeriperal(stream);
            DMA::EnableInterrupt(stream);
            
            if (usart == USART1)
            {
                RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN);
                __NVIC_SetPriority(DMA2_Stream2_IRQn, 0);
                __NVIC_EnableIRQ(DMA2_Stream2_IRQn);
                stream = DMA2_Stream2;
            }
            else if (usart == USART2)
            {
                RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
                __NVIC_SetPriority(DMA1_Stream5_IRQn, 0);
                __NVIC_EnableIRQ(DMA1_Stream5_IRQn);
                stream = DMA1_Stream5;
            }
            else if (usart == USART3)
            {
                RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
                __NVIC_SetPriority(DMA1_Stream1_IRQn, 0);
                __NVIC_EnableIRQ(DMA1_Stream1_IRQn);
                stream = DMA1_Stream1;
            }
            else if (usart == USART6)
            {
                RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN);
                __NVIC_SetPriority(DMA2_Stream1_IRQn, 0);
                __NVIC_EnableIRQ(DMA2_Stream1_IRQn);
                stream = DMA2_Stream1;
            }
            else if (usart == UART4)
            {
                RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
                __NVIC_SetPriority(DMA1_Stream2_IRQn, 0);
                __NVIC_EnableIRQ(DMA1_Stream2_IRQn);
                stream = DMA1_Stream2;
            }
            else if (usart == UART5)
            {
                RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
                __NVIC_SetPriority(DMA1_Stream0_IRQn, 0);
                __NVIC_EnableIRQ(DMA1_Stream0_IRQn);
                stream = DMA1_Stream0;
            }

            stream->CR = 0x00;
            stream->CR |= (DMA_SxCR_EN);   // disable DMA stream

            stream->CR |= ((1 << DMA_SxCR_PL_Pos) |    // Priority level: meduim
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