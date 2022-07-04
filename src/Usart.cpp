#include <Usart.h>

namespace STM32F417VG
{
    namespace USART
    {
        UsartError _Error = NO_ERROR;

        UsartError Init(USART_TypeDef *usart, uint32_t baud, PinOption pinOption)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

            EnablePeripheral(usart);
            ConfigureGpios(usart, pinOption);
            SetBaud(usart, baud);

            return _Error;
        }

        UsartError SetupReceiveIdle(USART_TypeDef *usart, uint32_t baud, PinOption pinOption)
        {
            _Error = NO_ERROR;
            if (Init(usart, baud, pinOption) != NO_ERROR)
                return _Error;
            
            EnableInterrupt(usart);    
            usart->CR1 |= (USART_CR1_IDLEIE);
            SetupReceiveDma(usart);

            return _Error;
        }

        UsartError SetupReceiveDma(USART_TypeDef *usart)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

            DMA_Stream_TypeDef *stream = GetDmaStreamRx(usart);
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

            return _Error;
        }

        UsartError SetBaud(USART_TypeDef *usart, uint32_t baud)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

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
                    _Error = INVALID_BAUD;
                    return _Error;
            }

            float usartDiv = (float)SystemCoreClock / ((float)baud * 16.0f);
            uint16_t mentissa = (uint16_t)usartDiv;
            uint16_t fraction = (uint16_t)((usartDiv - mentissa) * 16.0f);
            usart->BRR |= (mentissa << 4);
            usart->BRR |= (fraction << 0);
            
            return _Error;
        }

        UsartError EnablePeripheral(USART_TypeDef *usart)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

            if (usart == USART1) RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);
            else if (usart == USART2) RCC->APB1ENR |= (RCC_APB1ENR_USART2EN);
            else if (usart == USART2) RCC->APB1ENR |= (RCC_APB1ENR_USART3EN);
            else if (usart == USART2) RCC->APB2ENR |= (RCC_APB2ENR_USART6EN);
            else if (usart == USART2) RCC->APB1ENR |= (RCC_APB1ENR_UART4EN);
            else if (usart == USART2) RCC->APB1ENR |= (RCC_APB1ENR_UART5EN);

            return _Error;
        }

        UsartError ConfigureGpios(USART_TypeDef *usart, PinOption pinOption)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

            if (usart == USART1)
            {
                if (pinOption == DEFAULT_PINS)
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
                if (pinOption == DEFAULT_PINS)
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
                if (pinOption == DEFAULT_PINS)
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

                if (pinOption == DEFAULT_PINS)
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

            return _Error;
        }

        UsartError EnableInterrupt(USART_TypeDef *usart)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

            IRQn_Type irq;
            if (usart == USART1) irq = USART1_IRQn;
            else if (usart == USART2) irq = USART2_IRQn;
            else if (usart == USART3) irq = USART3_IRQn; 
            else if (usart == USART6) irq = USART6_IRQn;
            else if (usart == UART4) irq = UART4_IRQn;
            else irq = UART5_IRQn;
            
            __NVIC_SetPriority(irq, 0);
            __NVIC_EnableIRQ(irq);

            return _Error;
        }
        
        UsartError Transmit(USART_TypeDef *usart, uint8_t *txBuf, uint32_t size)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

            for (uint32_t i = 0; i < size; i++)
            {
                usart->DR = txBuf[i];
                while (!(usart->SR & USART_SR_TXE));
            }
            while (!(usart->SR & USART_SR_TC));

            return _Error;
        }

        UsartError TransmitDMA(USART_TypeDef *usart, uint8_t *txBuf, uint32_t size)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

            DMA_Stream_TypeDef *stream = GetDmaStreamTx(usart);
            stream->CR &= ~(DMA_SxCR_EN);
            stream->PAR = (uint32_t)&usart->DR;

            return _Error;
        }

        UsartError Receive(USART_TypeDef *usart, uint8_t *rxBuf, uint32_t size)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;
            return _Error;
        }

        UsartError ReceiveIdle(USART_TypeDef *usart, uint8_t *rxBuf, uint32_t size)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;
            return _Error;
        }
        
        UsartError ReceiveDma(USART_TypeDef *usart, uint8_t *rxBuf, uint32_t size)
        {
            _Error = NO_ERROR;
            if (IsUsart(usart) != NO_ERROR)
                return _Error;

            DMA_Stream_TypeDef *stream = GetDmaStreamRx(usart);
            DMA::SetupBuffer(stream, (uint32_t)&usart->DR, (uint32_t)rxBuf, size);

            return _Error;
        }

        UsartError IsUsart(USART_TypeDef *usart)
        {
            _Error = NO_ERROR;
            if (usart == USART1 || usart == USART2 || usart == USART3 ||
                usart == USART6 || usart == UART4 || usart == UART5)
            {
                _Error = NO_ERROR;
                return _Error;
            }

            _Error = NO_USART;
            return _Error;
        }

        DMA_Stream_TypeDef* GetDmaStreamRx(USART_TypeDef *usart)
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

        // TODO: Korrekte Streams eintragen! (wurde nur kopert als Platzhalter)
        DMA_Stream_TypeDef* GetDmaStreamTx(USART_TypeDef *usart)
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
    }
}