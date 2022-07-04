#ifndef __USART_H__
#define __USART_H__

#include <stm32f417xx.h>
#include <Dma.h>

namespace STM32F417VG
{
    namespace USART
    {
        enum PinOption
        {
            DEFAULT_PINS,
            ALTERNATE_PINS
        };

        enum UsartError
        {
            NO_ERROR,
            NO_USART,
            INVALID_BAUD
        };

        UsartError Init(USART_TypeDef *usart, uint32_t baud, PinOption pinOption);

        UsartError SetupReceiveIdle(USART_TypeDef *usart, uint32_t baud, PinOption pinOption);
        UsartError SetupReceiveDma(USART_TypeDef *usart);
        
        UsartError SetBaud(USART_TypeDef *usart, uint32_t baud);
        UsartError EnablePeripheral(USART_TypeDef *usart);
        UsartError ConfigureGpios(USART_TypeDef *usart, PinOption pinOption);
        UsartError EnableInterrupt(USART_TypeDef *usart);
        
        UsartError Transmit(USART_TypeDef *usart, uint8_t *txBuf, uint32_t size);
        UsartError TransmitDMA(USART_TypeDef *usart, uint8_t *txBuf, uint32_t size);

        UsartError Receive(USART_TypeDef *usart, uint8_t *rxBuf, uint32_t size);
        UsartError ReceiveIdle(USART_TypeDef *usart, uint8_t *rxBuf, uint32_t size);
        UsartError ReceiveDma(USART_TypeDef *usart, uint8_t *rxBuf, uint32_t size);


        UsartError IsUsart(USART_TypeDef *usart);

        DMA_Stream_TypeDef* GetDmaStreamRx(USART_TypeDef *usart);
        DMA_Stream_TypeDef* GetDmaStreamTx(USART_TypeDef *usart);
    }
}

#endif // __USART_H__