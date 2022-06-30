#ifndef __USART_H__
#define __USART_H__

#include "stm32f417xx.h"
#include "Dma.h"

namespace STM32F417VG
{
    namespace USART
    {
        int EnablePeripheral(USART_TypeDef *usart);
        int ConfigureGpios(USART_TypeDef *usart, uint8_t alternatePins);
        int SetupAsync(USART_TypeDef *usart, uint32_t baud, uint8_t alternatePins);
        int SetBaud(USART_TypeDef *usart, uint32_t baud);
        int SetupRcvDma(USART_TypeDef *usart);
        int RcvDma(USART_TypeDef *usart, uint8_t *rxBuf, uint32_t size);
        int IsUsart(USART_TypeDef *usart);

        DMA_Stream_TypeDef* GetDmaStreamRcv(USART_TypeDef *usart);
    }
}

#endif // __USART_H__