#ifndef __DMA_H__
#define __DMA_H__

#include "stm32f417xx.h"

namespace STM32F417VG
{
    namespace DMA
    {
            int EnablePeriperal(DMA_Stream_TypeDef *stream);
            int EnableInterrupt(DMA_Stream_TypeDef *stream);
            int SetupBuffer(DMA_Stream_TypeDef *stream, uint32_t peripheralAdd, uint32_t memAdd, uint32_t size);
            int IsDma(DMA_TypeDef *dma);
            int IsDmaStream(DMA_Stream_TypeDef *stream);
    }
}

#endif // __DMA_H__