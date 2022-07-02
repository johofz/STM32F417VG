#include <Dma.h>

namespace STM32F417VG
{
    namespace DMA
    {
        int EnablePeriperal(DMA_Stream_TypeDef *stream)
        {
            if (!IsDmaStream(stream))
                return 0;

            if (stream <= DMA1_Stream7)
                RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
            else
                RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN);
            
            return 1;
        }

        int EnableInterrupt(DMA_Stream_TypeDef *stream)
        {
            if (!IsDmaStream(stream)) return 0;

            IRQn_Type irq;  // Window WatchDog Interrupt as placeholder
            if (stream == DMA1_Stream0) irq = DMA1_Stream0_IRQn;
            else if (stream == DMA1_Stream1) irq = DMA1_Stream1_IRQn;
            else if (stream == DMA1_Stream2) irq = DMA1_Stream2_IRQn;
            else if (stream == DMA1_Stream3) irq = DMA1_Stream3_IRQn;
            else if (stream == DMA1_Stream4) irq = DMA1_Stream4_IRQn;
            else if (stream == DMA1_Stream5) irq = DMA1_Stream5_IRQn;
            else if (stream == DMA1_Stream6) irq = DMA1_Stream6_IRQn;
            else if (stream == DMA1_Stream7) irq = DMA1_Stream7_IRQn;
            else if (stream == DMA2_Stream0) irq = DMA2_Stream0_IRQn;
            else if (stream == DMA2_Stream1) irq = DMA2_Stream1_IRQn;
            else if (stream == DMA2_Stream2) irq = DMA2_Stream2_IRQn;
            else if (stream == DMA2_Stream3) irq = DMA2_Stream3_IRQn;
            else if (stream == DMA2_Stream4) irq = DMA2_Stream4_IRQn;
            else if (stream == DMA2_Stream5) irq = DMA2_Stream5_IRQn;
            else if (stream == DMA2_Stream6) irq = DMA2_Stream6_IRQn;
            else irq = DMA2_Stream7_IRQn;
            
            __NVIC_SetPriority(irq, 0);
            __NVIC_EnableIRQ(irq);

            return 1;
        }

        int SetupBuffer(DMA_Stream_TypeDef *stream, uint32_t peripheralAdd, uint32_t memAdd, uint32_t size)
        {
            if (!IsDmaStream(stream)) return 0;

            stream->CR &= ~(DMA_SxCR_EN);           // Disable DMA to make changes
            stream->NDTR = size;                    // Size of Buffer
            stream->PAR = peripheralAdd;            // Peripheral Address
            stream->M0AR = memAdd;                  // Memory Address
            stream->CR |= (DMA_SxCR_EN);            // Enable DMA

            return 1;
        }

        int IsDma(DMA_TypeDef *dma)
        {
            if (dma == DMA1 || dma == DMA2)
                return 1;

            return 0;
        }

        int IsDmaStream(DMA_Stream_TypeDef *stream)
        {
            if (stream == DMA1_Stream0 ||
                stream == DMA1_Stream1 ||
                stream == DMA1_Stream2 ||
                stream == DMA1_Stream3 ||
                stream == DMA1_Stream4 ||
                stream == DMA1_Stream5 ||
                stream == DMA1_Stream6 ||
                stream == DMA1_Stream7 ||
                stream == DMA2_Stream0 ||
                stream == DMA2_Stream1 ||
                stream == DMA2_Stream2 ||
                stream == DMA2_Stream3 ||
                stream == DMA2_Stream4 ||
                stream == DMA2_Stream5 ||
                stream == DMA2_Stream6 ||
                stream == DMA2_Stream7)
                return 1;

            return 0;
        }

    }
}