#ifndef __USART_H__
#define __USART_H__

#include <stm32f417xx.h>
#include <Dma.h>
#include <Time.h>

#define USART_COUNT 6

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
            SUCCESS,
            NO_USART,
            INVALID_BAUD,
            ALREADY_INITIALIZED,
            NOT_INITIALIZED
        };

        typedef void (*UsartISR)(void);

        class Usart
        {
        public:
            ~Usart() {}

            UsartError Init(USART_TypeDef *usart, uint32_t baud, PinOption pinOption);
            UsartError DeInit();

            UsartError SetupReceiveIdle(UsartISR isr);
            UsartError SetupReceiveDma();

            UsartError Enable();
            UsartError Disable();
            UsartError EnableInterrupt();
            UsartError RegisterISR(UsartISR isr);
            UsartError ConfigureGpios(PinOption pinOption);
            UsartError SetBaud(uint32_t baud);

            UsartError Transmit(uint8_t *txBuf, uint32_t size);
            UsartError TransmitDMA(uint8_t *txBuf, uint32_t size);
            UsartError Receive(uint8_t *rxBuf, uint32_t size);
            UsartError ReceiveIdle(uint8_t *rxBuf, uint32_t size);
            UsartError ReceiveDma(uint8_t *rxBuf, uint32_t size);
            
            DMA_Stream_TypeDef* GetDmaStreamRx(USART_TypeDef *usart);
            DMA_Stream_TypeDef* GetDmaStreamTx(USART_TypeDef *usart);

        private:
            UsartError IsUsart(USART_TypeDef *usart);

            USART_TypeDef *m_usart;
            uint32_t m_baud;
            PinOption m_pinOption;
            uint8_t m_initialized;
            
        };

    }
}

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void USART6_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);

#endif // __USART_H__