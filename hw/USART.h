#ifndef __USART_H
#define __USART_H

#include "user.h"
#include "TIM.h"
#include "CRC.h"

typedef const struct
{
    USART_TypeDef *USART_handle;
    float timeout;
    DMA_TypeDef *DMA_handle;
    void *DMA_subhandle;
    unsigned char DMA_ID;
} USART_info_t;

static inline unsigned char UART_SendData(USART_info_t *USART_info, unsigned char TxData)
{
#if defined STM32H7
    if (USART_info->USART_handle->ISR & 0x80) // TXE
    {
        USART_info->USART_handle->TDR = TxData;
#elif (defined STM32F4 || \
       defined STM32F1)
    if (USART_info->USART_handle->SR & 0x80) // TXE
    {
        USART_info->USART_handle->DR = TxData;
#else
#error No UART cfg.
#endif
        return 0;
    }
    return 1;
}

#define UART_TIMEOUT_ARRAY 0.0005
// @note 0.5ms timeout by default
static inline void UART_SendArray(USART_info_t *USART_info, unsigned char TxData[], unsigned char len)
{

#if (defined STM32H7 || \
     defined STM32F4)
    if (USART_info->DMA_handle && USART_info->DMA_subhandle && ((DMA_Stream_TypeDef *)USART_info->DMA_subhandle)->PAR && // DMA cfg
        USART_info->USART_handle->CR3 & 0x80)                                                                            // DMAT
    {
        // clear flags
        if (USART_info->DMA_ID > 5)
            USART_info->DMA_handle->HIFCR |= 0x20 << (6 * (USART_info->DMA_ID - 4) + 4);
        else if (USART_info->DMA_ID > 3)
            USART_info->DMA_handle->HIFCR |= 0x20 << 6 * (USART_info->DMA_ID - 4);
        else if (USART_info->DMA_ID > 1)
            USART_info->DMA_handle->LIFCR |= 0x20 << (6 * USART_info->DMA_ID + 4);
        else
            USART_info->DMA_handle->LIFCR |= 0x20 << 6 * USART_info->DMA_ID;

        ((DMA_Stream_TypeDef *)USART_info->DMA_subhandle)->NDTR = len;
        ((DMA_Stream_TypeDef *)USART_info->DMA_subhandle)->M0AR = (unsigned)TxData;
        ((DMA_Stream_TypeDef *)USART_info->DMA_subhandle)->CR |= 1;
    }
    else
#elif defined STM32F1
    if (USART_info->DMA_handle && USART_info->DMA_subhandle && ((DMA_Channel_TypeDef *)USART_info->DMA_subhandle)->PAR && // DMA cfg
        USART_info->USART_handle->CR3 & 0x80)                                                                             // DMAT
    {
        // clear flags
        USART_info->DMA_handle->IFCR |= 0x7 << 4 * (USART_info->DMA_ID - 1);

        ((DMA_Channel_TypeDef *)USART_info->DMA_subhandle)->CCR &= ~1;
        ((DMA_Channel_TypeDef *)USART_info->DMA_subhandle)->CNDTR = len;
        ((DMA_Channel_TypeDef *)USART_info->DMA_subhandle)->CMAR = (unsigned)TxData;
        ((DMA_Channel_TypeDef *)USART_info->DMA_subhandle)->CCR |= 1;
    }
    else
#else
#warning No DMA cfg.
#endif
    {
#ifdef TIMER
        timer_t USART_time = timer_InitStruct;
        unsigned short cnt = 0;
        while (!Timer_CheckTimeout(&USART_time, USART_info->timeout ? USART_info->timeout : UART_TIMEOUT_ARRAY) && cnt < len)
            if (!UART_SendData(USART_info, TxData[cnt]))
                cnt++;
#else
#warning UART may block the program.
        unsigned short cnt = 0;
        while (cnt < len)
            if (!UART_SendData(USART_info, TxData[cnt]))
                cnt++;
#endif
    }
}

#define MODBUS_R_COIL 0x01
#define MODBUS_R_DISCRETE_INPUT 0x02
#define MODBUS_R_HOLDING_REG 0x03
#define MODBUS_R_INPUT_REG 0x04
#define MODBUS_W_SGL_COIL 0x05
#define MODBUS_W_SGL_REG 0x06
#define MODBUS_W_MPL_COIL 0x0F
#define MODBUS_W_MPL_REG 0x10

static inline void Modbus_UART_Read(USART_info_t *USART_info, unsigned char slave_addr, unsigned char MODBUS_R_TYPE, unsigned short addr, unsigned short num)
{
    unsigned char TxData[8] = {slave_addr,
                               MODBUS_R_TYPE,
                               addr << 8,
                               addr,
                               num << 8,
                               num};
    *(unsigned short *)&TxData[6] = CRCsw_Calc(&CRC_16_MODBUS, TxData, 6);

    UART_SendArray(USART_info, TxData, 8);
}

static inline void Modbus_UART_WriteSgl(USART_info_t *USART_info, unsigned char slave_addr, unsigned char MODBUS_W_TYPE, unsigned short addr, unsigned short val)
{
    unsigned char TxData[8] = {slave_addr,
                               MODBUS_W_TYPE,
                               addr << 8,
                               addr};

    if (MODBUS_W_TYPE == MODBUS_W_SGL_COIL)
        *(unsigned short *)&TxData[4] = val ? 0xFF : 0;
    else if (MODBUS_W_TYPE == MODBUS_W_SGL_REG)
    {
        TxData[4] = val << 8;
        TxData[5] = val;
    }
    *(unsigned short *)&TxData[6] = CRCsw_Calc(&CRC_16_MODBUS, TxData, 6);

    UART_SendArray(USART_info, TxData, 8);
}

// void USART3_IRQHandler(void)
// {
//     static unsigned char RxData[17], cnt;
//     // new msg
//     if (USART3->ISR & 0x20)
//     {
//         RxData[cnt++] = USART3->RDR;
//     }
//     // idle
//     else if (USART3->ISR & 0x10)
//     {
//         USART3->ICR |= 0x10;
//         cnt = 0;
//     }
//     // overrun
//     else if (USART3->ISR & 0x8)
//     {
//         USART3->ICR |= 0x8;
//     }
// }

#endif