#ifndef __FDCAN_H
#define __FDCAN_H

#include "user.h"

static inline void CAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint32_t FDCAN_IDType, uint32_t ID, uint8_t TxData[], uint8_t length)
{
    FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
    FDCAN_TxHeader.Identifier = ID;
    FDCAN_TxHeader.IdType = FDCAN_IDType;
    FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN_TxHeader.DataLength = length;
    FDCAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    FDCAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    FDCAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCAN_TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN_TxHeader, TxData);
}

static inline void FDCAN_nBRS_SendData(FDCAN_HandleTypeDef *hfdcan, uint32_t FDCAN_IDType, uint32_t ID, uint8_t TxData[], uint32_t FDCAN_DLC)
{
    FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
    FDCAN_TxHeader.Identifier = ID;
    FDCAN_TxHeader.IdType = FDCAN_IDType;
    FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN_TxHeader.DataLength = FDCAN_DLC;
    FDCAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    FDCAN_TxHeader.FDFormat = FDCAN_FD_CAN;
    FDCAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCAN_TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN_TxHeader, TxData);
}

static inline void FDCAN_BRS_SendData(FDCAN_HandleTypeDef *hfdcan, uint32_t FDCAN_IDType, uint32_t ID, uint8_t TxData[], uint32_t FDCAN_DLC)
{
    FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
    FDCAN_TxHeader.Identifier = ID;
    FDCAN_TxHeader.IdType = FDCAN_IDType;
    FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN_TxHeader.DataLength = FDCAN_DLC;
    FDCAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    FDCAN_TxHeader.FDFormat = FDCAN_FD_CAN;
    FDCAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCAN_TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN_TxHeader, TxData);
}

#endif