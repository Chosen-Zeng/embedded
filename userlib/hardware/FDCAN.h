#ifndef __FDCAN_H
#define __FDCAN_H

#include "stm32g4xx_hal.h"

void FDCAN_SendData(FDCAN_HandleTypeDef* hfdcan, uint32_t ID, uint8_t TxData[], uint8_t length);

#endif