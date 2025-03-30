#include "HaiTai.h"
#include "CAN.h"
#include "USART.h"

#if defined HAITAI_NUM && defined HAITAI_ID_OFFSET

HaiTai_t HaiTai[HAITAI_NUM];

void HaiTai_CAN_SendCmd(void *CAN_handle, unsigned char ID, unsigned char HAITAI_cmd)
{
    unsigned char arrID = ID - HAITAI_ID_OFFSET, TxData[4], len = 0;

    switch (HAITAI_cmd)
    {
    case HAITAI_CTRL_OFF:
    case HAITAI_CTRL_0POS_MPL:
    case HAITAI_CTRL_0POS_SGL:
        break;
    case HAITAI_CTRL_PWR:
    {
        len = 2;
        *(short *)TxData = LIMIT_RANGE(HaiTai[arrID].ctrl.pwr, HAITAI_PWR_MAX, HAITAI_PWR_MIN);
        break;
    }
    case HAITAI_CTRL_SPD:
    {
        len = 2;
        *(short *)TxData = LIMIT_RANGE(HaiTai[arrID].ctrl.spd, HAITAI_SPD_MAX, HAITAI_SPD_MIN) / HAITAI_fSPD;
        break;
    }
    case HAITAI_CTRL_POS_ABS:
    {
        len = 4;
        *(int *)TxData = LIMIT_RANGE(HaiTai[arrID].ctrl.pos, HAITAI_POS_MAX, HAITAI_POS_MIN) / HAITAI_fPOS;
        break;
    }
    case HAITAI_CTRL_POS_REL:
    {
        len = 4;
        *(int *)TxData = LIMIT_RANGE(HaiTai[arrID].ctrl.pos, HAITAI_POS_MAX, HAITAI_POS_MIN) / HAITAI_fPOS;
        break;
    }
    case HAITAI_CTRL_POS_SPD_CFG:
    {
        len = 3;
        TxData[0] = HAITAI_POS_SPD_LIM_W;
        *(short *)&TxData[1] = LIMIT_RANGE(HaiTai[arrID].ctrl.spd, HAITAI_SPD_MAX, HAITAI_SPD_MIN) / HAITAI_fSPD;
        break;
    }
    }

    CAN_SendData(CAN_handle, FDCAN_STANDARD_ID, HAITAI_cmd << 4 | ID, TxData, len);
}

void HaiTai_RS485_SendCmd(USART_info_t *UART_info, unsigned char ID, unsigned char HAITAI_cmd)
{
    unsigned char arrID = ID - HAITAI_ID_OFFSET;

    HaiTai[arrID].TxData[0] = HAITAI_HEAD_SEND;
    HaiTai[arrID].TxData[1] = HAITAI_PID;
    HaiTai[arrID].TxData[2] = ID;
    HaiTai[arrID].TxData[3] = HAITAI_cmd;

    switch (HAITAI_cmd)
    {
    case HAITAI_SYS_PARAM_W:
    case HAITAI_SYS_PARAM_P:
    {
        // UB
        return;
    }
    case HAITAI_CTRL_PWR:
    {
        HaiTai[arrID].TxData[4] = 2;
        *(short *)&HaiTai[arrID].TxData[5] = LIMIT_RANGE(HaiTai[arrID].ctrl.pwr, HAITAI_PWR_MAX, HAITAI_PWR_MIN);
        break;
    }
    case HAITAI_CTRL_SPD:
    {
        HaiTai[arrID].TxData[4] = 2;
        *(short *)&HaiTai[arrID].TxData[5] = LIMIT_RANGE(HaiTai[arrID].ctrl.spd, HAITAI_SPD_MAX, HAITAI_SPD_MIN) / HAITAI_fSPD;
        break;
    }
    case HAITAI_CTRL_POS_ABS:
    {
        HaiTai[arrID].TxData[4] = 4;
        *(int *)&HaiTai[arrID].TxData[5] = LIMIT_RANGE(HaiTai[arrID].ctrl.pos, HAITAI_POS_MAX, HAITAI_POS_MIN) / HAITAI_fPOS;
        break;
    }
    case HAITAI_CTRL_POS_REL:
    {
        HaiTai[arrID].TxData[4] = 4;
        *(int *)&HaiTai[arrID].TxData[5] = LIMIT_RANGE(HaiTai[arrID].ctrl.pos, HAITAI_POS_MAX, HAITAI_POS_MIN) / HAITAI_fPOS;
        break;
    }
    case HAITAI_CTRL_POS_SPD_CFG:
    {
        HaiTai[arrID].TxData[4] = 3;
        HaiTai[arrID].TxData[5] = HAITAI_POS_SPD_LIM_W;
        *(short *)&HaiTai[arrID].TxData[6] = LIMIT_RANGE(HaiTai[arrID].ctrl.spd, HAITAI_SPD_MAX, HAITAI_SPD_MIN) / HAITAI_fSPD;
        break;
    }
    }

    *(unsigned short *)&HaiTai[arrID].TxData[4 + HaiTai[arrID].TxData[4] + 1] = CRCsw_Calc(&CRC_16_Modbus, HaiTai[arrID].TxData, 5 + HaiTai[arrID].TxData[4]);

    UART_SendArray(UART_info, HaiTai[arrID].TxData, 7 + HaiTai[arrID].TxData[4]);
}

/*
void FDCAN1_IT0_IRQHandler(void)
{
    if (FDCAN1->IR & 0x1)
    {
        FDCAN1->IR |= 0x1;

        FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
        uint8_t RxData[64];
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxData);

        unsigned char arrID = (FDCAN_RxHeader.Identifier & 0xF) - HAITAI_ID_OFFSET;

        HaiTai[arrID].fdbk.pos_sgl = *(unsigned short *)RxData * HAITAI_fPOS;
        HaiTai[arrID].fdbk.pos_mpl = *(int *)&RxData[2] * HAITAI_fPOS;
        HaiTai[arrID].fdbk.spd = *(short *)&RxData[6] * HAITAI_fSPD;
    }
}
*/
#endif