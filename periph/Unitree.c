#include "Unitree.h"
#include "USART.h"
#include "algorithm.h"
#include "CRC.h"

#if defined GO_M8010_6_NUM && defined GO_M8010_6_ID_OFFSET

GO_M8010_6_t GO_M8010_6[GO_M8010_6_NUM];

void GO_M8010_6_SendParam(USART_info_t *UART_info, unsigned char ID)
{
    unsigned char arrID = ID - GO_M8010_6_ID_OFFSET;

    *(unsigned short *)GO_M8010_6[arrID].TxData = GO_M8010_6_HEAD_SEND;
    GO_M8010_6[arrID].TxData[2] = ID | GO_M8010_6_MODE_FOC << 4;
    *(short *)&GO_M8010_6[arrID].TxData[3] = LIMIT_ABS(GO_M8010_6[arrID].ctrl.trq, GO_M8010_6_TRQ_LIMIT) * GO_M8010_6_fTRQ;
    *(short *)&GO_M8010_6[arrID].TxData[5] = LIMIT_ABS(GO_M8010_6[arrID].ctrl.spd, GO_M8010_6_SPD_LIMIT) * GO_M8010_6_fSPD;
    *(int *)&GO_M8010_6[arrID].TxData[7] = LIMIT_ABS(GO_M8010_6[arrID].ctrl.pos, GO_M8010_6_POS_LIMIT) * GO_M8010_6_fPOS;
    *(unsigned short *)&GO_M8010_6[arrID].TxData[11] = LIMIT(GO_M8010_6[arrID].ctrl.Kp, GO_M8010_6_Kp_LIMIT) * GO_M8010_6_fKp;
    *(unsigned short *)&GO_M8010_6[arrID].TxData[13] = LIMIT(GO_M8010_6[arrID].ctrl.Kd, GO_M8010_6_Kd_LIMIT) * GO_M8010_6_fKd;
    *(unsigned short *)&GO_M8010_6[arrID].TxData[15] = CRCsw_Calc(&CRC_16_CCITT, GO_M8010_6[arrID].TxData, 15);

    UART_SendArray(UART_info, GO_M8010_6[arrID].TxData, 17);
}

void GO_M8010_6_Stop(USART_info_t *UART_info, unsigned char ID)
{
    unsigned char arrID = ID - GO_M8010_6_ID_OFFSET;

    *(unsigned short *)GO_M8010_6[arrID].TxData = GO_M8010_6_HEAD_SEND;
    GO_M8010_6[arrID].TxData[2] = ID | GO_M8010_6_MODE_STOP << 4;
    *(unsigned short *)&GO_M8010_6[arrID].TxData[15] = CRCsw_Calc(&CRC_16_CCITT, GO_M8010_6[arrID].TxData, 15);

    UART_SendArray(UART_info, GO_M8010_6[arrID].TxData, 17);
}

// void USART3_IRQHandler(void)
// {
//     static unsigned char RxData[16], cnt;
//     if (USART3->ISR & 0x20) // new message
//     {
//         RxData[cnt++] = USART3->RDR;
//     }
//     else if (USART3->ISR & 0x10) // idle
//     {
//         USART3->ICR |= 0x10;
//         cnt = 0;

//         if (*(unsigned short *)RxData == GO_M8010_6_HEAD_RECV &&
//             !CRC_16_Cal(&CRC_16_CCITT, RxData, 16))
//         {
//             uint8_t arrID = (RxData[2] & 0x0F) - GO_M8010_6_ID_OFFSET;

//             GO_M8010_6[arrID].fdbk.mode = (RxData[2] & 0x70) >> 4;
//             GO_M8010_6[arrID].fdbk.trq = *(short *)&RxData[3] / GO_M8010_6_fTRQ;
//             GO_M8010_6[arrID].fdbk.spd = *(short *)&RxData[5] / GO_M8010_6_fSPD;
//             GO_M8010_6[arrID].fdbk.pos = *(int *)&RxData[7] / GO_M8010_6_fPOS;
//             GO_M8010_6[arrID].fdbk.temp = RxData[11];
//             GO_M8010_6[arrID].fdbk.err_motor = RxData[12] & 0x7;
//             GO_M8010_6[arrID].fdbk.footforce = (*(unsigned short *)&RxData[12] & 0xEFF8) >> 3;
//         }
//     }
//     else if (USART3->ISR & 0x80) // overrun
//     {
//         USART3->ICR |= 0x80;
//     }
// }

// void DMA1_Stream2_IRQHandler(void)
// {
//     if (DMA1->LISR & 0x200000)
//     {
//         DMA1->LIFCR |= 0x200000;

//         if (*(unsigned short *)RxData_D1S2 == GO_M8010_6_HEAD_RECV &&
//             !CRC_Cal(&CRC_16_CCITT, RxData_D1S2, 16))
//         {
//             uint8_t arrID = (RxData_D1S2[2] & 0x0F) - GO_M8010_6_ID_OFFSET;

//             GO_M8010_6[arrID].fdbk.trq = *(short *)&RxData_D1S2[3] / GO_M8010_6_fTRQ;
//             GO_M8010_6[arrID].fdbk.spd = *(short *)&RxData_D1S2[5] / GO_M8010_6_fSPD;
//             GO_M8010_6[arrID].fdbk.pos = *(int *)&RxData_D1S2[7] / GO_M8010_6_fPOS;
//             GO_M8010_6[arrID].fdbk.temp = RxData_D1S2[11];
//             GO_M8010_6[arrID].fdbk.err_motor = RxData_D1S2[12] & 0x7;
//             GO_M8010_6[arrID].fdbk.footforce = (*(unsigned short *)&RxData_D1S2[12] & 0xEFF8) >> 3;
//         }
//     }
// }
#endif

#if defined A1_NUM && defined A1_ID_OFFSET
#warning bug: no CRC algorithm matched

A1_t A1[A1_NUM + 1];

void A1_SendParam(USART_info_t *UART_info, unsigned char ID)
{
    unsigned char arrID = ID == A1_ID_BCAST ? A1_NUM : ID - A1_ID_OFFSET;

    *(unsigned short *)A1[arrID].TxData = A1_HEAD_SEND;
    A1[arrID].TxData[2] = ID;
    A1[arrID].TxData[4] = A1_MODE_FOC;
    A1[arrID].TxData[5] = 0xFF;
    *(short *)&A1[arrID].TxData[12] = LIMIT_ABS(A1[arrID].ctrl.trq, A1_TRQ_LIMIT) * A1_fTRQ;
    *(short *)&A1[arrID].TxData[14] = LIMIT_ABS(A1[arrID].ctrl.spd, A1_SPD_LIMIT) * A1_fSPD_MOTOR;
    *(int *)&A1[arrID].TxData[16] = LIMIT_ABS(A1[arrID].ctrl.pos, A1_POS_LIMIT) * A1_fPOS_MOTOR;
    *(unsigned short *)&A1[arrID].TxData[20] = LIMIT(A1[arrID].ctrl.Kp, A1_Kp_LIMIT) * A1_fKp;
    *(unsigned short *)&A1[arrID].TxData[22] = LIMIT(A1[arrID].ctrl.Kd, A1_Kd_LIMIT) * A1_fKd;
    *(unsigned *)&A1[arrID].TxData[30] = CRC_Calc(A1[arrID].TxData, 30, CRC_DATA_WORD);

    UART_SendArray(UART_info, A1[arrID].TxData, 34);
}

void A1_Stop(USART_info_t *UART_info, unsigned char ID)
{
    unsigned char arrID = ID == A1_ID_BCAST ? A1_NUM : ID - A1_ID_OFFSET;

    *(unsigned short *)A1[arrID].TxData = A1_HEAD_SEND;
    A1[arrID].TxData[2] = ID;
    A1[arrID].TxData[4] = A1_MODE_STOP;
    A1[arrID].TxData[5] = 0xFF;
    *(unsigned *)&A1[arrID].TxData[30] = CRC_Calc(A1[arrID].TxData, 30, CRC_DATA_WORD);

    UART_SendArray(UART_info, A1[arrID].TxData, 34);
}

/* __attribute__((section(".ARM.__at_0x24000000"))) unsigned char RxData_D1S1[78];
void DMA1_Stream1_IRQHandler(void)
{
    if (DMA1->LISR & 0x800)
    {
        DMA1->LIFCR |= 0x800;

        if (*(unsigned short *)RxData_D1S1 == A1_HEAD_RECV &&
            !CRC_Calc(&CRC_32_MPEG_2, RxData_D1S1, 74))
        {
            unsigned char arrID = RxData_D1S1[2] - A1_ID_OFFSET;

            A1[arrID].fdbk.temp.motor = RxData_D1S1[6];
            A1[arrID].fdbk.err.motor = RxData_D1S1[7];
            A1[arrID].fdbk.trq = *(short *)&RxData_D1S1[12] / A1_fTRQ;
            A1[arrID].fdbk.spd.motor = *(short *)&RxData_D1S1[14] / A1_fSPD_MOTOR;
            A1[arrID].fdbk.spd.motor_fltr = *(float *)&RxData_D1S1[16];
            A1[arrID].fdbk.spd.enc = *(short *)&RxData_D1S1[20] / A1_fSPD_ENC;
            A1[arrID].fdbk.spd.enc_fltr = *(float *)&RxData_D1S1[22];
            A1[arrID].fdbk.acc.motor = *(short *)&RxData_D1S1[26] / A1_GR;
            A1[arrID].fdbk.acc.enc = *(short *)&RxData_D1S1[28];
            A1[arrID].fdbk.pos.motor = *(int *)&RxData_D1S1[30] / A1_fPOS_MOTOR;
            // A1[arrID].fdbk.pos.enc = *(int *)&RxData_D1S1[34];
            A1[arrID].fdbk.IMU.board.gyro.x = *(short *)&RxData_D1S1[38] / A1_fGYRO;
            A1[arrID].fdbk.IMU.board.gyro.y = *(short *)&RxData_D1S1[40] / A1_fGYRO;
            A1[arrID].fdbk.IMU.board.gyro.z = *(short *)&RxData_D1S1[42] / A1_fGYRO;
            A1[arrID].fdbk.IMU.board.acc.x = *(short *)&RxData_D1S1[44] / A1_fACC;
            A1[arrID].fdbk.IMU.board.acc.y = *(short *)&RxData_D1S1[46] / A1_fACC;
            A1[arrID].fdbk.IMU.board.acc.z = *(short *)&RxData_D1S1[48] / A1_fACC;
            // A1[arrID].fdbk.IMU.foot.gyro.x = *(short *)&RxData_D1S1[50];
            // A1[arrID].fdbk.IMU.foot.gyro.y = *(short *)&RxData_D1S1[52];
            // A1[arrID].fdbk.IMU.foot.gyro.z = *(short *)&RxData_D1S1[54];
            // A1[arrID].fdbk.IMU.foot.acc.x = *(short *)&RxData_D1S1[56];
            // A1[arrID].fdbk.IMU.foot.acc.y = *(short *)&RxData_D1S1[58];
            // A1[arrID].fdbk.IMU.foot.acc.z = *(short *)&RxData_D1S1[60];
            // A1[arrID].fdbk.IMU.foot.mag.x = *(short *)&RxData_D1S1[62];
            // A1[arrID].fdbk.IMU.foot.mag.y = *(short *)&RxData_D1S1[64];
            // A1[arrID].fdbk.IMU.foot.mag.z = *(short *)&RxData_D1S1[66];
            A1[arrID].fdbk.temp.foot = RxData_D1S1[68] * A1_fTEMP_FOOT + A1_TEMP_FOOT_OFFSET;
            A1[arrID].fdbk.footforce = *(unsigned short *)&RxData_D1S1[69] << 16 | RxData_D1S1[71];
            A1[arrID].fdbk.err.foot = RxData_D1S1[72];
        }
    }
}
 */
#endif