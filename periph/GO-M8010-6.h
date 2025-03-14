#ifndef __GO_M8010_6_H
#define __GO_M8010_6_H

#include "user.h"

#if defined GO_M8010_6_NUM && defined GO_M8010_6_ID_OFFSET
#include "algorithm.h"

#define GO_M8010_6_GR (19 / 3)

#define GO_M8010_6_HEAD_SEND 0xEEFE
#define GO_M8010_6_HEAD_RECV 0xEEFD

#define GO_M8010_6_fTORQUE 255.f
#define GO_M8010_6_fSPD (255 / R2D * GO_M8010_6_GR)
#define GO_M8010_6_fPOS (32767 / R2D * GO_M8010_6_GR)
#define GO_M8010_6_fKpos 1279.f
#define GO_M8010_6_fKspd 1279.f

#define GO_M8010_6_TORQUE_LIMIT 128
#define GO_M8010_6_SPD_LIMIT (128 * R2D / GO_M8010_6_GR)
#define GO_M8010_6_POS_LIMIT (65536 * R2D / GO_M8010_6_GR)
#define GO_M8010_6_Kpos_LIMIT 25.6
#define GO_M8010_6_Kspd_LIMIT 25.6

#define GO_M8010_6_MOTOR_ERR_NONE 0
#define GO_M8010_6_MOTOR_ERR_OH 1
#define GO_M8010_6_MOTOR_ERR_OC 2
#define GO_M8010_6_MOTOR_ERR_OV 3
#define GO_M8010_6_MOTOR_ERR_ENC_FAULT 4
#define GO_M8010_6_MOTOR_ERR_BUSBAR_UV 5
#define GO_M8010_6_MOTOR_ERR_WINDING_OH 6

typedef struct
{
    unsigned char mode : 3;
    float torque;
    float spd;
    float pos;
    float Kpos;
    float Kspd;
} GO_M8010_6_ctrl_t;

typedef struct
{
    unsigned char mode : 3;
    float torque;
    float spd;
    float pos;
    char temp;
    unsigned short err_motor : 3;
    unsigned short force : 12;
} GO_M8010_6_fdbk_t;

typedef struct
{
    GO_M8010_6_ctrl_t ctrl;
    GO_M8010_6_fdbk_t fdbk;
} GO_M8010_6_t;
extern GO_M8010_6_t GO_M8010_6[GO_M8010_6_NUM];

typedef struct
{
    unsigned char TxData[17], RxData[16];
} GO_M8010_6_data_t;
extern GO_M8010_6_data_t GO_M8010_6_data[DMA_Stream_PAIR];

#define GO_M8010_6_MODE_LOCK 0
#define GO_M8010_6_MODE_FOC 1
#define GO_M8010_6_MODE_ENC_CAL 2

void GO_M8010_6_SendCmd(USART_info_t *USART_info, unsigned char ID);

#endif
#endif