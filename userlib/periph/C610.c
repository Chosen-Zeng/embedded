#include "main.h"
#include "C610.h"
#include "TIM.h"

#ifdef CAN_SUPPORT
#include "CAN.h"
#elif defined FDCAN_SUPPORT
#include "FDCAN.h"
#endif

#define ABS(X) ((X) >= 0 ? (X) : -(X)) // 输出X绝对值
#define ABS_LIMIT(X, Y) \
    if (ABS(X) > Y)     \
    X >= 0 ? (X = Y) : (X = -Y)

struct PID
{
    float p[8];
    float i[8];
    float d[8];
    float pterm[8];
    float iterm[8];
    float dterm[8];
    int16_t deprev[8];
    int16_t decurr[8];
} C610_PID_RPM, C610_PID_Angle;
C610_t C610_CTRL, C610_FDBK;
C610_Time_SRC_t C610_time_src =
#ifdef C610_MODE_RPM
    C610_TIME_SRC_RPM;
#elif defined C610_MODE_ANGLE
    C610_TIME_SRC_ANGLE;
#else
#error No mode selected.
#endif
static time_t C610_time;

static void C610_SetCurrent(void *CAN_handle, uint32_t C610_ID)
{
    uint8_t C610[16];

    for (int count = (C610_ID == C610_ID2 ? 4 : 0); count < (C610_ID == C610_ID1 ? 4 : 8); count++)
    {
        ABS_LIMIT(C610_CTRL.Current[count], C610_CURRENT_MAX);
        C610[count * 2] = (int16_t)(C610_CTRL.Current[count] / C610_fCURRENT) >> 8;
        C610[count * 2 + 1] = (int16_t)(C610_CTRL.Current[count] / C610_fCURRENT);
    }

    if (C610_ID == (C610_ID1 | C610_ID2))
    {
#ifdef CAN_SUPPORT
        CAN_SendData(CAN_handle, CAN_ID_STD, C610_ID1, C610, 8);
        CAN_SendData(CAN_handle, CAN_ID_STD, C610_ID2, &C610[8], 8);
#elif defined FDCAN_SUPPORT
        FDCAN_SendData(CAN_handle, FDCAN_STANDARD_ID, C610_ID1, C610, 8);
        FDCAN_SendData(CAN_handle, FDCAN_STANDARD_ID, C610_ID2, &C610[8], 8);
#endif
    }
    else
#ifdef CAN_SUPPORT
        CAN_SendData(CAN_handle, CAN_ID_STD, C610_ID, C610_ID == C610_ID1 ? C610 : &C610[8], 8);
#elif defined FDCAN_SUPPORT
        FDCAN_SendData(CAN_handle, FDCAN_STANDARD_ID, C610_ID, C610_ID == C610_ID1 ? C610 : &C610[8], 8);
#endif
}

void C610_SetRPM(void *CAN_handle, uint32_t C610_ID)
{
    if (C610_time_src == C610_TIME_SRC_RPM) // judge the time source(from lower or upper level)
        TIMSW_UpdateInterval(&C610_time);

    for (int count = (C610_ID == C610_ID2 ? 4 : 0); count < (C610_ID == C610_ID1 ? 4 : 8); count++)
    {
        C610_PID_RPM.pterm[count] = C610_CTRL.RPM[count] - C610_FDBK.RPM[count];
        C610_PID_RPM.p[count] = C610_PID_RPM.pterm[count] * C610_RPM_Kp;

        if (ABS(C610_PID_RPM.pterm[count]) >= C610_RPM_iSTART) // 积分分离
        {
            C610_PID_RPM.iterm[count] *= 0.4;
        }
        else if (ABS(C610_PID_RPM.iterm[count]) <= C610_RPM_iLIMIT) // 积分限幅
        {
            C610_PID_RPM.iterm[count] += C610_PID_RPM.pterm[count] * C610_time.interval;
            ABS_LIMIT(C610_PID_RPM.iterm[count], C610_RPM_iLIMIT);
        }
        C610_PID_RPM.i[count] = C610_PID_RPM.iterm[count] * C610_RPM_Ki;

        C610_PID_RPM.dterm[count] = (C610_PID_RPM.decurr[count] - C610_PID_RPM.deprev[count]) / C610_time.interval; // 微分先行
        C610_PID_RPM.d[count] = C610_PID_RPM.dterm[count] * C610_RPM_Kd;

        C610_CTRL.Current[count] = C610_PID_RPM.p[count] + C610_PID_RPM.i[count] + C610_PID_RPM.d[count];
    }
    C610_SetCurrent(CAN_handle, C610_ID);
}

void C610_SetAngle(void *CAN_handle, uint32_t C610_ID)
{
    if (C610_time_src == C610_TIME_SRC_ANGLE) // judge the time source(from lower or upper level)
        TIMSW_UpdateInterval(&C610_time);

    for (int count = (C610_ID == C610_ID2 ? 4 : 0); count < (C610_ID == C610_ID1 ? 4 : 8); count++)
    {
        C610_PID_Angle.pterm[count] = C610_CTRL.Angle[count] - C610_FDBK.Angle[count];
        C610_PID_Angle.p[count] = C610_PID_Angle.pterm[count] * C610_ANGLE_Kp;

        if (ABS(C610_PID_Angle.pterm[count]) >= C610_ANGLE_iSTART) // 积分分离
        {
            C610_PID_Angle.iterm[count] = 0;
        }
        else if (ABS(C610_PID_Angle.iterm[count]) <= C610_ANGLE_iLIMIT) // 积分限幅
        {
            C610_PID_Angle.iterm[count] += C610_PID_Angle.pterm[count] * C610_time.interval;
            ABS_LIMIT(C610_PID_Angle.iterm[count], C610_ANGLE_iLIMIT);
        }
        C610_PID_Angle.i[count] = C610_PID_Angle.iterm[count] * C610_ANGLE_Ki;

        C610_PID_Angle.dterm[count] = (C610_PID_Angle.decurr[count] - C610_PID_Angle.deprev[count]) / C610_time.interval; // 微分先行
        C610_PID_Angle.d[count] = C610_PID_Angle.dterm[count] * C610_ANGLE_Kd;

        C610_CTRL.RPM[count] = C610_PID_Angle.p[count] + C610_PID_Angle.i[count] + C610_PID_Angle.d[count];
    }
    C610_SetRPM(CAN_handle, C610_ID);
}

void C610_SetTorque(void *CAN_handle, uint32_t C610_ID)
{
    for (int count = (C610_ID == C610_ID2 ? 4 : 0); count < (C610_ID == C610_ID1 ? 4 : 8); count++)
    {
        C610_CTRL.Current[count] = C610_CTRL.Torque[count] / C610_fTORQUE;
    }
    C610_SetCurrent(CAN_handle, C610_ID);
}

#ifdef CAN_SUPPORT
#elif defined FDCAN_SUPPORT
__weak void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
    uint8_t RxFifo1[8];
    if (hfdcan->Instance == FDCAN2)
    {
        // sw reset unusable
        /*
        static struct // add delay at initialization to set sw zero point
        {
            uint8_t ZP_Status[8];
            float ZP[8];
        } C610_ZP;
        */
        static int16_t C610_lap[8];

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxHeader, RxFifo1);
        uint8_t count = FDCAN_RxHeader.Identifier - 0x200 - 1;

        static float angle_prev[8], angle_curr[8];
        /*
        if (!C610_ZP.ZP_Status[count])
        {
            angle_prev[count] = angle_curr[count] = C610_ZP.ZP[count] = ((RxFifo1[0] << 8) | RxFifo1[1]) * C610_fANGLE / C610_GR;
            C610_ZP.ZP_Status[count] = 1;
        }
        else
        */
        {
            angle_curr[count] = ((RxFifo1[0] << 8) | RxFifo1[1]) * C610_fANGLE / C610_GR;
            if (ABS(angle_prev[count] - angle_curr[count]) >= C610_DPS_MAX * C610_FDBK_RATE) // 计圈
                angle_prev[count] > angle_curr[count] ? C610_lap[count]++ : C610_lap[count]--;
            angle_prev[count] = angle_curr[count];
        }

        C610_FDBK.Angle[count] = angle_curr[count] + C610_lap[count] * 360 / C610_GR /*- C610_ZP.ZP[count]*/;
        C610_FDBK.RPM[count] = (int16_t)((RxFifo1[2] << 8) | RxFifo1[3]) / C610_GR;
        C610_FDBK.Torque[count] = (int16_t)((RxFifo1[4] << 8) | RxFifo1[5]) * C610_fCURRENT * C610_fTORQUE;

        C610_PID_RPM.deprev[count] = C610_PID_RPM.decurr[count];
        C610_PID_RPM.decurr[count] = C610_CTRL.RPM[count] - C610_FDBK.RPM[count];

#ifdef C610_MODE_ANGLE
        C610_PID_Angle.deprev[count] = C610_PID_Angle.decurr[count];
        C610_PID_Angle.decurr[count] = C610_CTRL.Angle[count] - C610_FDBK.Angle[count];
#endif
    }
}
#endif