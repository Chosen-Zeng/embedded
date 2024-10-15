#include "FDCAN.h"
#include "stm32g4xx_hal.h"
#include "C620.h"

#include <string.h>
#include <stdio.h>

#define ABS(X) ((X) >= 0 ? (X) : -(X))				  // 输出X绝对值
#define ABS_LIMIT(X, Y) (X >= 0 ? (X = Y) : (X = -Y)) // 将X限制为±Y

extern FDCAN_HandleTypeDef hfdcan1;

void C620_PID_Angle(FDCAN_HandleTypeDef *hfdcan, uint32_t ID);
void C620_SetI(FDCAN_HandleTypeDef *hfdcan, uint32_t ID);
void C620_PID_RPM(FDCAN_HandleTypeDef *hfdcan, uint32_t ID);

static struct PID
{
	float p[8];
	float i[8];
	float d[8];
	float pterm[8];
	float iterm[8];
	float dterm[8];
	int16_t deprev[8];
	int16_t decurr[8];
} RPM, Angle;
struct C620 C620_CTRL, C620_FDBK;
int16_t C620_lap[8];
static float time;

void C620_PID_Angle(FDCAN_HandleTypeDef *hfdcan, uint32_t ID)
{
	time = TIM6->CNT / 4000.f; // 时间间隔
	TIM6->CNT = 0;

	for (int count = 0; count < 8; count++)
	{
		C620_CTRL.Angle[count] *= GR_C620;

		if (ABS(Angle.pterm[count] = C620_CTRL.Angle[count] - (C620_FDBK.Angle[count] + C620_lap[count] * 360)) > ANGLE_pLIMIT) // 对pterm限幅 实现类似步进效果
			ABS_LIMIT(Angle.pterm[count], ANGLE_pLIMIT);
		Angle.p[count] = Angle.pterm[count] * ANGLE_Kp;

		if (ABS(Angle.pterm[count]) >= ANGLE_iSTART) // 积分分离
		{
			Angle.iterm[count] = 0;
		}
		else if (ABS(Angle.iterm[count]) <= ANGLE_iLIMIT) // 积分限幅
		{
			Angle.iterm[count] += Angle.pterm[count] * time;
			if (ABS(Angle.iterm[count]) > ANGLE_iLIMIT)
				ABS_LIMIT(Angle.iterm[count], ANGLE_iLIMIT);
		}
		Angle.i[count] = Angle.iterm[count] * ANGLE_Ki;

		Angle.dterm[count] = (Angle.decurr[count] - Angle.deprev[count]) / time; // 微分先行
		Angle.d[count] = Angle.dterm[count] * ANGLE_Kd;

		C620_CTRL.RPM[count] = Angle.p[count] + Angle.i[count] + Angle.d[count];
	}
	C620_PID_RPM(hfdcan, ID);
}

void C620_PID_RPM(FDCAN_HandleTypeDef *hfdcan, uint32_t ID)
{
#ifdef RPM_MODE
	time = TIM6->CNT / 4000.f;
	TIM6->CNT = 0;
#endif

	for (int count = 0; count < 8; count++)
	{
		if (ABS(RPM.pterm[count] = C620_CTRL.RPM[count] - C620_FDBK.RPM[count]) < RPM_pDEADBAND) // 死区
			RPM.pterm[count] = 0;
		RPM.p[count] = RPM.pterm[count] * RPM_Kp;

		if (ABS(RPM.pterm[count]) >= RPM_iSTART) // 积分分离
		{
			RPM.iterm[count] *= 0.4;
		}
		else if (ABS(RPM.iterm[count]) <= RPM_iLIMIT) // 积分限幅
		{
			RPM.iterm[count] += RPM.pterm[count] * time;
			if (ABS(RPM.iterm[count]) > RPM_iLIMIT)
				ABS_LIMIT(RPM.iterm[count], RPM_iLIMIT);
		}
		RPM.i[count] = RPM.iterm[count] * RPM_Ki;

		RPM.dterm[count] = (RPM.decurr[count] - RPM.deprev[count]) / time; // 微分先行
		RPM.d[count] = RPM.dterm[count] * RPM_Kd;

		C620_CTRL.I[count] = RPM.p[count] + RPM.i[count] + RPM.d[count];
	}
	C620_SetI(hfdcan, ID);
}

void C620_SetI(FDCAN_HandleTypeDef *hfdcan, uint32_t ID)
{
	static uint8_t C620[8];

	for (int count = 0; count <= 3; count++)
	{
		if (ABS(C620_CTRL.I[count]) > 20)
			ABS_LIMIT(C620_CTRL.I[count], 20);
		C620[count * 2] = (int16_t)(C620_CTRL.I[count] / fI_C620) >> 8;
		C620[count * 2 + 1] = (int16_t)(C620_CTRL.I[count] / fI_C620);
	}

	FDCAN_SendData(hfdcan, ID, C620, 8);
}
__weak void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
	uint8_t RxFifo0[8];
	if (hfdcan->Instance == FDCAN1)
	{
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxFifo0);
		int8_t count = FDCAN_RxHeader.Identifier - 0x200 - 1;

		C620_FDBK.Angle[count] = (int16_t)((RxFifo0[0] << 8) | RxFifo0[1]) * fANGLE_C620;
		C620_FDBK.RPM[count] = (int16_t)((RxFifo0[2] << 8) | RxFifo0[3]);
		C620_FDBK.I[count] = (int16_t)((RxFifo0[4] << 8) | RxFifo0[5]) * fI_C620;
		C620_FDBK.Temp[count] = RxFifo0[6];

		static int16_t angle[8];

		if (ABS(angle[count] - C620_FDBK.Angle[count]) >= 60) // 计圈
			angle[count] > C620_FDBK.Angle[count] ? C620_lap[count]++ : C620_lap[count]--;
		angle[count] = C620_FDBK.Angle[count];

		RPM.deprev[count] = RPM.decurr[count];
		RPM.decurr[count] = C620_CTRL.RPM[count] - C620_FDBK.RPM[count];

#ifdef ANGLE_MODE
		Angle.deprev[count] = Angle.decurr[count];
		if (ABS(Angle.decurr[count] = C620_CTRL.Angle[count] - (C620_FDBK.Angle[count] + C620_lap[count] * 360)) > 120) // 微分限幅
			ABS_LIMIT(Angle.decurr[count], 120);
#endif
	}
}