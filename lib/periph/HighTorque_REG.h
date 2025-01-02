#ifndef __HIGHTORQUE_REG_H
#define __HIGHTORQUE_REG_H

#include "user.h"

#if defined HIGHTORQUE_ID_OFFSET && defined HIGHTORQUE_NUM

#define HIGHTORQUE_REG_MODE 0x00
#define HIGHTORQUE_REG_POS_FDBK 0x01
#define HIGHTORQUE_REG_SPD_FDBK 0x02
#define HIGHTORQUE_REG_TORQUE_FDBK 0x03
#define HIGHTORQUE_REG_CURR_Q_FDBK 0x04
#define HIGHTORQUE_REG_CURR_D_FDBK 0x05
#define HIGHTORQUE_REG_VOLT 0x0D
#define HIGHTORQUE_REG_TEMP 0x0E
#define HIGHTORQUE_REG_ERROR 0x0F
#define HIGHTORQUE_REG_PWM_A 0x10
#define HIGHTORQUE_REG_PWM_B 0x11
#define HIGHTORQUE_REG_PWM_C 0x12
#define HIGHTORQUE_REG_VOLT_A 0x14
#define HIGHTORQUE_REG_VOLT_B 0x15
#define HIGHTORQUE_REG_VOLT_C 0x16
#define HIGHTORQUE_REG_FOC_VOLT_ANGLE 0x18
#define HIGHTORQUE_REG_FOC_VOLT_VOLT 0x19
#define HIGHTORQUE_REG_VOLT_D 0x1A
#define HIGHTORQUE_REG_VOLT_Q 0x1B
#define HIGHTORQUE_REG_CURR_D 0x1C
#define HIGHTORQUE_REG_CURR_Q 0x1D
#define HIGHTORQUE_REG_POS_CTRL 0x20
#define HIGHTORQUE_REG_SPD_CTRL 0x21
#define HIGHTORQUE_REG_TORQUE_OFFSET_CTRL 0x22
#define HIGHTORQUE_REG_KP 0x23
#define HIGHTORQUE_REG_KD 0x24
#define HIGHTORQUE_REG_POS_TORQUE_LIMIT 0x25
#define HIGHTORQUE_REG_POS_STOP 0x26
#define HIGHTORQUE_REG_SPD_LIMIT 0x28
#define HIGHTORQUE_REG_ACC_LIMIT 0x29
#define HIGHTORQUE_REG_TORQUE_KP 0x30
#define HIGHTORQUE_REG_TORQUE_KI 0x31
#define HIGHTORQUE_REG_TORQUE_KD 0x32
#define HIGHTORQUE_REG_TORQUE_OFFSET_FDBK 0x33
#define HIGHTORQUE_REG_POS_TORQUE_FDBK 0x34
#define HIGHTORQUE_REG_POS_MIN 0x40
#define HIGHTORQUE_REG_POS_MAX 0x41

#endif
#endif