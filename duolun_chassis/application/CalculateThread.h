#ifndef __CalculateThread_H
#define __CalculateThread_H
#include "struct_typedef.h"
#include "feet_motor.h"

typedef enum {
  NOFORCE,     // 底盘无力
  STOP,        // 底盘停止
  FALLOW,      // 底盘跟随云台
  ROTING_CW,   // 顺时针小陀螺
  ROTING_CCW,  // 逆时针小陀螺
  HIGHSPEED    // 高速模式
} ChassisMode_e;

typedef struct {
  motor_measure_t Motor6020[4];
  motor_measure_t Motor3508[4];
  motor_measure_t YawMotor;
  ChassisMode_e Mode;

  float vx;
  float vy;
  float wz;

  float Current[8];
  float WheelSpeed[4];
  float WheelAngle[4];
  float speed_6020[4];
  float Power_pre;
  float Power_Proportion;
  uint8_t CapKey;

} Chassis_t;

extern Chassis_t Chassis;
extern uint8_t left_flag, right_flag;

void CalculateThread(void const *pvParameters);
void ChassisCurrentUpdate();

#endif
