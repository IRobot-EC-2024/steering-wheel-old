#include "CalculateThread.h"
#include "feet_motor.h"
#include "Remote.h"
#include "AttitudeThread.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Setting.h"
#include "user_lib.h"
#include "CanPacket.h"
#include "stdio.h"
#include "InterruptService.h"
#include "RefereeBehaviour.h"
#include "usart.h"
#include "MahonyAHRS.h"
#include "arm_math.h"
#include "CMS.h"

#define START_POWER 15.0f

Chassis_t Chassis;
RC_ctrl_t Remote;
EulerSystemMeasure_t Imu;
Aim_t Aim;
PTZ_t PTZ;
ext_game_robot_status_t Referee;
extern ext_power_heat_data_t power_heat_data_t;
uint32_t F_Motor[8];
float WheelAngle[4];

fp32 wz;
// 四个舵电机在轮电机尾部朝外，轮子朝前摆正时的反馈角度
fp32 Angle_zero_6020[4] = {30.9632568f, 129.720428f, 160.354034f, 109.415192f};
// 舵电机的旋转方向
fp32 Direction[5] = {-1.0, -1.0, 1.0, 1.0, -1.0};
fp32 Maxspeed = 6000.0f;
fp32 speed[4];
fp32 angle[4];
KFP Power_kf;
float angle_minus;
float run_per;

// power control
float last_speed[8] = {0};
fp32 he = 0;
float kp = 1.30 * 1.99999999e-06;
float lijupower = 0.0f;
uint8_t power_flag = 0;
float power_scale;
// 未开电容时的速度增益系数，开电容时的速度增益系数，最大功率
fp32 v_gain = 0, cap_gain = 1, Power_Max = 45.0f;
// 不同功率限制情况下的速度增益系数，在RefereeInfUpdate函数中查表更新到v_gain和cap_gain变量中
// 45w 50w 55w 60w 65w 70w 75w 80w 85w 90w 95w 100w 120w 130w 140w 150w 160w 170w 180w 190w 200w 默认值
fp32 v_gain_table[22] = {0.91f, 0.95f, 0.99f, 1.02f, 1.06f, 1.12f, 1.16f, 1.25f, 1.27f, 1.29f, 1.33f,
                         1.36f, 1.51f, 1.58f, 1.65f, 1.73f, 1.80f, 1.87f, 1.93f, 2.f,   2.1f,  0.85f},
     cap_gain_table[22] = {2.74f, 2.62f, 2.48f, 2.40f, 2.32f, 2.15f, 2.09f, 2.04f, 2.02f, 2.f,  1.9f,
                           1.84f, 1.8f,  1.4f,  1.3f,  1.3f,  1.2f,  1.2f,  1.1f,  1.1f,  1.1f, 1.f};

uint8_t Mode_last;
uint8_t Mode_now;
uint8_t stop_flag = 0;
int16_t stop_countdown = 0;

pid_type_def follow_yaw;
pid_type_def follow;
pid_type_def left_front_6020_speed_pid;
pid_type_def right_front_6020_speed_pid;
pid_type_def right_back_6020_speed_pid;
pid_type_def left_back_6020_speed_pid;
pid_type_def left_front_6020_position_pid;
pid_type_def right_front_6020_position_pid;
pid_type_def right_back_6020_position_pid;
pid_type_def left_back_6020_position_pid;
pid_type_def left_front_3508_pid;
pid_type_def right_front_3508_pid;
pid_type_def right_back_3508_pid;
pid_type_def left_back_3508_pid;
pid_type_def power_control_pid;
first_order_filter_type_t current_6020_filter_type;
first_order_filter_type_t current_3508_filter_type;
first_order_filter_type_t referee_power;
first_order_filter_type_t wheel_angle_1;
first_order_filter_type_t wheel_angle_2;
first_order_filter_type_t wheel_angle_3;
first_order_filter_type_t wheel_angle_4;
first_order_filter_type_t wz_filter;

fp32 follow_angle;
fp32 follow_yaw_PID[3] = {0.08, 0, 1};
fp32 follow_PID[3] = {FOLLOW_KP, FOLLOW_KI, FOLLOW_KD};
fp32 left_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_front_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 right_front_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 right_back_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 left_back_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 left_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 left_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 power_control_PID[3] = {power_control_KP * 1.2, power_control_KI, power_control_KD};

extern motor_measure_t LEFT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_BACK_6020_Measure;
extern motor_measure_t LEFT_BACK_6020_Measure;
extern motor_measure_t LEFT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_BACK_3508_Measure;
extern motor_measure_t LEFT_BACK_3508_Measure;
extern motor_measure_t YawMotorMeasure;
OfflineMonitor_t Offline;

void ChassisInit();
void ChassisModeUpdate();
void ChassisPidUpadte();
void ChassisCommandUpdate();
void ChassisCurrentUpdate();
void RefereeInfUpdate(ext_game_robot_status_t *referee);
void ChassisInfUpdate();
void Angle_Speed_calc();
void CMS__();
uint8_t chassis_powerloop(Chassis_t *Chassis);

float vx_last = 0, vy_last = 0;
void CalculateThread(void const *pvParameters) {
  ChassisInit();

  while (1) {
    // Remote = *get_remote_control_point();

    DeviceOfflineMonitorUpdate(&Offline);
    ChassisModeUpdate();
    ChassisInfUpdate();
    RefereeInfUpdate(&Referee);
    GimbalEulerSystemMeasureUpdate(&Imu);
    ChassisCommandUpdate();
    chassis_powerloop(&Chassis);
    CMS__();
    Chassis_Control(Chassis.Current[0], Chassis.Current[1], Chassis.Current[2], Chassis.Current[3], Chassis.Current[4],
                    Chassis.Current[5], Chassis.Current[6], Chassis.Current[7]);

    osDelay(1);
  }
}

void ChassisInit() {
  PID_init(&left_front_6020_speed_pid, PID_POSITION, left_front_6020_speed_PID, 30000, 10000);
  PID_init(&right_front_6020_speed_pid, PID_POSITION, right_front_6020_speed_PID, 30000, 10000);
  PID_init(&right_back_6020_speed_pid, PID_POSITION, right_back_6020_speed_PID, 30000, 10000);
  PID_init(&left_back_6020_speed_pid, PID_POSITION, left_back_6020_speed_PID, 30000, 10000);
  PID_init(&left_front_6020_position_pid, PID_POSITION, left_front_6020_position_PID, 300, 60);  // 6020
  PID_init(&right_front_6020_position_pid, PID_POSITION, right_front_6020_position_PID, 300, 60);
  PID_init(&right_back_6020_position_pid, PID_POSITION, right_back_6020_position_PID, 300, 60);
  PID_init(&left_back_6020_position_pid, PID_POSITION, left_back_6020_position_PID, 300, 60);

  PID_init(&left_front_3508_pid, PID_POSITION, left_front_3508_PID, 16384, 1000);
  PID_init(&right_front_3508_pid, PID_POSITION, right_front_3508_PID, 16384, 1000);
  PID_init(&right_back_3508_pid, PID_POSITION, right_back_3508_PID, 16384, 1000);  // 3508
  PID_init(&left_back_3508_pid, PID_POSITION, left_back_3508_PID, 16384, 1000);

  PID_init(&follow_yaw, PID_POSITION, follow_yaw_PID, 1, 1);
  PID_init(&follow, PID_POSITION, follow_PID, 2, 1);

  // KalmanFilter_init(&Power_kf, 0.0f , 0.0001f,0.0118f ,0.0,50.0,2.0);//A,B,P,Q,R                   //����
  first_order_filter_init(&current_6020_filter_type, 0.002, 0.1);
  first_order_filter_init(&current_3508_filter_type, 0.002, 0.1);
  first_order_filter_init(&wheel_angle_1, 0.001, 0.1);
  first_order_filter_init(&wheel_angle_2, 0.001, 0.1);
  first_order_filter_init(&wheel_angle_3, 0.001, 0.1);
  first_order_filter_init(&wheel_angle_4, 0.001, 0.1);

  CMS_Data.charge_flag = 1;
};

void ChassisInfUpdate() {
  memcpy(&Chassis.Motor3508[0], &LEFT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
  memcpy(&Chassis.Motor3508[1], &RIGHT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
  memcpy(&Chassis.Motor3508[2], &RIGHT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
  memcpy(&Chassis.Motor3508[3], &LEFT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
  memcpy(&Chassis.Motor6020[0], &LEFT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
  memcpy(&Chassis.Motor6020[1], &RIGHT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
  memcpy(&Chassis.Motor6020[2], &RIGHT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
  memcpy(&Chassis.Motor6020[3], &LEFT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
}

void ChassisModeUpdate() {
  // bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
  // 高速  逆转 电容  顺转  随动  有力 无力  通信正常
  switch (PTZ.ChassisStatueRequest) {
    case 0x01: {
      Chassis.Mode = NOFORCE;
      break;
    }
    case 0x12:
    case 0x32:
    case 0x52:
    case 0x72: {
      Chassis.Mode = ROTING_CW;
      break;
    }
    case 0x62:
    case 0xc2:
    case 0xe2:
    case 0x42: {
      Chassis.Mode = ROTING_CCW;
      break;
    }
    case 0x0A:
    case 0x2A: {
      Chassis.Mode = FALLOW;
      break;
    }
    case 0x06:
    case 0x26: {
      Chassis.Mode = STOP;
      break;
    }
    default:
      break;
  }
  if ((PTZ.ChassisStatueRequest & (0x01 << 5)) != 0) {
    Chassis.CapKey = 1;
  } else
    Chassis.CapKey = 0;
}

fp32 spin_angle_compensation = 0.37f;
void ChassisCommandUpdate() {
  // 无力或者云台离线
  if (Chassis.Mode == NOFORCE || Offline.PTZnode == 1) {
    Chassis.Current[0] = 0;
    Chassis.Current[1] = 0;
    Chassis.Current[2] = 0;
    Chassis.Current[3] = 0;
    Chassis.Current[4] = 0;
    Chassis.Current[5] = 0;
    Chassis.Current[6] = 0;
    Chassis.Current[7] = 0;
    return;
  }
  if (Chassis.Mode == FALLOW || Chassis.Mode == ROTING_CW || Chassis.Mode == ROTING_CCW || Chassis.Mode == STOP) {
    follow_angle = loop_fp32_constrain(FollowAngle, YawMotorMeasure.angle - 180.0f, YawMotorMeasure.angle + 180.0f);

    if (Chassis.Mode == FALLOW) {
      angle_minus = -YawMotorMeasure.angle + FollowAngle;
      if (angle_minus > 180)
        angle_minus -= 360;
      else if (angle_minus < -180)
        angle_minus += 360;
      Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus / 180.0 * PI) -
                    (PTZ.LRSpeed / 32767.0f) * sin(angle_minus / 180.0 * PI)) *
                   (v_gain);
      Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus / 180.0 * PI) +
                    (PTZ.LRSpeed / 32767.0f) * cos(angle_minus / 180.0 * PI)) *
                   (v_gain);
      Chassis.wz = -PID_calc(&follow, YawMotorMeasure.angle, follow_angle);
      if (Fabs(Chassis.wz) < 0.5 * v_gain && Fabs(angle_minus) < 0.5) {
        Chassis.wz = 0.001 * Chassis.wz / Fabs(Chassis.wz);
      }
    } else if (Chassis.Mode == ROTING_CW || Chassis.Mode == ROTING_CCW) {
      if (Power_Max <= 80) {
        Chassis.wz = sin(v_gain / 4.2) * 3.8f;  // shift慢转
      } else {
        Chassis.wz = sin(1.25f / 4.2) * 3.8f;  // 如果小陀螺功率过大，就限制到一个较小的转速
      }
      if ((PTZ.ChassisStatueRequest & 0b10000000) == 0b10000000) {  // bit7, 高速模式
        if (Power_Max <= 80) {
          Chassis.wz = sin(v_gain / 4.2) * 4.5f;  // ctrl快转
        } else {
          Chassis.wz = sin(1.25f / 4.2) * 4.5f;  // 如果小陀螺功率过大，就限制到一个较小的转速
        }
      }
      if (Chassis.Mode == ROTING_CCW) {
        Chassis.wz = -Chassis.wz;  // 反转
      }
      angle_minus = -YawMotorMeasure.angle + FollowAngle - YawMotorMeasure.speed_rpm * spin_angle_compensation;
      Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus / 180.0 * PI) -
                    (PTZ.LRSpeed / 32767.0f) * sin(angle_minus / 180.0 * PI)) *
                   v_gain / 1.8;  //* (1.0f + Chassis.Power_Proportion /Power_Max );
      Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus / 180.0 * PI) +
                    (PTZ.LRSpeed / 32767.0f) * cos(angle_minus / 180.0 * PI)) *
                   v_gain / 1.8;  //* (1.0f + Chassis.Power_Proportion /Power_Max );
    } else if (Chassis.Mode == STOP) {
      angle_minus = -YawMotorMeasure.angle + FollowAngle;
      Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus / 180.0 * PI) -
                    (PTZ.LRSpeed / 32767.0f) * sin(angle_minus / 180.0 * PI));
      Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus / 180.0 * PI) +
                    (PTZ.LRSpeed / 32767.0f) * cos(angle_minus / 180.0 * PI));
      Chassis.wz = 0.0;
    }
  }
  /********************************	舵电机解算      ***********************/

  if (Fabs(PTZ.FBSpeed / 32767.0) > 0.05 || Fabs(PTZ.LRSpeed / 32767.0) > 0.05) {
    for (uint8_t i = 0; i < 4;) {
      Chassis.WheelAngle[i] =
          atan2((Chassis.vy) + Chassis.wz * gen2 * Direction[i], (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1])) /
              3.1415927 * 180.0 +
          Angle_zero_6020[i];
      i++;
    }
    vx_last = Chassis.vx;
    vy_last = Chassis.vy;
    stop_flag = 1;
  } else {
    if (stop_flag == 1 && Chassis.Mode == FALLOW && Fabs(angle_minus) < 2.0) {
      stop_flag = 2;
      stop_countdown = 500;
    } else if (Fabs(angle_minus) > 3.0) {
      stop_flag = 0;
    }
    if (stop_countdown <= 0) stop_flag = 0;
    if (stop_flag == 2 &&
        ((Fabs(LEFT_BACK_3508_Measure.speed_rpm) > 100 || Fabs(RIGHT_BACK_3508_Measure.speed_rpm) > 100 ||
          Fabs(LEFT_FRONT_3508_Measure.speed_rpm) > 100 || Fabs(RIGHT_FRONT_3508_Measure.speed_rpm) > 100)) &&
        Chassis.Mode == FALLOW) {
      Chassis.wz = 0;
      for (uint8_t i = 0; i < 4;) {
        Chassis.WheelAngle[i] = atan2(vy_last, (vx_last)) / 3.1415927 * 180.0 + Angle_zero_6020[i];
        i++;
      }
      // stop_countdown--;
    } else {
      vx_last = 0;
      vy_last = 0;
      stop_flag = 0;
      if (Chassis.wz == 0) {
        Chassis.WheelAngle[0] = 0 + Angle_zero_6020[0] + angle_minus;
        Chassis.WheelAngle[1] = 0 + Angle_zero_6020[1] + angle_minus;
        Chassis.WheelAngle[2] = 0 + Angle_zero_6020[2] + angle_minus;
        Chassis.WheelAngle[3] = 0 + Angle_zero_6020[3] + angle_minus;
      } else if (Chassis.wz > 0) {
        Chassis.WheelAngle[0] = -135.0f + Angle_zero_6020[0];
        Chassis.WheelAngle[1] = -45.0f + Angle_zero_6020[1];
        Chassis.WheelAngle[2] = 45.0f + Angle_zero_6020[2];
        Chassis.WheelAngle[3] = 135.0f + Angle_zero_6020[3];
      } else if (Chassis.wz < 0) {
        Chassis.WheelAngle[0] = 45.0f + Angle_zero_6020[0];
        Chassis.WheelAngle[1] = 135.0f + Angle_zero_6020[1];
        Chassis.WheelAngle[2] = -135.0f + Angle_zero_6020[2];
        Chassis.WheelAngle[3] = -45.0f + Angle_zero_6020[3];
      }
    }
  }
  Chassis.WheelAngle[0] = loop_fp32_constrain(Chassis.WheelAngle[0], LEFT_FRONT_6020_Measure.angle - 180.0f,
                                              LEFT_FRONT_6020_Measure.angle + 180.0f);
  Chassis.WheelAngle[1] = loop_fp32_constrain(Chassis.WheelAngle[1], RIGHT_FRONT_6020_Measure.angle - 180.0f,
                                              RIGHT_FRONT_6020_Measure.angle + 180.0f);
  Chassis.WheelAngle[2] = loop_fp32_constrain(Chassis.WheelAngle[2], RIGHT_BACK_6020_Measure.angle - 180.0f,
                                              RIGHT_BACK_6020_Measure.angle + 180.0f);
  Chassis.WheelAngle[3] = loop_fp32_constrain(Chassis.WheelAngle[3], LEFT_BACK_6020_Measure.angle - 180.0f,
                                              LEFT_BACK_6020_Measure.angle + 180.0f);

  /***********************                 轮电机解算                    ******************************/

  if (((CMS_Data.cms_status) & (uint16_t)1) != 1 && CMS_Data.Mode == FLY) {
    // 如果超级电容状态正常，并且操作手按了电容键，就给速度目标乘上当前功率下的电容速度增益系数
    // 各个功率限制下增益系数的值可以在全局变量cap_gain_table中修改
    Chassis.vx = cap_gain * Chassis.vx;
    Chassis.vy = cap_gain * Chassis.vy;
  } else if (((CMS_Data.cms_status) & (uint16_t)1) != 1 && CMS_Data.Mode == HIGH_SPEED &&
             PTZ.ChassisStatueRequest & (0x01 << 7) && Power_Max <= 120) {
    // 如果超级电容状态正常并且有电，报告可以开高速模式，并且操作手按了高速键，且最大功率小于120w，就给速度目标乘上1.24
    Chassis.vx = 1.24f * Chassis.vx;
    Chassis.vy = 1.24f * Chassis.vy;
  }

  for (uint8_t i = 0; i < 4;) {
    speed[i] = sqrtf((Chassis.vy + Chassis.wz * gen2 * Direction[i]) * (Chassis.vy + Chassis.wz * gen2 * Direction[i]) +
                     (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]) *
                         (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]));
    i++;
  }
  Chassis.WheelSpeed[0] = -speed[0];
  Chassis.WheelSpeed[1] = speed[1];
  Chassis.WheelSpeed[2] = speed[2];
  Chassis.WheelSpeed[3] = -speed[3];

  Angle_Speed_calc();

  Chassis.speed_6020[0] = PID_calc(&left_front_6020_position_pid, LEFT_FRONT_6020_Measure.angle, Chassis.WheelAngle[0]);
  Chassis.speed_6020[1] =
      PID_calc(&right_front_6020_position_pid, RIGHT_FRONT_6020_Measure.angle, Chassis.WheelAngle[1]);
  Chassis.speed_6020[2] = PID_calc(&right_back_6020_position_pid, RIGHT_BACK_6020_Measure.angle, Chassis.WheelAngle[2]);
  Chassis.speed_6020[3] = PID_calc(&left_back_6020_position_pid, LEFT_BACK_6020_Measure.angle, Chassis.WheelAngle[3]);

  ChassisCurrentUpdate();

  Mode_last = Mode_now;
  Mode_now = Chassis.Mode;
}

void Angle_Speed_calc() {
  for (uint8_t i = 0; i < 4;) {
    if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle > 90.0f) {
      Chassis.WheelAngle[i] -= 180.0f;
      Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
    }
    if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle < -90.0f) {
      Chassis.WheelAngle[i] += 180.0f;
      Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
    }
    i++;
  }
}

void ChassisCurrentUpdate() {
  if (stop_flag == 2) {
    Chassis.WheelSpeed[0] = 0;
    Chassis.WheelSpeed[1] = 0;
    Chassis.WheelSpeed[2] = 0;
    Chassis.WheelSpeed[3] = 0;
    left_front_3508_pid.Kp = 6000;
    right_front_3508_pid.Kp = 6000;
    right_back_3508_pid.Kp = 6000;
    left_back_3508_pid.Kp = 6000;
  } else {
    left_front_3508_pid.Kp = 3600;
    right_front_3508_pid.Kp = 3600;
    right_back_3508_pid.Kp = 3600;
    left_back_3508_pid.Kp = 3600;
  }
  Chassis.Current[0] = PID_calc(&left_front_6020_speed_pid, LEFT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[0]);
  Chassis.Current[1] = PID_calc(&right_front_6020_speed_pid, RIGHT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[1]);
  Chassis.Current[2] = PID_calc(&right_back_6020_speed_pid, RIGHT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[2]);
  Chassis.Current[3] = PID_calc(&left_back_6020_speed_pid, LEFT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[3]);
  Chassis.Current[4] =
      PID_calc(&left_front_3508_pid, LEFT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[0]);
  Chassis.Current[5] =
      PID_calc(&right_front_3508_pid, RIGHT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[1]);
  Chassis.Current[6] =
      PID_calc(&right_back_3508_pid, RIGHT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[2]);
  Chassis.Current[7] =
      PID_calc(&left_back_3508_pid, LEFT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[3]);
  if (stop_flag == 2) {
    Chassis.Current[0] *= 1.2;
    Chassis.Current[1] *= 1.2;
    Chassis.Current[2] *= 1.2;
    Chassis.Current[3] *= 1.2;
  }
}

void RefereeInfUpdate(ext_game_robot_status_t *referee) {
  memcpy(referee, &robot_state, sizeof(ext_game_robot_status_t));
  // 依据当前最大功率查表，获取当前最大功率对应的v_gain和cap_gain
  // v_gain是速度增益系数，cap_gain是超级电容打开后的速度增益系数
  // referee->chassis_power_limit -> u16
  if (referee->chassis_power_limit >= 45 && referee->chassis_power_limit <= 200) {
    Power_Max = referee->chassis_power_limit;
    v_gain = v_gain_table[referee->chassis_power_limit / 5 - 9];
    cap_gain = cap_gain_table[referee->chassis_power_limit / 5 - 9];
  } else {
    Power_Max = 45;
    v_gain = 0.85f;
    cap_gain = 1.f;
  }
}

extern uint16_t cms_offline_counter;
void CMS__() {
  if (CMS_Data.cms_cap_v < 12) {
    CMS_Data.charge_flag = 0;
  } else if (CMS_Data.cms_cap_v > 18 && (Chassis.CapKey)) {
    CMS_Data.charge_flag = 1;
  } else if (CMS_Data.cms_cap_v > 15 && (!Chassis.CapKey)) {
    CMS_Data.charge_flag = 2;
  } else if (CMS_Data.charge_flag == 1 && CMS_Data.cms_cap_v > 12 && (!Chassis.CapKey)) {
    CMS_Data.charge_flag = 2;
  }
  if ((Chassis.CapKey) && CMS_Data.cms_cap_v > 12 && CMS_Data.charge_flag == 1) {
    CMS_Data.Mode = FLY;
  } else if ((!Chassis.CapKey) && CMS_Data.cms_cap_v > 12 && CMS_Data.charge_flag == 2) {
    CMS_Data.Mode = HIGH_SPEED;
  } else {
    CMS_Data.Mode = NORMAL;
  }
  if (power_heat_data_t.buffer_energy < 20 || cms_offline_counter > 200) {
    CMS_Data.Mode = NORMAL;
  }
  if (power_heat_data_t.buffer_energy > 70) {
    CMS_Data.Mode = FLY;
  }
  cms_offline_counter++;
}
uint8_t chassis_limit_update_flag = 0;
void chassis_limit_update(void) {
  if (Referee.chassis_power_limit != Power_Max) {
    chassis_limit_update_flag = 1;
  } else {
    chassis_limit_update_flag = 0;
  }
}

uint8_t cms_flag = 0;
float Plimit = 0;
uint8_t chassis_powerloop(Chassis_t *Chassis) {
  he = fabs((float)Chassis->Motor3508[0].speed_rpm) * fabs((float)Chassis->Current[4]) * kp +
       fabs((float)Chassis->Motor3508[1].speed_rpm) * fabs((float)Chassis->Current[5]) * kp +
       fabs((float)Chassis->Motor3508[2].speed_rpm) * fabs((float)Chassis->Current[6]) * kp +
       fabs((float)Chassis->Motor3508[3].speed_rpm) * fabs((float)Chassis->Current[7]) * kp +
       fabs((float)Chassis->Motor6020[0].speed_rpm) * fabs((float)Chassis->Current[0]) * kp +
       fabs((float)Chassis->Motor6020[1].speed_rpm) * fabs((float)Chassis->Current[1]) * kp +
       fabs((float)Chassis->Motor6020[2].speed_rpm) * fabs((float)Chassis->Current[2]) * kp +
       fabs((float)Chassis->Motor6020[3].speed_rpm) * fabs((float)Chassis->Current[3]) * kp;

  lijupower = he + START_POWER;

  if (CMS_Data.cms_cap_v <= 15 || cms_offline_counter > 500 || power_heat_data_t.buffer_energy < 20) {
    power_flag = 0;
    cms_flag = 0;
  } else {
    power_flag = 1;
  }
  if (CMS_Data.Mode == FLY) {
    Power_Max = 200;
    cms_flag = 1;
    if (power_heat_data_t.buffer_energy < 20)
      power_flag = 0;
    else {
      power_flag = 1;
    }

  } else if (CMS_Data.Mode == HIGH_SPEED) {
    Power_Max += 50;
  }
  if (power_flag == 0) {
    if (power_heat_data_t.buffer_energy < 20 && power_heat_data_t.buffer_energy >= 18) {
      Plimit = 0.6;
      // power_scale = (Power_Max-2) / lijupower;
    } else if (power_heat_data_t.buffer_energy < 18 && power_heat_data_t.buffer_energy >= 15) {
      Plimit = 0.3;
      // power_scale = (Power_Max-2) / lijupower;
    } else if (power_heat_data_t.buffer_energy < 15 && power_heat_data_t.buffer_energy >= 10 && cms_flag == 0) {
      Plimit = 0.1;
      // power_scale = (Power_Max-2) / lijupower;
    } else if (power_heat_data_t.buffer_energy < 10 && power_heat_data_t.buffer_energy >= 0) {
      Plimit = 0.05;
      // power_scale = (Power_Max-2) / lijupower;}
    } else {
      Plimit = 1;
      // power_scale = 1;
    }
  }
  if (lijupower > Power_Max && power_flag == 0) {
    power_scale = (Power_Max - 2) / lijupower;
    Chassis->Current[0] *= (power_scale) * (Plimit);
    Chassis->Current[1] *= (power_scale) * (Plimit);
    Chassis->Current[2] *= (power_scale) * (Plimit);
    Chassis->Current[3] *= (power_scale) * (Plimit);
    Chassis->Current[4] *= (power_scale) * (Plimit);
    Chassis->Current[5] *= (power_scale) * (Plimit);
    Chassis->Current[6] *= (power_scale) * (Plimit);
    Chassis->Current[7] *= (power_scale) * (Plimit);
  }

  return 0;
}
