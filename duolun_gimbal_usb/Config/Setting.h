#ifndef SETTING_H
#define SETTING_H

#define MOTOR_OFFLINE_TIMEMAX 50
#define REMOTE_OFFLINE_TIMEMAX 550
#define AIMBOT_OFFLINE_TIMEMAX 550
#define REFEREE_OFFLINE_TIMEMAX 3000

// #define HERO1
// #define HERO1_0
// #define HERO1_1
// #define HERO1_2
#define INFANTRY3
// #define INFANTRY4
// #define INFANTRY5
// #define SENTRY7

#ifdef INFANTRY4

// ���������ļ�
#define PARAMETER_FILE "Infantry4Parameter.h"
// ��λ�����ļ�
#define KEYMAP_FILE "Infantry4KeyMap.h"
// imu��װ����
#define IMU_DIRECTION_zryx_XYZ
// gyro yaw��ƫ��
#define GYRO_YAW_BIAS -0.0055f
// �������������
#define MAIN_SHOOTER_TYPE_NORMAL
// #define MAIN_SHOOTER_TYPE_HEAVY
//  ���ID����
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define ROTOR_MOTOR_ID 0x203
#define AMMO_LEFT_MOTOR_ID 0x202
#define AMMO_RIGHT_MOTOR_ID 0x201
// �����װ����
// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#define YAW_MOTOR_DIRECTION 1
#define PITCH_MOTOR_DIRECTION 1
#define ROTOR_MOTOR_DIRECTION -1
#define AMMO_LEFT_MOTOR_DIRECTION -1
#define AMMO_RIGHT_MOTOR_DIRECTION 1
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE -144.102539f
#define PITCH_MIN_ANGLE -31.0f
#define PITCH_MAX_ANGLE 18.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID AMMO_LEFT_SPEED_15MS
#define DEFAULT_AMMOR_PID AMMO_RIGHT_SPEED_15MS
#define DEFAULT_AMMO_SPEEDSET AMMO_SPEEDSET_15MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT hcan1
#endif

#ifdef INFANTRY3

// ���������ļ�
#define PARAMETER_FILE "Infantry3Parameter.h"
// ��λ�����ļ�
#define KEYMAP_FILE "Infantry4KeyMap.h"
// imu��װ����
#define IMU_DIRECTION_rzyx_XYZ
// gyro yaw��ƫ��
#define GYRO_YAW_BIAS -0.0019f  //-0.00770,-0.01065f,INCREASE
// �������������
#define MAIN_SHOOTER_TYPE_NORMAL
// #define MAIN_SHOOTER_TYPE_HEAVY
//  ���ID����
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define ROTOR_MOTOR_ID 0x203
#define AMMO_LEFT_MOTOR_ID 0x201
#define AMMO_RIGHT_MOTOR_ID 0x202
// �����װ����
// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#define YAW_MOTOR_DIRECTION 1
#define PITCH_MOTOR_DIRECTION -1
#define ROTOR_MOTOR_DIRECTION 1
#define AMMO_LEFT_MOTOR_DIRECTION 1
#define AMMO_RIGHT_MOTOR_DIRECTION -1
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE 180.0f - 142.5f  // 21.4f
#define PITCH_MIN_ANGLE -20.0f             //-27.0f//-24.0f//-26.0f
#define PITCH_MAX_ANGLE 15.0f              // 15.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID AMMO_LEFT_SPEED_30MS
#define DEFAULT_AMMOR_PID AMMO_RIGHT_SPEED_30MS
#define DEFAULT_AMMO_SPEEDSET AMMO_SPEEDSET_30MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT hcan1
#endif

#ifdef SENTRY7
// ���������ļ�
#define PARAMETER_FILE "Sentry7Parameter.h"
// ��λ�����ļ�
#define KEYMAP_FILE "Sensor7KeyMap.h"
// imu��װ����
#define IMU_DIRECTION_yrxz_XYZ
// ���ID����
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define ROTOR_MOTOR_ID 0x203
#define AMMO_LEFT_MOTOR_ID 0x201
#define AMMO_RIGHT_MOTOR_ID 0x202
// �����װ����
// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#define YAW_MOTOR_DIRECTION -1
#define PITCH_MOTOR_DIRECTION -1
#define ROTOR_MOTOR_DIRECTION 1
#define AMMO_LEFT_MOTOR_DIRECTION -1
#define AMMO_RIGHT_MOTOR_DIRECTION 1
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE 0.0f
#define PITCH_MIN_ANGLE -8.8f
#define PITCH_MAX_ANGLE 28.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID AMMO_LEFT_SPEED_15MS
#define DEFAULT_AMMOR_PID AMMO_RIGHT_SPEED_15MS
#define DEFAULT_AMMO_SPEEDSET AMMO_SPEEDSET_15MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT hcan2
#endif

#ifdef HERO1_0
// ���������ļ�
#define PARAMETER_FILE "Hero1_0Parameter.h"
// ��λ�����ļ�
#define KEYMAP_FILE "Hero1_0KeyMap.h"
// imu��װ����
#define IMU_DIRECTION_xyz_XYZ
// ��װpitch�����������
#define PITCH_AUX
// ��װ�����������
#define SHOOTOR_AUX
// ���ID����
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define ROTOR_MOTOR_ID 0x208
// #define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID 0x201
#define AMMO_RIGHT_MOTOR_ID 0x202
// pitch�����������
// �˵����pitch���������װ�����෴��ִ��������ͬ������ע�������
#define PITCH_AUX_MOTOR_ID 0x207
// ��������������̵��
// �˵����pitch���������װ�����෴��ִ��������ͬ������ע�������
// #define ROTOR_AUX_MOTOR_ID          0x203
// �����װ����
// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#define YAW_MOTOR_DIRECTION 1
#define PITCH_MOTOR_DIRECTION 1
#define ROTOR_MOTOR_DIRECTION -1
#define AMMO_LEFT_MOTOR_DIRECTION 1
#define AMMO_RIGHT_MOTOR_DIRECTION -1
// ��������������̵��
// #define ROTOR_AUX_MOTOR_DIRECTION   -1
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE 4.65820313f
#define PITCH_MIN_ANGLE -35.0f
#define PITCH_MAX_ANGLE 18.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID AMMO_LEFT_SPEED_10MS
#define DEFAULT_AMMOR_PID AMMO_RIGHT_SPEED_10MS
#define DEFAULT_AMMO_SPEEDSET AMMO_SPEEDSET_10MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT hcan1

#endif

#ifdef HERO1_1
// ���������ļ�
#define PARAMETER_FILE "Hero1_1Parameter.h"
// ��λ�����ļ�
#define KEYMAP_FILE "Hero1_1KeyMap.h"
// imu��װ����
#define IMU_DIRECTION_xzry_XYZ
// ���ID����
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define ROTOR_MOTOR_ID 0x207
#define AMMO_LEFT_MOTOR_ID 0x201
#define AMMO_RIGHT_MOTOR_ID 0x202
// �����װ����
// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#define YAW_MOTOR_DIRECTION 1
#define PITCH_MOTOR_DIRECTION -1
#define ROTOR_MOTOR_DIRECTION -1
#define AMMO_LEFT_MOTOR_DIRECTION 1
#define AMMO_RIGHT_MOTOR_DIRECTION -1
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE -114.873047f
#define PITCH_MIN_ANGLE -36.0f
#define PITCH_MAX_ANGLE 26.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID AMMO_LEFT_SPEED_10MS
#define DEFAULT_AMMOR_PID AMMO_RIGHT_SPEED_10MS
#define DEFAULT_AMMO_SPEEDSET AMMO_SPEEDSET_10MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT hcan2

#endif

// #ifdef INFANTRY5

//// ���������ļ�
// #define PARAMETER_FILE "Infantry4Parameter.h"
//// ��λ�����ļ�
// #define KEYMAP_FILE "Infantry4KeyMap.h"
//// imu��װ����
// #define IMU_DIRECTION_xyz_XYZ
//// ���ID����
// #define YAW_MOTOR_ID                0x205
// #define PITCH_MOTOR_ID              0x206
// #define ROTOR_MOTOR_ID              0x203
// #define AMMO_LEFT_MOTOR_ID          0x201
// #define AMMO_RIGHT_MOTOR_ID         0x202
//// �����װ����
//// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
//// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
//// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
// #define YAW_MOTOR_DIRECTION         1
// #define PITCH_MOTOR_DIRECTION       -1
// #define ROTOR_MOTOR_DIRECTION       1
// #define AMMO_LEFT_MOTOR_DIRECTION   -1
// #define AMMO_RIGHT_MOTOR_DIRECTION  1
//// ��̨YAW�����͸����޷�
// #define YAW_ZERO_ECDANGLE           2.72460938f
// #define PITCH_MIN_ANGLE             -20.0f
// #define PITCH_MAX_ANGLE             20.0f
//// Ĭ��Ħ�����ٶ�
// #define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_15MS
// #define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_15MS
// #define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_15MS
//// ͨ��can����λ��
// #define COMMUNICATE_CANPORT         hcan2
// #endif

#ifdef INFANTRY5

// 2?��y???????t
#define PARAMETER_FILE "Infantry5Parameter.h"
// ?��?????????t
#define KEYMAP_FILE "Infantry4KeyMap.h"
// imu��2���㡤??��
#define IMU_DIRECTION_zryx_XYZ
// gyro yaw?��????
#define GYRO_YAW_BIAS -0.0025f  // 055f
// ?�¡��騦??��11����D��
#define MAIN_SHOOTER_TYPE_NORMAL
// #define MAIN_SHOOTER_TYPE_HEAVY
//  ��??��ID��???
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define ROTOR_MOTOR_ID 0x203
#define AMMO_LEFT_MOTOR_ID 0x201
#define AMMO_RIGHT_MOTOR_ID 0x202
// ��??����2���㡤??��
// ??������??��?y?��???����??��o��??������?��?�ꡧ????���?����?�̨�??��?a1��?����?��?a-1
// 2|?����??��?y?��???����??��o���̡�����??��??11����??����??��?a1��?����?��?a-1
// ?|2��??��??��?y?��???����??��o���̡��̨���??��?a1��?����?��?a-1
#define YAW_MOTOR_DIRECTION 1
#define PITCH_MOTOR_DIRECTION 1
#define ROTOR_MOTOR_DIRECTION -1
#define AMMO_LEFT_MOTOR_DIRECTION -1
#define AMMO_RIGHT_MOTOR_DIRECTION 1
// ??����YAW?����?��?o��????T����
#define YAW_ZERO_ECDANGLE 19.0f  //-144.102539f
#define PITCH_MIN_ANGLE -19.5f   //-31.0f
#define PITCH_MAX_ANGLE 25.0f    // 15.0f//18.0f
// ??��??|2��???��?��
#define DEFAULT_AMMOL_PID AMMO_LEFT_SPEED_15MS
#define DEFAULT_AMMOR_PID AMMO_RIGHT_SPEED_15MS
#define DEFAULT_AMMO_SPEEDSET AMMO_SPEEDSET_15MS
// ����D?can����??????
#define COMMUNICATE_CANPORT hcan1
#endif

#ifdef HERO1_2
// ���������ļ�
#define PARAMETER_FILE "Hero1_1Parameter.h"
// imu��װ����
#define IMU_DIRECTION_xyz_XYZ
// ���ID����
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206
#define ROTOR_MOTOR_ID 0x207
#define AMMO_LEFT_MOTOR_ID 0x201
#define AMMO_RIGHT_MOTOR_ID 0x202
// �����װ����
// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#define YAW_MOTOR_DIRECTION 1
#define PITCH_MOTOR_DIRECTION -1
#define ROTOR_MOTOR_DIRECTION -1
#define AMMO_LEFT_MOTOR_DIRECTION 1
#define AMMO_RIGHT_MOTOR_DIRECTION -1
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE -164.223633f
#define PITCH_MIN_ANGLE -36.0f
#define PITCH_MAX_ANGLE 26.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID AMMO_LEFT_SPEED_10MS
#define DEFAULT_AMMOR_PID AMMO_RIGHT_SPEED_10MS
#define DEFAULT_AMMO_SPEEDSET AMMO_SPEEDSET_10MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT hcan2

#endif

// #define IMU_DIRECTION_xyz_XYZ
// #define IMU_DIRECTION_yrxz_XYZ
// #define IMU_DIRECTION_rxryz_XYZ
// #define IMU_DIRECTION_ryxz_XYZ

// #define IMU_DIRECTION_zryx_XYZ
// #define IMU_DIRECTION_yzx_XYZ
// #define IMU_DIRECTION_rzyx_XYZ
// #define IMU_DIRECTION_ryrzx_XYZ

#endif
