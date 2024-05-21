#ifndef _INFANTRY4_KEY_MAP_
#define _INFANTRY4_KEY_MAP_

#include "Remote.h"
#define YAW_REMOTE_SENS                         0.25f
#define PITCH_REMOTE_SENS                       0.25f
#define YAW_MOUSE_SENS                          10
#define PITCH_MOUSE_SENS                        30

// ״̬������
// ��̨����
//#define GIMBAL_WEAK_KEYMAP              SwitchRightDownSide()
// ��̨ʹ��
#define GIMBAL_ENABLE_KEYMAP            SwitchRightMidSide() || SwitchRightUpSide()
// �������ʹ��
#define SHOOTER_ENABLE_KEYMAP           SwitchRightUpSide()
// ����ʹ��
#define CHASSIS_ENABLE_KEYMAP           SwitchLeftDownSide()

// ��̨�˶�����ָ��
// YAW
#define GIMBAL_CMD_YAW_KEYMAP           NormalizedLimit(MouseMoveY()*YAW_MOUSE_SENS + RemoteChannalLeftY()*YAW_REMOTE_SENS)
// PITCH
#define GIMBAL_CMD_PITCH_KEYMAP         -NormalizedLimit(MouseMoveX()*PITCH_MOUSE_SENS + RemoteChannalLeftX()*PITCH_REMOTE_SENS)

// �����˶�����ָ��
// ǰ��
#define CHASSIS_CMD_X_KEYMAP \
  NormalizedLimit((RemoteChannalRightX() - CheakKeyPress(KEY_PRESSED_OFFSET_S) + CheakKeyPress(KEY_PRESSED_OFFSET_W)))
// ����
#define CHASSIS_CMD_Y_KEYMAP \
  NormalizedLimit((RemoteChannalRightY() - CheakKeyPress(KEY_PRESSED_OFFSET_D) + CheakKeyPress(KEY_PRESSED_OFFSET_A)))
// ����
#define CHASSIS_HIGH_SPEED_KEYMAP CheakKeyPress(KEY_PRESSED_OFFSET_R)
// ����
#define CHASSIS_STOP_KEYMAP CheakKeyPressOnce(KEY_PRESSED_OFFSET_F)
// ����С����
#define CHASSIS_HIGH_SPEED_ROTATE CheakKeyPress(KEY_PRESSED_OFFSET_CTRL)

// �������ݿ���
#define SUPER_CAP_SWITCH_KEYMAP         CheakKeyPress(KEY_PRESSED_OFFSET_C)  
// С����
#define CHASSIS_ROTATE_SWITCH_KEYMAP    CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT)  || (RemoteDial() < -0.99f) || CheakKeyPress(KEY_PRESSED_OFFSET_CTRL)
#define CHASSIS_ROTATE_CCW_KEYMAP       (RemoteDial() > 0.99f)

//�򿪵��ո�
#define COVER_SWITCH_KEYMAP             CheakKeyPressOnce(KEY_PRESSED_OFFSET_R)


// ����pitch����
#define AIMBOT_PITCH_BIAS_LOW_KEYMAP	(CheakKeyPress(KEY_PRESSED_OFFSET_CTRL) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_C))
#define AIMBOT_PITCH_BIAS_HIGH_KEYMAP	(CheakKeyPress(KEY_PRESSED_OFFSET_CTRL) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_V))
#define AIMBOT_PITCH_BIAS_ZERO_KEYMAP	(CheakKeyPress(KEY_PRESSED_OFFSET_CTRL) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_Z))




    
// ����ָ��
#define SHOOT_COMMAND_KEYMAP            ((RemoteDial() > 0.99f) || (MousePressLeft()))
// ����ָ��
#define AIMBOT_COMMAND_KEYMAP           (SwitchLeftUpSide() || MousePressRight())
// ���ģʽ���ֶ�/�Զ�������
#define FIRE_MODE_KEYMAP                CheakKeyPressOnce(KEY_PRESSED_OFFSET_Q)
// ���������������󣨴��������ģʽ��
#define SINGLE_SHOOT_KEMAP              CheakKeyPressOnce(KEY_PRESSED_OFFSET_E)

#define BIG_RUNE_KEYMAP                 CheakKeyPress(KEY_PRESSED_OFFSET_G)

#define SMALL_RUNE_KEYMAP				CheakKeyPress(KEY_PRESSED_OFFSET_B)

#define SHOOT_SPEED_INCREASE_KEYMAP     CheakKeyPressOnce(KEY_PRESSED_OFFSET_X)
#define SHOOT_SPEED_DECREASE_KEYMAP     CheakKeyPressOnce(KEY_PRESSED_OFFSET_V)

// ����pitch����
#define AIMBOT_PITCH_BIAS_LOW_KEYMAP (CheakKeyPress(KEY_PRESSED_OFFSET_CTRL) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_C))
#define AIMBOT_PITCH_BIAS_HIGH_KEYMAP \
  (CheakKeyPress(KEY_PRESSED_OFFSET_CTRL) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_V))
#define AIMBOT_PITCH_BIAS_ZERO_KEYMAP \
  (CheakKeyPress(KEY_PRESSED_OFFSET_CTRL) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_Z))




#endif
