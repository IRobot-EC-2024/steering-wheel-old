/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "Remote.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"
#include "stm32f4xx_hal_uart.h"

extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;

// �����ж�
void USART3_IRQHandler(void) {
  if (huart3.Instance->SR & UART_FLAG_RXNE)  // ���յ�����
  {
    __HAL_UART_CLEAR_PEFLAG(&huart3);
  } else if (USART3->SR & UART_FLAG_IDLE) {
    static uint16_t this_time_rx_len = 0;

    __HAL_UART_CLEAR_PEFLAG(&huart3);

    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
      /* Current memory buffer used is Memory 0 */

      // disable DMA
      // ʧЧDMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);

      // get receive data length, length = set_data_length - remain_length
      // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

      // reset set_data_lenght
      // �����趨���ݳ���
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

      // set memory buffer 1
      // �趨������1
      hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

      // enable DMA
      // ʹ��DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        // ����ң��������
        sbus_to_rc(0);
        // ��¼���ݽ���ʱ��
      }
    } else {
      /* Current memory buffer used is Memory 1 */
      // disable DMA
      // ʧЧDMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);

      // get receive data length, length = set_data_length - remain_length
      // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

      // reset set_data_lenght
      // �����趨���ݳ���
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

      // set memory buffer 0
      // �趨������0
      DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

      // enable DMA
      // ʹ��DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        // ����ң��������
        sbus_to_rc(1);
        // ��¼���ݽ���ʱ��
      }
    }
  }
}

// ң����������������
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

// ȡ������
static int16_t RC_abs(int16_t value);
/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          ң����Э�����
 * @param[in]      sbus_buf: ԭ������ָ��
 * @param[out]     rc_ctrl: ң��������ָ
 * @retval         none
 */
void sbus_to_rc(uint8_t DmaBufNmb);

// remote control data
// ң�������Ʊ���
RC_ctrl_t rc_ctrl;
// ����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

// ���ش������
uint16_t KeyFormerChannal = 0;
uint16_t KeyJumpChannal = 0;
uint16_t KeyUsed = 0;

/**
 * @brief          remote control init
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          ң������ʼ��
 * @param[in]      none
 * @retval         none
 */
void remote_control_init(void) { usart3_rx_dma_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM); }
/**
 * @brief          get remote control data point
 * @param[in]      none
 * @retval         remote control data point
 */
/**
 * @brief          ��ȡң��������ָ��
 * @param[in]      none
 * @retval         ң��������ָ��
 */
const RC_ctrl_t *get_remote_control_point(void) { return &rc_ctrl; }

// �ж�ң���������Ƿ����
uint8_t RC_data_is_error(void) {
  // ��ֹʹ��go to��䣡����������
  if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    return 1;
  }
  if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    return 1;
  }
  if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    return 1;
  }
  if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    return 1;
  }
  if (rc_ctrl.rc.s[0] == 0) {
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    return 1;
  }
  if (rc_ctrl.rc.s[1] == 0) {
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    return 1;
  }
  return 0;
}

void slove_RC_lost(void) { usart3_rx_dma_restart(SBUS_RX_BUF_NUM); }
void slove_data_error(void) { usart3_rx_dma_restart(SBUS_RX_BUF_NUM); }

// ȡ������
static int16_t RC_abs(int16_t value) {
  if (value > 0) {
    return value;
  } else {
    return -value;
  }
}
/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          ң����Э�����
 * @param[in]      sbus_buf: ԭ������ָ��
 * @param[out]     rc_ctrl: ң��������ָ
 * @retval         none
 */
void sbus_to_rc(uint8_t DmaBufNmb) {
  KeyFormerChannal = rc_ctrl.key.v;

  rc_ctrl.rc.ch[0] = (sbus_rx_buf[DmaBufNmb][0] | (sbus_rx_buf[DmaBufNmb][1] << 8)) & 0x07ff;         //!< Channel 0
  rc_ctrl.rc.ch[1] = ((sbus_rx_buf[DmaBufNmb][1] >> 3) | (sbus_rx_buf[DmaBufNmb][2] << 5)) & 0x07ff;  //!< Channel 1
  rc_ctrl.rc.ch[2] = ((sbus_rx_buf[DmaBufNmb][2] >> 6) | (sbus_rx_buf[DmaBufNmb][3] << 2) |           //!< Channel 2
                      (sbus_rx_buf[DmaBufNmb][4] << 10)) &
                     0x07ff;
  rc_ctrl.rc.ch[3] = ((sbus_rx_buf[DmaBufNmb][4] >> 1) | (sbus_rx_buf[DmaBufNmb][5] << 7)) & 0x07ff;  //!< Channel 3
  rc_ctrl.rc.s[0] = ((sbus_rx_buf[DmaBufNmb][5] >> 4) & 0x0003);                                      //!< Switch left
  rc_ctrl.rc.s[1] = ((sbus_rx_buf[DmaBufNmb][5] >> 4) & 0x000C) >> 2;                                 //!< Switch right
  rc_ctrl.mouse.y = -(sbus_rx_buf[DmaBufNmb][6] | (sbus_rx_buf[DmaBufNmb][7] << 8));                  //!< Mouse X axis
  rc_ctrl.mouse.x = -(sbus_rx_buf[DmaBufNmb][8] | (sbus_rx_buf[DmaBufNmb][9] << 8));                  //!< Mouse Y axis
  rc_ctrl.mouse.z = sbus_rx_buf[DmaBufNmb][10] | (sbus_rx_buf[DmaBufNmb][11] << 8);                   //!< Mouse Z axis
  rc_ctrl.mouse.press_l = sbus_rx_buf[DmaBufNmb][12];                                 //!< Mouse Left Is Press ?
  rc_ctrl.mouse.press_r = sbus_rx_buf[DmaBufNmb][13];                                 //!< Mouse Right Is Press ?
  rc_ctrl.key.v = sbus_rx_buf[DmaBufNmb][14] | (sbus_rx_buf[DmaBufNmb][15] << 8);     //!< KeyBoard value
  rc_ctrl.rc.ch[4] = sbus_rx_buf[DmaBufNmb][16] | (sbus_rx_buf[DmaBufNmb][17] << 8);  // NULL

  rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
  rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
  rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
  rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
  rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;

  KeyJumpChannal = (rc_ctrl.key.v ^ KeyFormerChannal);
}

// ����Ӧ�ù���
// ���������Ϊ���������Ĺ������͵㰴�����Ĺ�����
// ���� W��A��S��D��SHIFT��CTRL��F Ϊ���������Ĺ�����
// ��������Ϊ�㰴������

// ������������ļ�ֵ���

bool_t CheakKeyPress(uint16_t Key) {
  if ((rc_ctrl.key.v & Key) == 0) return 0;

  return 1;
}

bool_t CheakKeyPressOnce(uint16_t Key) {
  if ((rc_ctrl.key.v & Key) == 0) {
    KeyUsed &= (~Key);
    return 0;
  }

  if ((KeyJumpChannal & Key) == 0) {
    return 0;
  } else {
    if ((KeyUsed & Key) == 0) {
      KeyUsed |= Key;
      return 1;
    }
    return 0;
  }
}

// ��һ��ҡ��ֵ
fp32 RemoteChannalRightX() { return (rc_ctrl.rc.ch[1] / 660.0f); }
fp32 RemoteChannalRightY() { return (-rc_ctrl.rc.ch[0] / 660.0f); }
fp32 RemoteChannalLeftX() { return (rc_ctrl.rc.ch[3] / 660.0f); }
fp32 RemoteChannalLeftY() { return (-rc_ctrl.rc.ch[2] / 660.0f); }
fp32 RemoteDial() { return (rc_ctrl.rc.ch[4] / 660.0f); }

// ��һ������ƶ�
fp32 MouseMoveX() { return (rc_ctrl.mouse.x / 32768.0f); }
fp32 MouseMoveY() { return (rc_ctrl.mouse.y / 32768.0f); }

// ������Ҽ�
bool_t MousePressLeft() { return rc_ctrl.mouse.press_l; }
bool_t MousePressRight() { return rc_ctrl.mouse.press_r; }

// ����λ�ü��
bool_t SwitchRightUpSide() { return (rc_ctrl.rc.s[0] == RC_SW_UP); }
bool_t SwitchRightMidSide() { return (rc_ctrl.rc.s[0] == RC_SW_MID); }
bool_t SwitchRightDownSide() { return (rc_ctrl.rc.s[0] == RC_SW_DOWN); }
bool_t SwitchLeftUpSide() { return (rc_ctrl.rc.s[1] == RC_SW_UP); }
bool_t SwitchLeftMidSide() { return (rc_ctrl.rc.s[1] == RC_SW_MID); }
bool_t SwitchLeftDownSide() { return (rc_ctrl.rc.s[1] == RC_SW_DOWN); }

fp32 NormalizedLimit(fp32 input) {
  if (input > 1.0f) {
    input = 1.0f;
  } else if (input < -1.0f) {
    input = -1.0f;
  }
  return input;
}
