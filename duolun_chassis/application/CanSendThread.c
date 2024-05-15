/*
 * ���������������̨������Ϣ
 * ��������������ȼ�����ʵʱ�Ե�
 * ���⣺Ŀǰ������Ϣ�����ж���ᵼ��FIFO���������Ӷ�����ʧ��
 * ���˼·����������FIFO���������ֱ��Ӧ�ߵ����ȼ��������ȼ����������������ȼ�FIFO��
 *			ÿ�����ȷ���ֱ�������ȼ���FIFO��ֱ���䷢����ϲŷ��͵����ȼ�FIFO
 * ʵ�ֲ��֣�������FIFO���ִ���ʱ��ʵ�����ƣ����ǲ��ܼ���Ƿ�ɹ�����
 * �޸ģ�1.��������ͬ������Ҫע����ǣ�һ��������FIFO���������Լ�һ������ϵͳ���ݻ�������
 *			Ŀǰ˼·Ϊ����FIFO��һֱ�ڽ����ģ���������������ʱ������FIFO��գ�ͬʱ�����ݴ��뷢��FIFO��
 *			�Լ�����ϵͳ���ݻ���������ʱ����������𣬷����������С�����FIFO���ʱ������������𣬽����������С�
 *			�м����FIFOʼ�������У��ɶ�����������á�
 */
#include "cmsis_os.h"
#include "CanSendThread.h"
#include "tim.h"
#include "can.h"
#include "RefereeBehaviour.h"

uint8_t DataBuffer[11];
extern osThreadId CanSendHandle;
extern osThreadId RefereeHandle;

static CAN_TxHeaderTypeDef can_tx_message;

static void Can_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t lenth, uint8_t *buffer);
void CanSendTask(void const *argument) {
  uint8_t SendLenth;
  uint32_t SendId;
  while (1) {
    if (Pop(SendBuffer, (SendBuffer + 1), DataBuffer) == 1) {
      SendId = (uint32_t)(DataBuffer[0] << 8 | DataBuffer[1]);

      SendLenth = DataBuffer[2];

      Can_Send(&hcan1, SendId, SendLenth, &DataBuffer[3]);
    }

    osDelay(1);
  }
}

static void Can_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t lenth, uint8_t *buffer) {
  uint32_t send_mail_box;
  can_tx_message.StdId = id;
  can_tx_message.IDE = CAN_ID_STD;
  can_tx_message.RTR = CAN_RTR_DATA;
  can_tx_message.DLC = lenth;

  HAL_CAN_AddTxMessage(hcan, &can_tx_message, buffer, &send_mail_box);
}

uint32_t CMSCounter = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  } else if (htim->Instance == TIM3) {
    CMSCounter++;
  }
}