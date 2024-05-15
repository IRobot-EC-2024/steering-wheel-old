#include "user_lib.h"
#include "arm_math.h"
// #include "math.h"

/**
 * @brief          б��������ʼ��
 * @author         RM
 * @param[in]      б�������ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @param[in]      ���ֵ
 * @param[in]      ��Сֵ
 * @retval         ���ؿ�
 */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min) {
  ramp_source_type->frame_period = frame_period;
  ramp_source_type->max_value = max;
  ramp_source_type->min_value = min;
  ramp_source_type->input = 0.0f;
  ramp_source_type->out = 0.0f;
}

/**
 * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
 * @author         RM
 * @param[in]      б�������ṹ��
 * @param[in]      ����ֵ
 * @param[in]      �˲�����
 * @retval         ���ؿ�
 */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input) {
  ramp_source_type->input = input;
  ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
  if (ramp_source_type->out > ramp_source_type->max_value) {
    ramp_source_type->out = ramp_source_type->max_value;
  } else if (ramp_source_type->out < ramp_source_type->min_value) {
    ramp_source_type->out = ramp_source_type->min_value;
  }
}
/**
 * @brief          һ�׵�ͨ�˲���ʼ��
 * @author         RM
 * @param[in]      һ�׵�ͨ�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @param[in]      �˲�����
 * @retval         ���ؿ�
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num) {
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num = num;
  first_order_filter_type->input = 0.0f;
  first_order_filter_type->out = 0.0f;
}

/**
 * @brief          һ�׵�ͨ�˲�����
 * @author         RM
 * @param[in]      һ�׵�ͨ�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @retval         ���ؿ�
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input) {
  first_order_filter_type->input = input;
  first_order_filter_type->out =
      first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) *
          first_order_filter_type->out +
      first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) *
          first_order_filter_type->input;
}

// ��������
void abs_limit(fp32 *num, fp32 Limit) {
  if (*num > Limit) {
    *num = Limit;
  } else if (*num < -Limit) {
    *num = -Limit;
  }
}

// �жϷ���λ
fp32 sign(fp32 value) {
  if (value >= 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}

// ��������
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0.0f;
  }
  return Value;
}

// int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0;
  }
  return Value;
}

// �޷�����
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

// �޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

// ѭ���޷�����
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue) {
  if (maxValue < minValue) {
    return Input;
  }

  if (Input > maxValue) {
    fp32 len = maxValue - minValue;
    while (Input > maxValue) {
      Input -= len;
    }
  } else if (Input < minValue) {
    fp32 len = maxValue - minValue;
    while (Input < minValue) {
      Input += len;
    }
  }
  return Input;
}

// ���ȸ�ʽ��Ϊ-PI~PI

// �Ƕȸ�ʽ��Ϊ-180~180
fp32 theta_format(fp32 Ang) { return loop_fp32_constrain(Ang, -180.0f, 180.0f); }

fp32 fast_atan2f(fp32 x, fp32 y) {
  fp32 a = fminf(fabsf(x), fabsf(y)) / fmaxf(fabsf(x), fabsf(y));
  fp32 s = a * a;
  fp32 r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  if (fabsf(y) > fabsf(x)) {
    r = 1.57079637f - r;
  }
  if (x < 0) {
    r = 3.14159274f - r;
  }
  if (y < 0) {
    r = -r;
  }
  return r;
}
/**һά�������˲���  ��H����Ϊ 1
 *�������˲���
 *@param KFP *kfp �������ṹ�����
 *   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
 */
float KalmanFilter(KFP *kfp, float Zk, float current_1, float current_2) {
  // ����ļ��㣺A*��һʱ��Ԥ��ֵ + B*ϵͳ����ֵ
  kfp->X_k = kfp->A * kfp->out + kfp->B[0] * current_1 + kfp->B[1] * current_2;
  // Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� +
  // ��������Э����
  kfp->Now_P = kfp->A * kfp->LastP + kfp->Q;
  // ���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� +
  // �۲�����Э���
  kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
  // ��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
  kfp->out = kfp->X_k + kfp->Kg * (Zk - kfp->out);  // ��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
  // ����Э�����: ���ε�ϵͳЭ����� kfp->LastP
  // ����һ������׼����
  kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
  return kfp->out;
}
void KalmanFilter_init(KFP *kfp, float A, float B_1, float B_2, float LastP, float Q, float R) {
  kfp->A = A;
  kfp->B[0] = B_1;
  kfp->B[1] = B_2;
  kfp->LastP = LastP;
  kfp->Q = Q;
  kfp->R = R;
  kfp->X_k = 0;
  kfp->out = 0;
  kfp->Now_P = 0;
  kfp->Kg = 0;
}

// Function to calculate absolute value
float Fabs(float a) {
  // If a is negative, return its negation
  if (a < 0) return -a;
  // Otherwise, return a
  return a;
}
fp32 Calc(double x) {
  if (x > 0)
    return sqrt(2.0 * x) / 2.0;
  else
    return -sqrt(-2.0 * x) / 2.0;
}
fp32 sig(fp32 x) {
  if (x > 0) return 1;
  return -1;
}