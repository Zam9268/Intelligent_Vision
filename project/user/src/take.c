#include "take.h"

uint16 servo1_duty = 50;
uint16 servo2_duty = 50;//�����ʼֵ

uint8 arm_pick_flag = ARM_PICK_DONE; //һ��ʼ��ԭ�������Ĭ����ʰȡ�����ȥ��һ�����
uint8 arm_state_flag = ARM_STATE_OFF;

uint8 one_pick = 0;
uint8 arm_put_down = 0;//��е�۷�����Ʒ�ı�־λ

void my_pwm_gpio(void)
{
  pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(35));
  pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(50)); //��ʼ���͹��е�λ�����
  pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(79)); //��ʼ����̨���12  73  133

  gpio_init(C9, GPO, 0, GPO_PUSH_PULL);                                       //�����                              //60Ϊ��߷�ͼƬ��88�ұ߷�ͼƬ

  gpio_init(B14, GPO, 0, GPO_PUSH_PULL); //�����
  gpio_init(B16, GPO, 0, GPO_PUSH_PULL); //�����
  gpio_init(B17, GPO, 0, GPO_PUSH_PULL); //�����

  gpio_init(C12, GPI, 1, GPI_PULL_UP); //����
  gpio_init(C13, GPI, 1, GPI_PULL_UP); //����
  gpio_init(C14, GPI, 1, GPI_PULL_UP); //����
  gpio_init(C15, GPI, 1, GPI_PULL_UP); //����

  gpio_init(C26, GPI, 1, GPI_PULL_UP); //��
  gpio_init(C27, GPI, 1, GPI_PULL_UP); //���뿪��
}

/**************************************************************************
�������ܣ������������
��ڲ�����_servo1_angle,_servo2_angle,_step_count�����1Ŀ���ٶȣ����2Ŀ���ٶȣ��������ã�
����ֵ����
��ע��_step_countԽС���ٶ�Խ�죨һ���������ã����٣�10�����٣�50�����٣�100��
����ʾ����servo_slow_ctrl(148,110,10);
**************************************************************************/
void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count)
{
  float servo1_start = (float)servo1_duty, servo2_start = (float)servo2_duty;
  float servo1_step = (float)(_servo1_angle - servo1_duty) / _step_count, servo2_step = (float)(_servo2_angle - servo2_duty) / _step_count;//ÿһ����Ҫִ�еĽǶ�
  while (1)
  {
    system_delay_ms(5);//fabsf�������������㵥���ȸ������ľ���ֵ�ģ�����˫���ȸ������ľ���ֵ��Ȼ�󽫽��ת���ɵ����ȸ���������
    if (fabsf(servo1_start - (float)_servo1_angle) >= servo1_step)//�������ֵ���ڵ��������Ķ���Ƕ�ʱ����ʼ�Ƕȼӵ����Ķ���Ƕ�
      servo1_start += servo1_step;
    else//�������ֵС�ڵ��������Ķ���Ƕ�
      servo1_start = _servo1_angle;//��ʼ�Ƕ�ֱ�Ӹ���Ϊ_servo1_angle
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_start));//SERVO_MOTOR_DUTY��ת���Ƕ�ת���ɶ��ռ�ձ�(�������)

    if (fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)
      servo2_start += servo2_step;
    else
      servo2_start = _servo2_angle;
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start));

    if (fabsf(servo1_start - (float)_servo1_angle) < 1 && fabsf(servo2_start - (float)_servo2_angle) < 1)//1Ϊ��Χ����������ĽǶ����ʼ�Ƕ�����Сʱ
    {
      servo1_duty = (uint16)_servo1_angle;
      servo2_duty = (uint16)_servo2_angle;//���¶���Ƕ�
      return;
    }
  }
}

/**************************************************************************
�������ܣ���е�ۿ���ģʽѡ��
��ڲ�����mode
����ֵ����
��ע��1. ʰȡģʽ 2. ����ģʽ 3. ����ģʽ 4. ����ģʽ�������� 5. ����Ĭ��Ϊ���У���ʱ���Խ׶�
����ʾ����arm_control(1);
**************************************************************************/
void arm_control(uint8 mode)
{
  switch (mode)
  {

  case 1:                  //ģʽ1��ʰȡģʽ
    gpio_set_level(C9, 1); //��������ߵ�ƽ����
    servo_slow_ctrl(148, 141, 5);
    break;

  case 2: //ģʽ2������ģʽ
    gpio_set_level(C9, 1);
    servo_slow_ctrl(148, 110, 10);
    servo_slow_ctrl(22, 110, 50); // 58 110   58  34//����ٶȲ�Ҫ�Ҹ�,��תǰ�ۣ���ת���
    servo_slow_ctrl(22, 38, 100); //���ײ�����ȷ��ʱ�䣺2023��4��24��18:02:30
    break;

  case 3: //ģʽ3������ģʽ
    gpio_set_level(C9, 0);
    servo_slow_ctrl(50, 50, 100);//Ĭ��ģʽ
    break;

  case 4: //ģʽ4�����ᱣ��ģʽ
    gpio_set_level(C9, 1);
    servo_slow_ctrl(148, 110, 10);
    break;

  case 5: //ģʽ5������ģʽ
          //���²�D27������C4,��Ӧ��C30����Ƕ�����
    if (!gpio_get_level(C4) && gpio_get_level(D27))
    {
      servo1_duty += 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_duty));
    }
    //���²�D27������C26��Ӧ��C30����Ƕȼ�С
    if (!gpio_get_level(C26) && gpio_get_level(D27))
    {
      servo1_duty -= 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_duty));
    }
    //���ϲ�D27������C4��Ӧ��C31����Ƕ�����
    if (!gpio_get_level(C4) && !gpio_get_level(D27))
    {
      servo2_duty += 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_duty));
    }
    //���ϲ�D27������C26��Ӧ��C31����Ƕȼ�С
    if (!gpio_get_level(C26) && !gpio_get_level(D27))
    {
      servo2_duty -= 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_duty));//ֱ�Ӹ�ֵ�۲�
    }
    if (!gpio_get_level(D4))
      gpio_set_level(C9, 1); //�ߵ�ƽ�Ӵ���
    else
      gpio_set_level(C9, 0);
    break;

  default:
    break; //���ӦC30��servo1�����ڶ�ӦC31��servo2��
  }
}