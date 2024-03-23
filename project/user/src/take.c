#include "take.h"
#include "math.h"
#include "isr.h"  
#include "zf_common_headfile.h"

#define PIT_CH_TIME (PIT_CH2)

uint16 servo1_duty = 50;
uint16 servo2_duty = 50;
uint16 servo3_duty = 50;//�����ʼ�Ƕ�

uint32 servo1_pwm = 0;
uint32 servo2_pwm = 0;
uint32 servo3_pwm = 0;//���ռ�ձ�

uint8 step = 1;
uint8 side_step = 1;
uint8 arm_flag = 0;//��ʱ��ɱ�־
uint8 arm_pick_flag = ARM_PICK_DONE; //
uint8 arm_state_flag = ARM_STATE_OFF;

uint8 one_pick = 0;
uint8 arm_put_down = 0;//��е�۷�����Ʒ�ı�־λ


void PIT_CH2_Int_Init(uint32 ldval)
{
    pit_ms_init(PIT_CH_TIME, ldval);//��ʼ��CH2ͨ�����ж�����Ϊ ldval ms
    interrupt_global_enable(0);
}

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

  gpio_init(C26, GPI, 1, GPI_PULL_UP); //���뿪��
  gpio_init(C27, GPI, 1, GPI_PULL_UP); //���뿪��
}

/**************************************************************************
�������ܣ������������
��ڲ�����_servo3_angle,_servo2_angle,_step_count�����1Ŀ���ٶȣ����2Ŀ���ٶȣ��������ã�
����ֵ����
��ע��_step_countԽС���ٶ�Խ�죨һ���������ã����٣�10�����٣�50�����٣�100��
����ʾ����servo_slow_ctrl(148,110,10);
**************************************************************************/
void servo_slow_ctrl(uint16 _servo3_angle, uint16 _servo2_angle, float _step_count)
{
  float servo3_start = (float)servo3_duty, servo2_start = (float)servo2_duty;//?bug���ֵĵط���
  float servo3_step = (float)(_servo3_angle - servo3_duty) / _step_count, servo2_step = (float)(_servo2_angle - servo2_duty) / _step_count;//ÿһ����Ҫִ�еĽǶ�
  while (1)
  {
    system_delay_ms(5);
		//fabsf�������������㵥���ȸ������ľ���ֵ�ģ�����˫���ȸ������ľ���ֵ��Ȼ�󽫽��ת���ɵ����ȸ���������
    if (fabsf(servo3_start - (float)_servo3_angle) >= servo3_step)//�������ֵ���ڵ��������Ķ���Ƕ�ʱ����ʼ�Ƕȼӵ����Ķ���Ƕ�
      servo3_start += servo3_step;
    else//�������ֵС�ڵ��������Ķ���Ƕ�
      servo3_start = _servo3_angle;//��ʼ�Ƕ�ֱ�Ӹ���Ϊ_servo3_angle
		
    servo1_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo3_start);//�鿴ռ�ձ�
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_start));//SERVO_MOTOR_DUTY��ת���Ƕ�ת���ɶ��ռ�ձ�(�������)

    if (fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)
      servo2_start += servo2_step;
    else
      servo2_start = _servo2_angle;
    servo2_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start);
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start));

    if (fabsf(servo3_start - (float)_servo3_angle) <= 1 && fabsf(servo2_start - (float)_servo2_angle) <= 1)//1Ϊ��Χ�����ڸ���������������ĽǶ����ʼ�Ƕ�����Сʱ
    {
      servo3_duty = (uint16)_servo3_angle;
      servo2_duty = (uint16)_servo2_angle;//
      return;
    }
  }
}
/**************************************************************************
�������ܣ���������������
��ڲ�����_servo3_angle,_step_count�����1Ŀ���ٶȣ����2Ŀ���ٶȣ��������ã�
����ֵ����
��ע��_step_countԽС���ٶ�Խ�죨һ���������ã����٣�10�����٣�50�����٣�100��
����ʾ����servo_slow_ctrl(148,10);servo3_duty
**************************************************************************/
void side_servo_slow_ctrl(uint16 _servo3_angle,float _step_count)
{
  float servo3_start = (float)servo3_duty;//��ʼ�Ƕ�
  float servo3_step = (float)(_servo3_angle - servo3_duty) / _step_count;//ÿһ����Ҫִ�еĽǶ�
  while (1)
  {
    system_delay_ms(5);
		//fabsf�������������㵥���ȸ������ľ���ֵ�ģ�����˫���ȸ������ľ���ֵ��Ȼ�󽫽��ת���ɵ����ȸ���������
    if (fabsf(servo3_start - (float)_servo3_angle) >= servo3_step)//�������ֵ���ڵ��������Ķ���Ƕ�ʱ����ʼ�Ƕȼӵ����Ķ���Ƕ�
      servo3_start += servo3_step;
    else//�������ֵС�ڵ��������Ķ���Ƕ�
      servo3_start = _servo3_angle;//��ʼ�Ƕ�ֱ�Ӹ���Ϊ_servo3_angle
		
    servo3_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo3_start);//�鿴ռ�ձ�
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_start));//SERVO_MOTOR_DUTY��ת���Ƕ�ת���ɶ��ռ�ձ�(�������)

    if (fabsf(servo3_start - (float)_servo3_angle) <= 1)//1Ϊ��Χ�����ڸ���������������ĽǶ����ʼ�Ƕ�����Сʱ
    {
      servo3_duty = (uint16)_servo3_angle;//���¶���Ƕ�
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
//  ips114_show_string( 0 , 40,   "SUCCESS");                          // ��ʾ�ַ���
  switch (mode)
  {

  case 1:                  //ģʽ1��ʰȡģʽ
    gpio_set_level(C9, 1); //��������ߵ�ƽ����
//	  ips114_show_string( 0 , 40,   "SUCCESS");                          //������ϵ�ɹ�
    servo_slow_ctrl(148, 141, 5);
	  ips114_show_string( 0 , 40,   "SUCCESS");                          //���Զ������ʾ�ɹ������Ƕ�����ᶯ
    break;

  case 2: //ģʽ2������ģʽ
    gpio_set_level(C9, 1);
    switch (step)
  {
    case 1:
        servo_slow_ctrl(148, 110, 10);
        PIT_CH2_Int_Init(10);
    if(arm_flag==1)//��ʱ��������ɱ�־
    {
       mode = 2;
       step = 2;
       arm_flag = 0;//��ռ�ʱ��־λ
    }
    case 2:
    servo_slow_ctrl(22, 110, 50); // 58 110   58  34//����ٶȲ�Ҫ�Ҹ�,��תǰ�ۣ���ת���
    PIT_CH2_Int_Init(10);
    if(arm_flag==1)//��ʱ��������ɱ�־
    {
       mode = 2;
       step = 3;
       arm_flag = 0;
    }
    case 3:
    servo_slow_ctrl(22, 38, 100); //���ײ�����ȷ��ʱ�䣺2023��4��24��18:02:30
    PIT_CH2_Int_Init(10);
      if(arm_flag==1)//��ʱ��������ɱ�־
    {
       arm_flag = 0;
       step = 0;//���step
       mode = 3;//���ٽ���ص�ģʽ2������Ĭ��ģʽ3�ϵ�
    }
    break;
  }
  case 3: //ģʽ3������ģʽ
    gpio_set_level(C9, 0);
    servo_slow_ctrl(50, 50, 100);//Ĭ��ģʽ
    break;

  case 4: //ģʽ4�����ᱣ��ģʽ
    gpio_set_level(C9, 1);
    servo_slow_ctrl(148, 110, 10);
    break;

  case 5: //ģʽ5������ģʽ
          //���²�C27������C14,��Ӧ��C30����Ƕ�����
    if (!gpio_get_level(C14) && gpio_get_level(C27))
    {
      servo3_duty += 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_duty));
    }
    //���²�C27������C26��Ӧ��C30����Ƕȼ�С
    if (!gpio_get_level(C26) && gpio_get_level(C27))
    {
      servo3_duty -= 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_duty));
    }
    //���ϲ�C27������C14��Ӧ��C31����Ƕ�����
    if (!gpio_get_level(C14) && !gpio_get_level(C27))
    {
      servo2_duty += 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_duty));
    }
    //���ϲ�C27������C26��Ӧ��C31����Ƕȼ�С
    if (!gpio_get_level(C26) && !gpio_get_level(C27))
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
  case 6: //ģʽ6�������е�ۿ���
  gpio_set_level(C9, 1);//������ϵ�
    switch (side_step)
  {
    case 1:
        side_servo_slow_ctrl(20, 30);//����
        PIT_CH2_Int_Init(10);
    if(arm_flag==1)//��ʱ��������ɱ�־
    {
       mode = 6;
       side_step = 2;
       arm_flag = 0;//��ռ�ʱ��־λ
    }
    case 2:
    side_servo_slow_ctrl(141, 50); //����
    PIT_CH2_Int_Init(10);
    if(arm_flag==1)//��ʱ��������ɱ�־
    {
       mode = 6;
       side_step = 3;
       arm_flag = 0;
    }
    case 3:
    side_servo_slow_ctrl(22, 100); //����
    PIT_CH2_Int_Init(10);
      if(arm_flag==1)//��ʱ��������ɱ�־
    {
       arm_flag = 0;
       side_step = 4;//���side_step
       mode = 0;//���ٽ���ص�ģʽ2������Ĭ��ģʽ3�ϵ�
    }
    case 4:
    gpio_set_level(C9, 0);//����Ĭ�϶ϵ�
    side_servo_slow_ctrl(50, 100); //�ָ���ʼ״̬
    PIT_CH2_Int_Init(10);
    if(arm_flag==1)//��ʱ��������ɱ�־
    {
       arm_flag = 0;
       side_step = 0;//���side_step
       mode = 0;//���ٽ���ص�ģʽ6
    }
    break;
  }
  default:
    break; //���ӦC30��servo1�����ڶ�ӦC31��servo2���������ӦC6��servo3��
  }
}
//*******************************���Ի�е��******************************//
void text_arm(void)
{
	my_pwm_gpio();//��ʼ����������
	arm_control(1);//���뺯����û�з�Ӧ�����ǵ�����ϵ�ɹ�
//	ips114_show_string( 0 , 40,   "SUCCESS");                          // ��ʾ�ַ���
}