#include "control.h"
#include "zf_common_headfile.h"
float Vx, Vy, Vz;//VxΪ����x�᷽���ٶȣ�VyΪ����y�᷽���ٶȣ�Vz�������ĳ�������ת���ٶ�
float target_motor[4];//�ĸ����ӵ�Ŀ��pwmֵ
float Car_H = 0.8;//������
float Car_W = 0.6;//�����ȣ���λ��Ϊm,���������
float Velocity_KP = 0.8; //�ٶ�PID
float Velocity_KI = 1.6; //�ٶ�PID�����ﻹû��ȷ�����շ�����������򵥵���д
float turn_angle;//����ת��Ƕ�
int encoder[4];//��ű�������ֵ
float PID_motor[4];//���pid��������ֵ
pid_info LF_motor_pid;//���pid
pid_info RF_motor_pid;
pid_info LB_motor_pid;
pid_info RB_motor_pid;


/*******************************************�����ʼ��*************************************/
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); //��ǰ
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); //���
  gpio_init(DIR_RF, GPO, GPIO_HIGH, GPO_PUSH_PULL); //��ǰ
  gpio_init(DIR_RB, GPO, GPIO_HIGH, GPO_PUSH_PULL); //�Һ�

  pwm_init(motor_LF, 15000, 0); //��ǰpwm
  pwm_init(motor_LB, 15000, 0); //���pwm
  pwm_init(motor_RF, 15000, 0); //��ǰpwm
  pwm_init(motor_RB, 15000, 0); //�Һ�pwm
}


/*******************************************��������ʼ��***********************************/
void Encoder_Init(void)
{
  encoder_dir_init(ENCODER_LF, ENCODER_LF_LSB, ENCODER_LF_DIR); // ��ʼ��������ģ�������� ���������ģʽ
  encoder_dir_init(ENCODER_LB, ENCODER_LB_LSB, ENCODER_LB_DIR); // ��ʼ��������ģ�������� ���������ģʽ
  encoder_dir_init(ENCODER_RF, ENCODER_RF_LSB, ENCODER_RF_DIR); // ��ʼ��������ģ�������� ���������ģʽ
  encoder_dir_init(ENCODER_RB, ENCODER_RB_LSB, ENCODER_RB_DIR); // ��ʼ��������ģ�������� ���������ģʽ
}


/*******************************************��ȡ��������ֵ*********************************/
void Read_Encoder(void)
{
  encoder[0] = encoder_get_count(ENCODER_LF);
  encoder[1] = encoder_get_count(ENCODER_LB);
  encoder[2] = -encoder_get_count(ENCODER_RF);
  encoder[3] = -encoder_get_count(ENCODER_RB);
  //dasdas
  //��ձ���������
  encoder_clear_count(ENCODER_LF);
  encoder_clear_count(ENCODER_LB);
  encoder_clear_count(ENCODER_RF);
  encoder_clear_count(ENCODER_RB);
}

/*******************************************���ֿ��������***************************************
����ֵ:���̵�x�ᣬy�ᣬ�Ƴ�������ת�Ľ��ٶ�
����ֵ:�ĸ����ӵĿ����ٶ�
************************************************************************************************/
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz) //��ڲ���С��Ŀ��xyz�ٶ�
{
  target_motor[0] = -(+target_Vx + target_Vy + target_Vz); //������
  target_motor[1] = -(-target_Vx + target_Vy + target_Vz); //������
  target_motor[2] = -(-target_Vx + target_Vy - target_Vz); //������
  target_motor[3] = -(+target_Vx + target_Vy - target_Vz); //������
}

// �����ƶ���ת��Ϊ�����ٶ�  x:ǰ+��-  y:��+��-  z:��+˳-
void Move_Transfrom(double target_Vx, double target_Vy, double target_Vz)
{
	target_motor[0]= target_Vx + target_Vy - target_Vz*(Car_H/2+Car_W/2);
	target_motor[1]= -target_Vx + target_Vy - target_Vz*(Car_H/2+Car_W/2);
	target_motor[2]= -target_Vx + target_Vy + target_Vz*(Car_H/2+Car_W/2);
	target_motor[3]= target_Vx + target_Vy + target_Vz*(Car_H/2+Car_W/2);
}
//void Move_Transfrom(double Vx,double Vy,double Vz)//����Ϊ����ǰ��Ϊ��
//{
//	PID_motor[0]=Vx-Vy-Vz*(Car_H/2+Car_W/2);
//	PID_motor[1]=Vx+Vy-Vz*(Car_H/2+Car_W/2);
//	PID_motor[2]=Vx+Vy+Vz*(Car_H/2+Car_W/2);
//	PID_motor[3]=Vx-Vy+Vz*(Car_H/2+Car_W/2);
//}
                    
/**************************************************************************
�������ܣ�����ʽ�ٶ�PI �����������pid����Ҫ��ֵ
��ڲ�����PID_motor[4]   encoder[4]
����  ֵ���� 
**************************************************************************/
void Incremental_PI(void)
{
  static float PID_err[4], PID_last_err[4],PID_previous_Err[4];//PID_err�����ű�����PID_last_err����ϴ����

  PID_err[0] = target_motor[0] - encoder[0]; //��ǰƫ��ֵ
  PID_err[1] = target_motor[1] - encoder[1]; //��ǰƫ��ֵ
  PID_err[2] = target_motor[2] - encoder[2]; //��ǰƫ��ֵ
  PID_err[3] = target_motor[3] - encoder[3]; //��ǰƫ��ֵ

  PID_motor[0] += Velocity_KP * (PID_err[0] - PID_last_err[0]) + Velocity_KI * PID_err[0]; //����ʽPI������
  PID_motor[1] += Velocity_KP * (PID_err[1] - PID_last_err[1]) + Velocity_KI * PID_err[1]; //����ʽPI������
  PID_motor[2] += Velocity_KP * (PID_err[2] - PID_last_err[2]) + Velocity_KI * PID_err[2]; //����ʽPI������
  PID_motor[3] += Velocity_KP * (PID_err[3] - PID_last_err[3]) + Velocity_KI * PID_err[3]; //����ʽPI������

  PID_last_err[0] = PID_err[0]; //������һ��ƫ��
  PID_last_err[1] = PID_err[1]; //������һ��ƫ��
  PID_last_err[2] = PID_err[2]; //������һ��ƫ��
  PID_last_err[3] = PID_err[3]; //������һ��ƫ��
}
//pid������ʼ��
void PidInit(pid_info * pid)
{
  pid->kp        = 0;
  pid->ki        = 0;
  pid->kd        = 0;
  pid->error     = 0;
  pid->lastError = 0;
  pid->dError    = 0;
  pid->output    = 0;
  pid->output_last   = 0;
}
//����ʽPID
float increment_pid(float error,pid_info *pid)
{
	pid->error = error;
	pid->dError = error - pid->lastError;//����������ϴ�����ƫ��ֵ
	pid->lastError = error;//��¼�±������Ա��´�ʹ��
	pid->output =(pid->kp*pid->dError)+(pid->ki*error); //�ٶ�pi���Ʊջ�
	pid->output_last = pid->output;//��¼�±��ε�pid���
	return pid->output;//��������ʽ�ļ���ֵ
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PID��ֵ
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:pi����
//-------------------------------------------------------------------------------------------------------------------
void PID_cale()
{
  PidInit(&LF_motor_pid);//��ʼ��pid����
	PidInit(&LB_motor_pid);
	PidInit(&RF_motor_pid);
	PidInit(&RB_motor_pid);
	
	LF_motor_pid.kp=0.8;
	LF_motor_pid.ki=1.6;
	
	RF_motor_pid.kp=0.8;
	RF_motor_pid.ki=1.6;
	
	LB_motor_pid.kp=0.8;
	LB_motor_pid.ki=1.6;
	
	LB_motor_pid.kp=0.8;
	LB_motor_pid.ki=1.6;

	PID_motor[0] += increment_pid(target_motor[0]-encoder[0],&LF_motor_pid);
  PID_motor[1] += increment_pid(target_motor[1]-encoder[1],&LB_motor_pid);
  PID_motor[2] += increment_pid(target_motor[2]-encoder[2],&RF_motor_pid);
  PID_motor[3] += increment_pid(target_motor[3]-encoder[3],&RB_motor_pid);//�����ĸ������ٶȱջ��������pwm
}


/************************************����ջ���������*************************/
void motor_close_control(void)
{
  int j;
  int Amplitude_motor = 20000; //===PWM������50000 ������50000 ����ٶ���20000����

  for (j = 0; j < 4; j++) //�޷�
  {
    if (PID_motor[j] > Amplitude_motor)
      PID_motor[j] = Amplitude_motor;
    if (PID_motor[j] < -Amplitude_motor)
      PID_motor[j] = -Amplitude_motor;
  }

  // DIR1�ĳ���0��0��1

  if (target_motor[0] > 0) //���1   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/TIMER1_PWM_DUTY_MAX*100)
  {
    gpio_set_level(DIR_LF, 0);                 // DIR����ߵ�ƽ
    pwm_set_duty(motor_LF, (int)PID_motor[0]); // ����ռ�ձ�
  }
  else //���1   ��ת
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-PID_motor[0]);
  }

  if (PID_motor[1] > 0) //���2   ��ת
  {
    gpio_set_level(DIR_LB, 0);
    pwm_set_duty(motor_LB, (int)PID_motor[1]);
  }
  else //���2   ��ת
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-PID_motor[1]);
  }

  if (PID_motor[2] > 0) //���3   ��ת
  {
    gpio_set_level(DIR_RF, 1);
    pwm_set_duty(motor_RF, (int)PID_motor[2]);
  }
  else //���3   ��ת
  {
    gpio_set_level(DIR_RF, 0);
    pwm_set_duty(motor_RF, (int)-PID_motor[2]);
  }

  if (PID_motor[3] > 0) //���4   ��ת
  {
    gpio_set_level(DIR_RB, 1);
    pwm_set_duty(motor_RB, (int)PID_motor[3]);
  }
  else //���4   ��ת
  {
    gpio_set_level(DIR_RB, 0);
    pwm_set_duty(motor_RB, (int)-PID_motor[3]);
  }
}
/************************************���������������*************************/
void motor_control(void)
{
  int j;
  int Amplitude_motor = 20000; //===PWM������50000 ������50000 ����ٶ���20000����

  for (j = 0; j < 4; j++) //�޷�
  {
    if (target_motor[j] > Amplitude_motor)
      target_motor[j] = Amplitude_motor;
    if (target_motor[j] < -Amplitude_motor)
      target_motor[j] = -Amplitude_motor;
  }

  // DIR1�ĳ���0��0��1

  if (target_motor[0] > 0) //���1   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/TIMER1_PWM_DUTY_MAX*100)
  {
    gpio_set_level(DIR_LF, 0);                 // DIR����ߵ�ƽ
    pwm_set_duty(motor_LF, (int)target_motor[0]); // ����ռ�ձ�
  }
  else //���1   ��ת
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-target_motor[0]);
  }

  if (PID_motor[1] > 0) //���2   ��ת
  {
    gpio_set_level(DIR_LB, 0);
    pwm_set_duty(motor_LB, (int)target_motor[1]);
  }
  else //���2   ��ת
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-target_motor[1]);
  }

  if (PID_motor[2] > 0) //���3   ��ת
  {
    gpio_set_level(DIR_RF, 1);
    pwm_set_duty(motor_RF, (int)target_motor[2]);
  }
  else //���3   ��ת
  {
    gpio_set_level(DIR_RF, 0);
    pwm_set_duty(motor_RF, (int)-target_motor[2]);
  }

  if (PID_motor[3] > 0) //���4   ��ת
  {
    gpio_set_level(DIR_RB, 1);
    pwm_set_duty(motor_RB, (int)target_motor[3]);
  }
  else //���4   ��ת
  {
    gpio_set_level(DIR_RB, 0);
    pwm_set_duty(motor_RB, (int)-target_motor[3]);
  }
}

/***********************************************��װ�ٶȺ���******************************************/
void Speed_Control(float Vx_Speed, float Vy_Speed, float Vz_Speed)
{
	Read_Encoder();
  Move_Transfrom(Vx_Speed, Vy_Speed, Vz_Speed);//�д�����
  PID_cale();//PID��ֵ�����
//  Car_Inverse_kinematics_solution(Vx_Speed, Vy_Speed, Vz_Speed);
  motor_control();
	//motor_close_control();//�ջ��������
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת��
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage: valueΪת��Ƕȣ�turn_speedΪת���ٶ�
//-------------------------------------------------------------------------------------------------------------------
void Turn_angle(float value,int16 turn_speed)//
{
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������ٶ�     (�ٶ� = ( (�������� / ����������)  * ���������� / ��ģ���� ) * ���ܳ� / �����ȡʱ��)
//  @param      encoder  ��������ֵ
//  @return     V_enco   �ñ��������ٶȣ�mm/s��
//  @since      v1.0
//  Sample usage: ENCO_speed(master_encoder_left)
//-------------------------------------------------------------------------------------------------------------------
//17�����ֱ������ٶ���λ��
//AT_ITCM_SECTION_INIT(float ENCO_speed(int16 encoder))
//float ENCO_speed(int16 encoder)
//{
//		V_enco=(encoder*50*PI*11000/104/1024)*0.9f;//0.932f//*0.9f;//*0.9153f;//*0.9153f;  //  ��λ��mm/s��(11000=55/0.005) (encoder=61.634->����1m/s)
//	V_enco=encoder*16.224796f;
//	return V_enco;
//}

///**************************************************************************
//�������ܣ�18���Ӿ���ȡ��������ֵ�����㳵���ٶȲ����λ�ƣ���λm/s
//(�����ٶ� = ( (�������� / ����������)  * ���������� / ��ģ���� ) * ���ܳ� / �����ȡʱ��)
//��ڲ�������
//����ֵ����
//**************************************************************************/
//void Encoder_odometer(void)
//{
//  static float Angle_Bias = 0;//��Ƕ�λ�ƣ�����
//  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

//  Angle_Bias = (90 - Angle_Z) * PI / 180;

///****��������ٶ�******/
//  V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637�����پ�ȷ����λ
//  V_enco[1] = 0.2636719 * PI * (float)encoder[1];
//  V_enco[2] = 0.2636719 * PI * (float)encoder[2];
//  V_enco[3] = 0.2636719 * PI * (float)encoder[3];
//  //�������ԭ���ģ�û�и���
//  //    Vx_enco=(V_enco[0]-V_enco[1]-V_enco[2]+V_enco[3])/4;//����Ϊ��
//  //    Vy_enco=(V_enco[0]+V_enco[1]+V_enco[2]+V_enco[3])/4;//ǰ��Ϊ��
//  Vx_enco = -(V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //����Ϊ���������ȥ��������
//  Vy_enco = -(V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //ǰ��Ϊ��
//  //    Car_dis_x+=Vx_enco*0.01;
//  //    Car_dis_y+=Vy_enco*0.01;

//#if 1
//  if (Angle_Bias >= 0)
//  {
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //�ֽ⵽����������
//    Vx_world = Vx_2 - Vy_2;
//    Vy_world = Vx_1 + Vy_1;
//  }
//  if (Angle_Bias < 0)
//  {
//    Angle_Bias = -Angle_Bias;
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //�ֽ⵽����������
//    Vx_world = Vx_2 + Vy_2;
//    Vy_world = -Vx_1 + Vy_1;
//  }
//#endif
//  Car_dis_x += Vx_world * 0.01;
//  Car_dis_y += Vy_world * 0.01;

//  Car_dis_x2 += Vx_world * 0.01;
//  Car_dis_y2 += Vy_world * 0.01;
//}
