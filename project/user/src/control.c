#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#define Row   180 //148
#define Col   180
#define CONTROL_FREQUENCY  100//��������ȡƵ��(������Ϊ0.01s 10ms)
float Vx, Vy, Vz;//����Ŀ���ٶ�
float target_motor[4];//Ŀ��pwm
float Car_H = 0.8;//����
float Car_W = 0.6; // ����
float turn_angle; // ת��Ƕ�
int spin; // ��ת��
int translation = 0; // ������
int encoder[4];   // �ĸ�������������ֵ
int encoder_sum[4];//�ĸ����������ۻ���ֵ
float target_encoder_sum[4];//�ĸ���������Ŀ���ۼ���ֵ
float loc_target[4];//λ��ʽ����������Ŀ���ٶ�
int Turn_Left_flag,Turn_Right_flag;//��ת��־����ת��־
int loc_Finish_flag = 0;//λ��ʽ������ɱ�־
int pid_motor[4]; // PID�����pwm
int midline[Row]; // ����λ������
float err_mid;//����ƫ�����
float PID_Bias[4]={0.0}, PID_Last_bias[4]={0.0};
float Keep_Bias; //��ͷ�Ƕ����
int test_count=0;
float dt=0.005;

pid_info Pos_turn_pid[4];//λ��ʽ������pid

pid_info Speed[4]; // �ٶȻ�pid

/**
 * @brief �����ʼ��
 * @param ��
 * @return��
 */
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // gpio�ڸ��ߵ�ƽ
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 
  gpio_init(DIR_RF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 
  gpio_init(DIR_RB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 

  pwm_init(motor_LF, 15000, 0); // PWMͨ����ʼ��
  pwm_init(motor_LB, 15000, 0); //
  pwm_init(motor_RF, 15000, 0); // 
  pwm_init(motor_RB, 15000, 0); // 

  
}


/**
 * @brief ��������ʼ��
 * @param �?
 * @return �??
 */
void Encoder_Init(void)
{
  encoder_dir_init(ENCODER_LF, ENCODER_LF_LSB, ENCODER_LF_DIR); // 
  encoder_dir_init(ENCODER_LB, ENCODER_LB_LSB, ENCODER_LB_DIR); // 
  encoder_dir_init(ENCODER_RF, ENCODER_RF_LSB, ENCODER_RF_DIR); // 
  encoder_dir_init(ENCODER_RB, ENCODER_RB_LSB, ENCODER_RB_DIR); // 

  for(uint8 i=0;i<4;i++)
  {
    encoder[i]=0;//��ձ���������
  }
}

/**
 * @brief ��ȡ��������ֵ
 * @param ��
 * @return ��
 */
void Read_Encoder(void)
{
  // ��������
  encoder[0] = -encoder_get_count(ENCODER_LF); // ��ǰ
  encoder[1] = encoder_get_count(ENCODER_LB); // ���
  encoder[2] = encoder_get_count(ENCODER_RF); // ��ǰ
  encoder[3] = -encoder_get_count(ENCODER_RB); // �Һ�

  // ���㳵��ʵ���ٶ�
  for(uint8 i=0;i<4;i++)
  {
    Speed[i].now_speed = (encoder[i] * 0.2636719*PI); // 0.2637��ȡ�����ٶȣ���λΪm/s
  } 
  
  // ��ձ���������
  encoder_clear_count(ENCODER_LF);
  encoder_clear_count(ENCODER_LB);
  encoder_clear_count(ENCODER_RF);
  encoder_clear_count(ENCODER_RB);
}

/**
 * @brief �����ٶȽ���1
 * @param ��
 * @return ��
 */
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = -(+target_Vx + target_Vy + target_Vz); // ��ǰ��
  Speed[1].target_speed = -(-target_Vx + target_Vy + target_Vz); // ���
  Speed[2].target_speed = -(-target_Vx + target_Vy - target_Vz); // ��ǰ
  Speed[3].target_speed = -(+target_Vx + target_Vy - target_Vz); // �Һ�
}

/**
 * @brief �����ٶȽ���2
 * @param target_Vx x��Ŀ���ٶ�
 * @param target_Vy y��Ŀ���ٶ�
 * @param target_Vz �����ٶ�
 * @return ��
 * @attention //
 */
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); // ��ǰ
  Speed[1].target_speed = -target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); // ���
  Speed[2].target_speed = -target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); // ��ǰ
  Speed[3].target_speed = target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); // �Һ�
}
         
/**
 * @brief λ��ʽpid��ʼ��
 * @param ��
 * @return ��
 */
void Pos_PidInit(void)
{
  for(uint8 i=0;i<4;i++)
  {
    Pos_turn_pid[i].target_speed = 0.00;
    Pos_turn_pid[i].target_pwm = 0;
    Pos_turn_pid[i].kp        = 0.00;
    Pos_turn_pid[i].ki        = 0.00;
    Pos_turn_pid[i].kd        = 0.00;
    Pos_turn_pid[i].error     = 0.00;
    Pos_turn_pid[i].lastError = 0.00;
    Pos_turn_pid[i].dError    = 0.00;
    Pos_turn_pid[i].output    = 0.00;
    Pos_turn_pid[i].output_last   = 0.00;
    Pos_turn_pid[i].xuhao=i; //���������
  }

  //��ǰ
  Pos_turn_pid[0].kp = 0.0;
  Pos_turn_pid[0].kd = 0.0;
  //���
  Pos_turn_pid[1].kp = 0.0;
  Pos_turn_pid[1].kd = 0.0;
  //��ǰ
  Pos_turn_pid[2].kp = 0.0;
  Pos_turn_pid[2].kd = 0.0;
  //�Һ�
  Pos_turn_pid[3].kp = 0.0;
  Pos_turn_pid[3].kd = 0.0; //�������ӷֿ�����

}

void PidInit(void)
{
  for(uint8 i=0;i<4;i++)
  {
    Speed[i].target_speed = 0.00;
    Speed[i].target_pwm = 0;
    Speed[i].kd= 0.00;
    Speed[i].ki        = 0.00;
    Speed[i].kd        = 0.00;
    Speed[i].error     = 0.00;
    Speed[i].lastError = 0.00;
    Speed[i].dError    = 0.00;
    Speed[i].output    = 0.00;
    Speed[i].output_last   = 0.00;
    Speed[i].xuhao=i; //���������
  }

  // ��ǰ
  Speed[0].kp = -22.5;
  Speed[0].ki = -1.50;
  // ���
  Speed[1].kp = -30;
  Speed[1].ki = -0.5;
  // ��ǰ
  Speed[2].kp = -25;
  Speed[2].ki = -0.5;
  //�Һ�
  Speed[3].kp = -25;
  Speed[3].ki = -0.8; ////�������ӷֿ����� 

}
/**
 * @brief ����ʽ�ٶȻ�(�ڻ�)
 * @param pid_info *pid 
 * @return �������ӵ�pwm
 */
void increment_pid(void)
{
  for(uint8 i=0;i<4;i++)
  {
      //�ٶȻ�
      Speed[i].lastlastError = Speed[i].lastError;  //��¼���ϴ����
      Speed[i].lastError = Speed[i].error;          //��¼�ϴ����
      Speed[i].error = Speed[i].target_speed - Speed[i].now_speed; //�������
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //����ʽ����
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //�޷�
  }
}
/**
 * @brief λ��ʽpid����
 * @param ���룺pid_info *pid�ĸ����ӵ�pwmֵ Target��Ŀ�����Ĵ����ı��������ۼ�ֵ encoder���������ۼ�������ֵ
 * @return ��������ٶ�pid->output��׼���������ٶȻ��ջ�
 */
int Location_pid(pid_info *pid, float Encoder, float Target)
{
    static float Bias,Speed;

    Bias= Target - Encoder; //Calculate the deviation //����ƫ��
    
    pid->output = pid->kp*Bias + pid->kd * (Bias-pid->lastError); //ԭ����+=���ָ�Ϊ=      2024/3/26
    
    pid->lastError=Bias;//�����ϴ�ƫ��
    return pid->output;
}
/**
 * @brief ��ձ������ۼ�ֵ
 * @param ��
 * @return ��
 */
void clear_encoder_sum(void)
{
  encoder_sum[0] = 0;//�������ۼ�ֵ
  encoder_sum[1] = 0;
  encoder_sum[2] = 0;
  encoder_sum[3] = 0;
}

void Set_Distence_m(uint16_t distance)
{
  clear_encoder_sum();    

  target_encoder_sum[0] = (distance/PI) * 100/0.2636719;//���ｫĿ�����ת���������ۼ�
  target_encoder_sum[1] = target_encoder_sum[0];
  target_encoder_sum[2] = target_encoder_sum[0];
  target_encoder_sum[3] = target_encoder_sum[0];
}

/**************************************************************************
λ�û�����ٶ�
**************************************************************************/
void Drive_Motor()
{
  float LF_Target,LB_Target,RF_Target,RB_Target;
  //����������������� Ҫ�����ٶȻ����轫λ�û��������������ת��Ϊ�ٶ�
  encoder_sum[0] += encoder[0];//�������ۼ�ֵ
  encoder_sum[1] += encoder[1];
  encoder_sum[2] += encoder[2];
  encoder_sum[3] += encoder[3];

    LF_Target = Location_pid(Pos_turn_pid[0], -encoder_sum[0], target_encoder_sum[0]);
    LB_Target = Location_pid(Pos_turn_pid[1], -encoder_sum[0], target_encoder_sum[1]);
    RF_Target = Location_pid(Pos_turn_pid[2], -encoder_sum[0], target_encoder_sum[2]);
    RB_Target = Location_pid(Pos_turn_pid[3], -encoder_sum[0], target_encoder_sum[3]);
            
    loc_target[0] = LF_Target*CONTROL_FREQUENCY*0.2636719;
    loc_target[1] = LB_Target*CONTROL_FREQUENCY*0.2636719;
    loc_target[2] = RF_Target*CONTROL_FREQUENCY*0.2636719;
    loc_target[3] = RB_Target*CONTROL_FREQUENCY*0.2636719;

  if(Turn_Left_flag==1)//��ת
  {
    loc_target[0]=-fabsf(loc_target[0]);
    loc_target[1]=-fabsf(loc_target[1]);
    loc_target[2]=fabsf(loc_target[2]);
    loc_target[3]=fabsf(loc_target[3]);
  }
  else if(Turn_Right_flag==1)//��ת
  {
    loc_target[0]=fabsf(loc_target[0]);
    loc_target[1]=fabsf(loc_target[1]);
    loc_target[2]=-fabsf(loc_target[2]);
    loc_target[3]=-fabsf(loc_target[3]);
  }
}
void turnloc_pid(void)
{
  if(Err_Handle()<2)//�趨һ����ֵ��Χ��С�������ֵʱ��Ϊ������Ҫ����,ֻ���ٶȻ���Ŀ���ٶȱջ�
  {
    loc_Finish_flag = 1;//λ�õ�����ɱ�־
    clear_encoder_sum();//�������ۼ�ֵ��0
  }
  if(loc_Finish_flag == 0)//λ�õ���δ���
  {
  Set_Distence_m(Err_Handle());//�����ٶ�ת�������,��Ҫ����һ�²���
  if(Err_Handle() < 0)
     Turn_Left_flag = 1;
  if(Err_Handle() > 0)
     Turn_Right_flag =1;
  Drive_Motor();//λ�û�����ٶ�

  for(uint8 i=0;i<4;i++)
  {
      //�����ٶȻ�����
      Speed[i].lastlastError = Speed[i].lastError;  //��¼���ϴ����
      Speed[i].lastError = Speed[i].error;          //��¼�ϴ����
      Speed[i].error = Speed[i].target_speed + loc_target[i] - Speed[i].now_speed; //�������,���ܻ᲻��ԭ����Ŀ���ٶ�
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //����ʽ�������pwm
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //�޷�
  }
  Turn_Left_flag = 0;
  Turn_Right_flag = 0;//��־λ����
  }
}
/**
 * @brief �ٶȱջ�����
 * @param ��
 * @return ��
 * @attention 
 */
void motor_close_control(void)
{
  int j;
  for (j = 0; j < 4; j++) //???
  {
    pid_motor[j]=Speed[j].output;//������pwm��ֵ
    // Speed[j].output=0;
  }
    if (pid_motor[0] > 0) //��ת
    {
      gpio_set_level(DIR_LF, 0);                 // DIR0
      pwm_set_duty(motor_LF, (int)pid_motor[0]); // ��ǰ
    }
    else //反转
    {
      gpio_set_level(DIR_LF, 1);
      pwm_set_duty(motor_LF, (int)-pid_motor[0]);
    }

    if (pid_motor[1] > 0) //��ת
    {
      gpio_set_level(DIR_LB, 0);
      pwm_set_duty(motor_LB, (int)pid_motor[1]);
    }
    else //��ת
    {
      gpio_set_level(DIR_LB, 1);
      pwm_set_duty(motor_LB, (int)-pid_motor[1]);
    }

    if (pid_motor[2] > 0) //��ת
    {
      gpio_set_level(DIR_RF, 1);
      pwm_set_duty(motor_RF, (int)pid_motor[2]);
    }
    else //反转
    {
      gpio_set_level(DIR_RF, 0); //��ת
      pwm_set_duty(motor_RF, (int)-pid_motor[2]);
    }

    if (pid_motor[3] > 0) //��ת
    {
      gpio_set_level(DIR_RB, 1);
      pwm_set_duty(motor_RB, (int)pid_motor[3]);
    }
    else //��ת
    {
      gpio_set_level(DIR_RB, 0);
      pwm_set_duty(motor_RB, (int)-pid_motor[3]);
    }
  }
/**
 * @brief �����������
 * @param ��
 * @return ��
 * @attention 
 */
void motor_control(void)
{
  for (int j= 0; j < 4; j++) //���pwm��ֵ
  {
    if (pid_motor[j] > AMPLITUDE_MOTOR)
      pid_motor[j] = AMPLITUDE_MOTOR;
    if (pid_motor[j] < -AMPLITUDE_MOTOR)
      pid_motor[j] = -AMPLITUDE_MOTOR;
  }
  if (pid_motor[0] > 0) //��ǰ
  {
    gpio_set_level(DIR_LF, 0);                 // DIR0
    pwm_set_duty(motor_LF, (int)pid_motor[0]); // ��ת
  }
  else //DIR1   ��ת
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-pid_motor[0]);
  }

  if (pid_motor[1] > 0) //���
  {
    gpio_set_level(DIR_LB, 0);               // DIR0
    pwm_set_duty(motor_LB, (int)pid_motor[1]);// ��ת
  }
  else //DIR1  ��ת
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-pid_motor[1]);
  }

  if (pid_motor[2] > 0) //��ǰ
  {
    gpio_set_level(DIR_RF, 1);
    pwm_set_duty(motor_RF, (int)pid_motor[2]);// ��ת
  }
  else //DIR0 ��ת
  {
    gpio_set_level(DIR_RF, 0);
    pwm_set_duty(motor_RF, (int)-pid_motor[2]);
  }

  if (pid_motor[3] > 0) //DIR1  
  {
    gpio_set_level(DIR_RB, 1);
    pwm_set_duty(motor_RB, (int)pid_motor[3]);// ��ת
  }
  else //DIR0  ��ת
  {
    gpio_set_level(DIR_RB, 0);
    pwm_set_duty(motor_RB, (int)-pid_motor[3]);
  }
}

/**
 * @brief 18电机编码器里程�?算函数，根据编码器脉冲数计算车辆行驶距�?，单位：m
 * @param �?
 * @return �?
 * @attention 行驶距�?计算�?�� = (编码器脉冲数 / 编码器分辨率) * (2 * π * �?��半径) / �?��周长
 */
void Encoder_odometer(void)
{
  static float Angle_Bias = 0; //角度偏差
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  // Angle_Bias = (90 - Angle_Z) * PI / 180; //计算角度偏差

/****计算车辆速度******/
  // V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637为编码器分辨�?
  // V_enco[1] = 0.2636719 * PI * (float)encoder[1];
  // V_enco[2] = 0.2636719 * PI * (float)encoder[2];
  // V_enco[3] = 0.2636719 * PI * (float)encoder[3];
  // Vx_enco=(V_enco[0]-V_enco[1]-V_enco[2]+V_enco[3])/4; //计算X轴速度
  // Vy_enco=(V_enco[0]+V_enco[1]+V_enco[2]+V_enco[3])/4; //计算Y轴速度
  // Vx_enco = -(V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //计算X轴速度（修正）
  // Vy_enco = -(V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //计算Y轴速度（修正）
  // Car_dis_x += Vx_enco * 0.01; //计算车辆X轴�?驶距�?
  // Car_dis_y += Vy_enco * 0.01; //计算车辆Y轴�?驶距�?

// #if 1
//  if (Angle_Bias >= 0)
//  {
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //计算车辆速度在世界坐标系下的分量
//    Vx_world = Vx_2 - Vy_2;
//    Vy_world = Vx_1 + Vy_1;
//  }
//  if (Angle_Bias < 0)
//  {
//    Angle_Bias = -Angle_Bias;
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //计算车辆速度在世界坐标系下的分量
//    Vx_world = Vx_2 + Vy_2;
//    Vy_world = -Vx_1 + Vy_1;
//  }
// #endif
//  Car_dis_x += Vx_world * 0.01; //计算车辆X轴�?驶距离（�??�?
//  Car_dis_y += Vy_world * 0.01; //计算车辆Y轴�?驶距离（�??�?

//  Car_dis_x2 += Vx_world * 0.01;
//  Car_dis_y2 += Vy_world * 0.01;
}

/**
 * @brief PID�޷�
 *
 * @param Value    pid������pwm
 * @param MaxValue ���pwm
 * @return float
 */
float PIDInfo_Limit(float Value, float MaxValue)
{
	if (fabs(Value) > MaxValue)
	{
		if (Value >= 0)
			Value = MaxValue;
		else
			Value = -MaxValue;
	}

	return Value;
}