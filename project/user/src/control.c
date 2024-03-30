#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#define CONTROL_FREQUENCY  100//编码器读取周期(0.01s 10ms)
float Car_H = 0.8;//车长
float Car_W = 0.6; // 车宽
int encoder[4];   // 编码器数据
float encoder_sum[4];//编码器累加值
float target_encoder_sum[4];//目标编码器累加值
float loc_target[4];//位置式处理后的速度
float loc_last_target[4];//上次位置式处理后的速度
int Turn_Left_flag,Turn_Right_flag;//左转右转标志
int loc_Finish_flag = 0;//位置式处理完成标志
int Location_pid_flag = 1;//位置式处理允许标志
float loc_err;//位置式输入误差
float abs_loc_err;//位置式输入误差绝对值
int pid_motor[4]; // PID处理后的电机pwm
float dt=0.005;

pid_info Pos_turn_pid[4];//位置式pid

pid_info Speed[4]; // 位置pid

/**
 * @brief 电机初始化
 * @param  无
 * @return 无
 */
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // gpio给高电平
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 
  gpio_init(DIR_RF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 
  gpio_init(DIR_RB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 

  pwm_init(motor_LF, 15000, 0); // PWM初始化
  pwm_init(motor_LB, 15000, 0); //
  pwm_init(motor_RF, 15000, 0); // 
  pwm_init(motor_RB, 15000, 0); // 

  
}


/**
 * @brief 编码器初始化
 * @param 无
 * @return 无
 */
void Encoder_Init(void)
{
  encoder_dir_init(ENCODER_LF, ENCODER_LF_LSB, ENCODER_LF_DIR); // 编码器通道初始化
  encoder_dir_init(ENCODER_LB, ENCODER_LB_LSB, ENCODER_LB_DIR); // 
  encoder_dir_init(ENCODER_RF, ENCODER_RF_LSB, ENCODER_RF_DIR); // 
  encoder_dir_init(ENCODER_RB, ENCODER_RB_LSB, ENCODER_RB_DIR); // 

  for(uint8 i=0;i<4;i++)
  {
    encoder[i]=0;//编码器清零
  }
}

/**
 * @brief 读取编码器数值
 * @param 无
 * @return 无
 */
void Read_Encoder(void)
{
  // ????????
  encoder[0] = -encoder_get_count(ENCODER_LF); // 左前
  encoder[1] = encoder_get_count(ENCODER_LB); // 左后
  encoder[2] = encoder_get_count(ENCODER_RF); // 右前
  encoder[3] = -encoder_get_count(ENCODER_RB); // 右后，正转读正

  // ????????????
  for(uint8 i=0;i<4;i++)
  {
    Speed[i].now_speed = (encoder[i] * 0.2636719*PI); // 编码器数据转换成车轮速度，单位为cm/s
  } 
  
  // ????????????
  encoder_clear_count(ENCODER_LF);
  encoder_clear_count(ENCODER_LB);
  encoder_clear_count(ENCODER_RF);
  encoder_clear_count(ENCODER_RB);
}

/**
 * @brief 麦轮速度解算1
 * @param 无
 * @return 无
 */
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = -(+target_Vx + target_Vy + target_Vz); // 左前
  Speed[1].target_speed = -(-target_Vx + target_Vy + target_Vz); // 左后
  Speed[2].target_speed = -(-target_Vx + target_Vy - target_Vz); // 右前
  Speed[3].target_speed = -(+target_Vx + target_Vy - target_Vz); // 右后
}

/**
 * @brief 麦轮速度解算2
 * @param target_Vx x轴目标速度
 * @param target_Vy y轴目标速度
 * @param target_Vz z轴目标速度
 * @return ??
 * @attention //
 */
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); //左前
  Speed[1].target_speed = -target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2);//左后
  Speed[2].target_speed = -target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2);//右前
  Speed[3].target_speed = target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); //右后
}
         
/**
 * @brief 位置式pid初始化
 * @param 无
 * @return 无
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
    Pos_turn_pid[i].xuhao=i; //序号
  }

  //左前
  Pos_turn_pid[0].kp = 0.5;   //0.5对应速度40
  Pos_turn_pid[0].kd = 0.5;   //0.5对应速度40
  //左后
  Pos_turn_pid[1].kp = 0.5;
  Pos_turn_pid[1].kd = 0.5;
  //右前
  Pos_turn_pid[2].kp = 0.5;
  Pos_turn_pid[2].kd = 0.5;
  //右后
  Pos_turn_pid[3].kp = 0.5;
  Pos_turn_pid[3].kd = 0.5; //PD赋值

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
    Speed[i].xuhao=i;  //序号
  }

  // ???
  Speed[0].kp = -22.5;
  Speed[0].ki = -1.50;
  // ???
  Speed[1].kp = -30;
  Speed[1].ki = -0.5;
  // ???
  Speed[2].kp = -25;
  Speed[2].ki = -0.5;
  //???
  Speed[3].kp = -25;
  Speed[3].ki = -0.8; //PI赋值

}
/**
 * @brief 增量式pid(单环pid)速度环
 * @param pid_info *pid 
 * @return pwm
 */
void increment_pid(void)
{
  for(uint8 i=0;i<4;i++)
  {
      //????
      Speed[i].lastlastError = Speed[i].lastError;  //记录上上次误差
      Speed[i].lastError = Speed[i].error;          //记录上次误差
      Speed[i].error = Speed[i].target_speed - Speed[i].now_speed; //计算本次误差
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //增量式处理
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //限幅
  }
}
/**
 * @brief 位置式pid(单环)
 * @param pid_info *pid:pid结构体pwm? 
 *        Target目标距离转换成的编码器数值 
 *        encoder编码器读数
 * @return pid->output 可以是其他值，在此处是脉冲数
 */
float Location_pid(pid_info *pid, float Encoder, float Target)
{
    pid->error = Target - Encoder; //Calculate the deviation //
    
    pid->output = pid->kp * pid->error + pid->kd * (pid->error-pid->lastError); //原本的+=，现在改成=      2024/3/26
    
    pid->lastError=pid->error;//记录下上次误差
	
    return pid->output;
}
/**
 * @brief 清空编码器累加值
 * @param 无
 * @return 无
 */
void clear_encoder_sum(void)
{
  encoder_sum[0] = 0;//编码器累加值归零
  encoder_sum[1] = 0;
  encoder_sum[2] = 0;
  encoder_sum[3] = 0;
}
/**
 * @brief 目标距离转换成脉冲数
 * @param 无
 * @return 无
 */
void Set_Distence_m(float distance)
{
  target_encoder_sum[0] = (distance/PI) *100 /0.2636719;//目标编码器累计脉冲值(即目标脉冲值)
  target_encoder_sum[1] = target_encoder_sum[0];
  target_encoder_sum[2] = target_encoder_sum[0];
  target_encoder_sum[3] = target_encoder_sum[0];
}

/**************************************************************************
位置环处理
**************************************************************************/
void Drive_Motor()
{
  float LF_Target,LB_Target,RF_Target,RB_Target; //各个轮子处理输出的脉冲值
	loc_err = Err_Handle();
	abs_loc_err = fabsf(Err_Handle());   //Err_Handle()

   if(abs_loc_err < 1.0)//设置中线绝对值阈值，小于这个值时，位置式不再起调整作用
  {
    loc_Finish_flag = 1;//位置式完成标志
    clear_encoder_sum();//清空编码器累计值
    int i = 0;
    for(i = 0;i < 4; i++) 
    {
      loc_target[i] = 0;//各个轮子的位置式输出速度归零
    }
  }
   
	if(loc_err > 0)       //这里还有点小bug，右转弯识别出的误差居然是<0,逆天
     Turn_Left_flag = 1;//左转标志位
  if(loc_err < 0)
     Turn_Right_flag =1;//右转标志位

  if(loc_Finish_flag == 0)//位置式调整
  {
    Set_Distence_m(abs_loc_err);//转换中线误差
	
//     int i = 0;
//     for(i = 0;i < 4; i++)
//    {
//        if(encoder_sum[i] < target_encoder_sum[i])
//      {
//          encoder_sum[i] += fabsf(encoder[i]);//测试使用
//      }
//        else
//      {
//          Location_pid_flag = 0;//位置式调整允许标志
//          loc_target[i] = loc_last_target[i];//记录上次处理结果
//      }
//    }
		
    LF_Target = Location_pid(&Pos_turn_pid[0], encoder_sum[0], target_encoder_sum[0]);
    LB_Target = Location_pid(&Pos_turn_pid[1], encoder_sum[0], target_encoder_sum[1]);
    RF_Target = Location_pid(&Pos_turn_pid[2], encoder_sum[0], target_encoder_sum[2]);
    RB_Target = Location_pid(&Pos_turn_pid[3], encoder_sum[0], target_encoder_sum[3]);//位置式处理
            
    loc_target[0] = LF_Target* 0.2636719 *PI /100;//将脉冲数转换成编码器速度
    loc_target[1] = LB_Target* 0.2636719 *PI /100;
    loc_target[2] = RF_Target* 0.2636719 *PI /100;
    loc_target[3] = RB_Target* 0.2636719 *PI /100;//单位为cm/s

  if(Turn_Left_flag==1)//左转，或者在中线右侧
  {
    loc_target[0] = -fabsf(loc_target[0]);
    loc_target[1] = -fabsf(loc_target[1]);
    loc_target[2] = fabsf(loc_target[2]);
    loc_target[3] = fabsf(loc_target[3]);
  }
  else if(Turn_Right_flag==1)//右转，或者在中线左侧
  {
    loc_target[0] = fabsf(loc_target[0]);
    loc_target[1] = fabsf(loc_target[1]);
    loc_target[2] = -fabsf(loc_target[2]);
    loc_target[3] = -fabsf(loc_target[3]);
  }
	
	  loc_last_target[0] = loc_target[0];//记录上次位置式处理速度
    loc_last_target[1] = loc_target[1];
    loc_last_target[2] = loc_target[2];
    loc_last_target[3] = loc_target[3];
  }
}
/**
 * @brief 串级pid 双环(位置环+速度环)
 * @param 无
 * @return ?无
 */
void turnloc_pid(void)
{
  Drive_Motor();//位置式处理

  for(uint8 i=0;i<4;i++)
  {
      //速度环
      Speed[i].lastlastError = Speed[i].lastError;  //记录上上次输出
      Speed[i].lastError = Speed[i].error;          //记录上次输出
      Speed[i].error = Speed[i].target_speed + loc_target[i] - Speed[i].now_speed; //改变目标速度
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //输出pwm
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //限幅
  }

  Turn_Left_flag = 0;
  Turn_Right_flag = 0;//左转/右转标志位清零

  loc_Finish_flag = 0;//位置式完成标志清零
}
/**
 * @brief 电机驱动，电机闭环控制
 * @param 无
 * @return 无
 * @attention 
 */
void motor_close_control(void)
{
  int j;
  for (j = 0; j < 4; j++) //各个电机的pwm赋值
  {
    pid_motor[j]=Speed[j].output;
    // Speed[j].output=0;
  }
    if (pid_motor[0] > 0) //正转
    {
      gpio_set_level(DIR_LF, 0);                 // DIR0
      pwm_set_duty(motor_LF, (int)pid_motor[0]); // 左前
    }
    else //��ת
    {
      gpio_set_level(DIR_LF, 1);
      pwm_set_duty(motor_LF, (int)-pid_motor[0]);
    }

    if (pid_motor[1] > 0) //???
    {
      gpio_set_level(DIR_LB, 0);
      pwm_set_duty(motor_LB, (int)pid_motor[1]);
    }
    else //???
    {
      gpio_set_level(DIR_LB, 1);
      pwm_set_duty(motor_LB, (int)-pid_motor[1]);
    }

    if (pid_motor[2] > 0) //???
    {
      gpio_set_level(DIR_RF, 1);
      pwm_set_duty(motor_RF, (int)pid_motor[2]);
    }
    else //��ת
    {
      gpio_set_level(DIR_RF, 0); //???
      pwm_set_duty(motor_RF, (int)-pid_motor[2]);
    }

    if (pid_motor[3] > 0) //???
    {
      gpio_set_level(DIR_RB, 1);
      pwm_set_duty(motor_RB, (int)pid_motor[3]);
    }
    else //???
    {
      gpio_set_level(DIR_RB, 0);
      pwm_set_duty(motor_RB, (int)-pid_motor[3]);
    }
  }

/**
 * @brief 18������������??�㺯�������ݱ��������������㳵����ʻ��??����λ��m
 * @param ??
 * @return ??
 * @attention ��ʻ��??����???? = (������������ / �������ֱ���) * (2 * �� * ????�뾶) / ????�ܳ�
 */
void Encoder_odometer(void)
{
  static float Angle_Bias = 0; //�Ƕ�ƫ��
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  // Angle_Bias = (90 - Angle_Z) * PI / 180; //����Ƕ�ƫ��

/****���㳵���ٶ�******/
  // V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637Ϊ�������ֱ�??
  // V_enco[1] = 0.2636719 * PI * (float)encoder[1];
  // V_enco[2] = 0.2636719 * PI * (float)encoder[2];
  // V_enco[3] = 0.2636719 * PI * (float)encoder[3];
  // Vx_enco=(V_enco[0]-V_enco[1]-V_enco[2]+V_enco[3])/4; //����X���ٶ�
  // Vy_enco=(V_enco[0]+V_enco[1]+V_enco[2]+V_enco[3])/4; //����Y���ٶ�
  // Vx_enco = -(V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //����X���ٶȣ�������
  // Vy_enco = -(V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //����Y���ٶȣ�������
  // Car_dis_x += Vx_enco * 0.01; //���㳵��X��??ʻ��??
  // Car_dis_y += Vy_enco * 0.01; //���㳵��Y��??ʻ��??

// #if 1
//  if (Angle_Bias >= 0)
//  {
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //���㳵���ٶ�����������ϵ�µķ���
//    Vx_world = Vx_2 - Vy_2;
//    Vy_world = Vx_1 + Vy_1;
//  }
//  if (Angle_Bias < 0)
//  {
//    Angle_Bias = -Angle_Bias;
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //���㳵���ٶ�����������ϵ�µķ���
//    Vx_world = Vx_2 + Vy_2;
//    Vy_world = -Vx_1 + Vy_1;
//  }
// #endif
//  Car_dis_x += Vx_world * 0.01; //���㳵��X��??ʻ���루?????
//  Car_dis_y += Vy_world * 0.01; //���㳵��Y��??ʻ���루?????

//  Car_dis_x2 += Vx_world * 0.01;
//  Car_dis_y2 += Vy_world * 0.01;
}

/**
 * @brief PID限幅
 *
 * @param Value    pid处理后输出的pwm
 * @param MaxValue 最大pwm
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
void test_pos(void)
{
    turnloc_pid();
}