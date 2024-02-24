#include "control.h"
#include "zf_common_headfile.h"
float Vx, Vy, Vz;//Vx为整车x轴方向速度，Vy为整车y轴方向速度，Vz是整车饶车中心旋转的速度
float target_motor[4];//四个轮子的目标pwm值
float Car_H = 0.8;//车身长度
float Car_W = 0.6;//车身宽度，单位均为m,可随意调整
float Velocity_KP = 0.8; //速度PID
float Velocity_KI = 1.6; //速度PID；这里还没有确定最终方案，先用最简单的来写
int encoder[4];//存放编码器数值
float PID_motor[4];//存放pid输出后的数值



/*******************************************电机初始化*************************************/
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); //左前
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); //左后
  gpio_init(DIR_RF, GPO, GPIO_HIGH, GPO_PUSH_PULL); //右前
  gpio_init(DIR_RB, GPO, GPIO_HIGH, GPO_PUSH_PULL); //右后

  pwm_init(motor_LF, 15000, 1000); //左前pwm
  pwm_init(motor_LB, 15000, 1000); //左后pwm
  pwm_init(motor_RF, 15000, 1000); //右前pwm
  pwm_init(motor_RB, 15000, 1000); //右后pwm
}


/*******************************************编码器初始化***********************************/
void Encoder_Init(void)
{
  encoder_dir_init(ENCODER_LF, ENCODER_LF_LSB, ENCODER_LF_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_LB, ENCODER_LB_LSB, ENCODER_LB_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_RF, ENCODER_RF_LSB, ENCODER_RF_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_RB, ENCODER_RB_LSB, ENCODER_RB_DIR); // 初始化编码器模块与引脚 方向编码器模式
}


/*******************************************读取编码器数值*********************************/
void Read_Encoder(void)
{
  encoder[0] = encoder_get_count(ENCODER_LF);
  encoder[1] = encoder_get_count(ENCODER_LB);
  encoder[2] = -encoder_get_count(ENCODER_RF);
  encoder[3] = -encoder_get_count(ENCODER_RB);
  //dasdas
  //清空编码器计数
  encoder_clear_count(ENCODER_LF);
  encoder_clear_count(ENCODER_LB);
  encoder_clear_count(ENCODER_RF);
  encoder_clear_count(ENCODER_RB);
}

/*******************************************麦轮控制逆解算***************************************
输入值:底盘的x轴，y轴，绕车中心旋转的角速度
返回值:四个轮子的控制速度
************************************************************************************************/
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz) //入口参数小车目标xyz速度
{
  target_motor[0] = -(+target_Vx + target_Vy + target_Vz); //左上轮
  target_motor[1] = -(-target_Vx + target_Vy + target_Vz); //左下轮
  target_motor[2] = -(-target_Vx + target_Vy - target_Vz); //右上轮
  target_motor[3] = -(+target_Vx + target_Vy - target_Vz); //右下轮
}

// 整车移动量转换为单轮速度  x:前+后-  y:左+右-  z:逆+顺-
void Move_Transfrom(double target_Vx, double target_Vy, double target_Vz)
{
	target_motor[0]= target_Vx + target_Vy - target_Vz*(Car_H/2+Car_W/2);
	target_motor[1]= -target_Vx + target_Vy - target_Vz*(Car_H/2+Car_W/2);
	target_motor[2]= -target_Vx + target_Vy + target_Vz*(Car_H/2+Car_W/2);
	target_motor[3]= target_Vx + target_Vy + target_Vz*(Car_H/2+Car_W/2);
}
//void Move_Transfrom(double Vx,double Vy,double Vz)//左移为正，前进为正
//{
//	PID_motor[0]=Vx-Vy-Vz*(Car_H/2+Car_W/2);
//	PID_motor[1]=Vx+Vy-Vz*(Car_H/2+Car_W/2);
//	PID_motor[2]=Vx+Vy+Vz*(Car_H/2+Car_W/2);
//	PID_motor[3]=Vx-Vy+Vz*(Car_H/2+Car_W/2);
//}
                    
/**************************************************************************
函数功能：增量式速度PI 计算出个轮子pid所需要的值
入口参数：PID_motor[4]   encoder[4]
返回  值：无 
**************************************************************************/
void Incremental_PI(void)
{
  static float PID_err[4], PID_last_err[4],PID_previous_Err[4];//PID_err数组存放本次误差，PID_last_err存放上次误差

  PID_err[0] = target_motor[0] - encoder[0]; //当前偏差值
  PID_err[1] = target_motor[1] - encoder[1]; //当前偏差值
  PID_err[2] = target_motor[2] - encoder[2]; //当前偏差值
  PID_err[3] = target_motor[3] - encoder[3]; //当前偏差值

  PID_motor[0] += Velocity_KP * (PID_err[0] - PID_last_err[0]) + Velocity_KI * PID_err[0]; //增量式PI控制器
  PID_motor[1] += Velocity_KP * (PID_err[1] - PID_last_err[1]) + Velocity_KI * PID_err[1]; //增量式PI控制器
  PID_motor[2] += Velocity_KP * (PID_err[2] - PID_last_err[2]) + Velocity_KI * PID_err[2]; //增量式PI控制器
  PID_motor[3] += Velocity_KP * (PID_err[3] - PID_last_err[3]) + Velocity_KI * PID_err[3]; //增量式PI控制器

  PID_last_err[0] = PID_err[0]; //更新上一次偏差
  PID_last_err[1] = PID_err[1]; //更新上一次偏差
  PID_last_err[2] = PID_err[2]; //更新上一次偏差
  PID_last_err[3] = PID_err[3]; //更新上一次偏差
}

/************************************电机驱动函数*************************/
void motor_control(void)
{
  int j;
  int Amplitude_motor = 20000; //===PWM满幅是50000 限制在50000 最高速度在20000左右

  for (j = 0; j < 4; j++) //限幅
  {
    if (PID_motor[j] > Amplitude_motor)
      PID_motor[j] = Amplitude_motor;
    if (PID_motor[j] < -Amplitude_motor)
      PID_motor[j] = -Amplitude_motor;
  }

  // DIR1改成了0，0改1

  if (PID_motor[0] > 0) //电机1   正转 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
  {
    gpio_set_level(DIR_LF, 0);                 // DIR输出高电平
    pwm_set_duty(motor_LF, (int)PID_motor[0]); // 计算占空比
  }
  else //电机1   反转
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-PID_motor[0]);
  }

  if (PID_motor[1] > 0) //电机2   正转
  {
    gpio_set_level(DIR_LB, 0);
    pwm_set_duty(motor_LB, (int)PID_motor[1]);
  }
  else //电机2   反转
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-PID_motor[1]);
  }

  if (PID_motor[2] > 0) //电机3   正转
  {
    gpio_set_level(DIR_RF, 1);
    pwm_set_duty(motor_RF, (int)PID_motor[2]);
  }
  else //电机3   反转
  {
    gpio_set_level(DIR_RF, 0);
    pwm_set_duty(motor_RF, (int)-PID_motor[2]);
  }

  if (PID_motor[3] > 0) //电机4   正转
  {
    gpio_set_level(DIR_RB, 1);
    pwm_set_duty(motor_RB, (int)PID_motor[3]);
  }
  else //电机4   反转
  {
    gpio_set_level(DIR_RB, 0);
    pwm_set_duty(motor_RB, (int)-PID_motor[3]);
  }
}

/***********************************************封装速度函数******************************************/
void Speed_Control(float Vx_Speed, float Vy_Speed, float Vz_Speed)
{
	Read_Encoder();
  Move_Transfrom(Vx_Speed, Vy_Speed, Vz_Speed);//有待测试
//  Car_Inverse_kinematics_solution(Vx_Speed, Vy_Speed, Vz_Speed);
  Incremental_PI();
  motor_control();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器速度     (速度 = ( (总脉冲数 / 编码器线数)  * 编码器齿数 / 车模齿数 ) * 轮周长 / 脉冲读取时间)
//  @param      encoder  编码器的值
//  @return     V_enco   该编码器的速度（mm/s）
//  @since      v1.0
//  Sample usage: ENCO_speed(master_encoder_left)
//-------------------------------------------------------------------------------------------------------------------
//17届麦轮编码器速度求位移
//AT_ITCM_SECTION_INIT(float ENCO_speed(int16 encoder))
//float ENCO_speed(int16 encoder)
//{
//		V_enco=(encoder*50*PI*11000/104/1024)*0.9f;//0.932f//*0.9f;//*0.9153f;//*0.9153f;  //  单位（mm/s）(11000=55/0.005) (encoder=61.634->车速1m/s)
//	V_enco=encoder*16.224796f;
//	return V_enco;
//}

///**************************************************************************
//函数功能：18届视觉读取编码器数值并计算车轮速度并求出位移，单位m/s
//(车轮速度 = ( (总脉冲数 / 编码器线数)  * 编码器齿数 / 车模齿数 ) * 轮周长 / 脉冲读取时间)
//入口参数：无
//返回值：无
//**************************************************************************/
//void Encoder_odometer(void)
//{
//  static float Angle_Bias = 0;//求角度位移？？？
//  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

//  Angle_Bias = (90 - Angle_Z) * PI / 180;

///****求编码器速度******/
//  V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637可以再精确多三位
//  V_enco[1] = 0.2636719 * PI * (float)encoder[1];
//  V_enco[2] = 0.2636719 * PI * (float)encoder[2];
//  V_enco[3] = 0.2636719 * PI * (float)encoder[3];
//  //下面的是原来的，没有负号
//  //    Vx_enco=(V_enco[0]-V_enco[1]-V_enco[2]+V_enco[3])/4;//右移为正
//  //    Vy_enco=(V_enco[0]+V_enco[1]+V_enco[2]+V_enco[3])/4;//前进为正
//  Vx_enco = -(V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //右移为正，这里回去看看车轮
//  Vy_enco = -(V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //前进为正
//  //    Car_dis_x+=Vx_enco*0.01;
//  //    Car_dis_y+=Vy_enco*0.01;

//#if 1
//  if (Angle_Bias >= 0)
//  {
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //分解到世界坐标上
//    Vx_world = Vx_2 - Vy_2;
//    Vy_world = Vx_1 + Vy_1;
//  }
//  if (Angle_Bias < 0)
//  {
//    Angle_Bias = -Angle_Bias;
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //分解到世界坐标上
//    Vx_world = Vx_2 + Vy_2;
//    Vy_world = -Vx_1 + Vy_1;
//  }
//#endif
//  Car_dis_x += Vx_world * 0.01;
//  Car_dis_y += Vy_world * 0.01;

//  Car_dis_x2 += Vx_world * 0.01;
//  Car_dis_y2 += Vy_world * 0.01;
//}
