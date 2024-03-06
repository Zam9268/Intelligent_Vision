#include "control.h"
#include "imu660ra.h"
#include "zf_common_headfile.h"
#define Row   180 //148
#define Col   180
float Vx, Vy, Vz;//Vx为整车x轴方向速度，Vy为整车y轴方向速度，Vz是整车饶车中心旋转的速度
float target_motor[4];//四个轮子的目标pwm值
float Car_H = 0.8;//车身长度
float Car_W = 0.6;//车身宽度，单位均为m,可随意调整
float Velocity_KP = 0; //速度PID
float Velocity_KI = 0.1; //速度PID；这里还没有确定最终方案，先用最简单的来写
float turn_angle;//过弯转向角度
int spin;//过弯时的平均中线
int translation=0;//屏幕中线与赛道中线间的平行偏差量
int encoder[4];//存放编码器数值
float PID_motor[4];//存放pid输出后的数值
int midline[Row];//存放中线数组
float Keep_Bias;//车辆转向误差
pid_info LF_motor_pid;//电机pid
pid_info RF_motor_pid;
pid_info LB_motor_pid;
pid_info RB_motor_pid;


/*******************************************电机初始化*************************************/
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); //左前
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); //左后
  gpio_init(DIR_RF, GPO, GPIO_HIGH, GPO_PUSH_PULL); //右前
  gpio_init(DIR_RB, GPO, GPIO_HIGH, GPO_PUSH_PULL); //右后

  pwm_init(motor_LF, 15000, 0); //左前pwm
  pwm_init(motor_LB, 15000, 0); //左后pwm
  pwm_init(motor_RF, 15000, 0); //右前pwm
  pwm_init(motor_RB, 15000, 0); //右后pwm
}


/*******************************************编码器初始化***********************************/
void Encoder_Init(void)
{
  encoder_dir_init(ENCODER_LF, ENCODER_LF_LSB, ENCODER_LF_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_LB, ENCODER_LB_LSB, ENCODER_LB_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_RF, ENCODER_RF_LSB, ENCODER_RF_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_RB, ENCODER_RB_LSB, ENCODER_RB_DIR); // 初始化编码器模块与引脚 方向编码器模式
}


void Read_Encoder(void)
{
  encoder[0] = -encoder_get_count(ENCODER_LF);//获取编码器数值
  encoder[1] = -encoder_get_count(ENCODER_LB);
  encoder[2] = encoder_get_count(ENCODER_RF);
  encoder[3] = encoder_get_count(ENCODER_RB);

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
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz)
{
	target_motor[0]= target_Vx + target_Vy - target_Vz*(Car_H/2+Car_W/2);
	target_motor[1]= -target_Vx + target_Vy - target_Vz*(Car_H/2+Car_W/2);
	target_motor[2]= -target_Vx + target_Vy + target_Vz*(Car_H/2+Car_W/2);
	target_motor[3]= target_Vx + target_Vy + target_Vz*(Car_H/2+Car_W/2);//今晚在验证一下方向是否正确
}
                 
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
//pid参数初始化
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
//增量式PID
float increment_pid(float error,pid_info *pid)
{
	pid->error = error;
	pid->dError = error - pid->lastError;//本次误差与上次误差的偏差值
  pid->lastError = error;//记录下本次误差，以备下次使用
	pid->output +=(pid->kp*pid->dError)+(pid->ki*pid->error); //速度pi控制闭环
	pid->output_last = pid->output;//记录下本次的pid输出
	return pid->output;//返回增量式的计算值
}
//增量式PID
float PidIncCtrl(float error, pid_info *pid)
{
  
  pid->out_p = pid->kp * (error - pid->last_error);
  pid->out_i = pid->ki * error;
  
  pid->last_error = error;
  
  pid->out += pid->out_p + pid->out_i;
  
  return pid->out;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      PID赋值
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:pi调参
//-------------------------------------------------------------------------------------------------------------------
void PID_cale()
{
  PidInit(&LF_motor_pid);//初始化pid参数
	PidInit(&LB_motor_pid);
	PidInit(&RF_motor_pid);
	PidInit(&RB_motor_pid);
	
	LF_motor_pid.kp=Velocity_KP;
	LF_motor_pid.ki=Velocity_KI;
	
	LB_motor_pid.kp=Velocity_KP;
	LB_motor_pid.ki=Velocity_KI;
	
	RF_motor_pid.kp=Velocity_KP;
	RF_motor_pid.ki=Velocity_KI;
	
	RB_motor_pid.kp=Velocity_KP;
	RB_motor_pid.ki=Velocity_KI;//各个轮子pi赋值

  static float PID_err[4];
  PID_err[0] = target_motor[0] - encoder[0]; //当前偏差值
  PID_err[1] = target_motor[1] - encoder[1]; //当前偏差值
  PID_err[2] = target_motor[2] - encoder[2]; //当前偏差值
  PID_err[3] = target_motor[3] - encoder[3]; //当前偏差值

	PID_motor[0] = increment_pid(PID_err[0], &LF_motor_pid);
  PID_motor[1] = increment_pid(PID_err[1], &LB_motor_pid);
  PID_motor[2] = increment_pid(PID_err[2], &RF_motor_pid);
  PID_motor[3] = increment_pid(PID_err[3], &RB_motor_pid);//计算四个轮子速度闭环后输出的pwm
}


/************************************电机闭环驱动函数*************************/
void motor_close_control(void)
{
  int j;
  int Amplitude_motor = 3000; //===PWM满幅是50000 限制在50000 最高速度在20000左右

  for (j = 0; j < 4; j++) //限幅
  {
    if (PID_motor[j] > Amplitude_motor)
      PID_motor[j] = Amplitude_motor;
    if (PID_motor[j] < -Amplitude_motor)
      PID_motor[j] = -Amplitude_motor;
  }

  // DIR1改成了0，0改1

  if (target_motor[0] > 0) //电机1   正转 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
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
    gpio_set_level(DIR_RF, 0);//可能是这里出现问题，此处DIR可以更改？？
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
/************************************电机开环驱动函数*************************/
void motor_control(void)
{
  int j;
  int Amplitude_motor = 20000; //===PWM满幅是50000 限制在50000 最高速度在20000左右

  for (j = 0; j < 4; j++) //限幅
  {
    if (target_motor[j] > Amplitude_motor)
      target_motor[j] = Amplitude_motor;
    if (target_motor[j] < -Amplitude_motor)
      target_motor[j] = -Amplitude_motor;
  }

  // DIR1改成了0，0改1

  if (target_motor[0] > 0) //电机1   正转 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
  {
    gpio_set_level(DIR_LF, 0);                 // DIR输出高电平
    pwm_set_duty(motor_LF, (int)target_motor[0]); // 计算占空比
  }
  else //电机1   反转
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-target_motor[0]);
  }

  if (PID_motor[1] > 0) //电机2   正转
  {
    gpio_set_level(DIR_LB, 0);
    pwm_set_duty(motor_LB, (int)target_motor[1]);
  }
  else //电机2   反转
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-target_motor[1]);
  }

  if (PID_motor[2] > 0) //电机3   正转
  {
    gpio_set_level(DIR_RF, 1);
    pwm_set_duty(motor_RF, (int)target_motor[2]);
  }
  else //电机3   反转
  {
    gpio_set_level(DIR_RF, 0);
    pwm_set_duty(motor_RF, (int)-target_motor[2]);
  }

  if (PID_motor[3] > 0) //电机4   正转
  {
    gpio_set_level(DIR_RB, 1);
    pwm_set_duty(motor_RB, (int)target_motor[3]);
  }
  else //电机4   反转
  {
    gpio_set_level(DIR_RB, 0);
    pwm_set_duty(motor_RB, (int)-target_motor[3]);
  }
}

/***********************************************封装速度函数******************************************/
void Speed_Control(float Vx_Speed, float Vy_Speed, float Vz_Speed)
{
// 	  Car_Inverse_kinematics_solution(Vx_Speed, Vy_Speed, Vz_Speed);
//     Move_Transfrom(Vx_Speed, Vy_Speed, Vz_Speed);//通过测试
//     motor_control();
 Car_Inverse_kinematics_solution(Vx_Speed, Vy_Speed, Vz_Speed);
 PID_cale();//PID闭环后赋值给电机
motor_close_control();//闭环电机测试
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取过弯转向的转向速度及角度
//  @param      mode (1为前后移动，2为转动)，value为转向角度（通过对赛道中线对比获取），turn_speed为转向速度(target_Vx)
//  @return     void
//  @since      v1.0
//  Sample usage: value为转向角度（通过对赛道中线对比获取），turn_speed为转向速度
//-------------------------------------------------------------------------------------------------------------------
void move(int8 mode,float value,int16 stra_speed,int16 turn_speed)
{
  int k = 0;
  translation=midline[Row-1]-Col/2; //计算平移量小心数组越界
  for(k = Row; k < 65; k--)
  {
     spin=spin+midline[k];//对过弯时的中线行数做累加
     spin=spin/(Row-k)-Col/2;//求平均后减去屏幕中线求出偏差值
  }
}


///**************************************************************************
// 函数功能：18届视觉读取编码器数值并计算车轮速度并求出位移，单位m/s
// (车轮速度 = ( (总脉冲数 / 编码器线数)  * 编码器齿数 / 车模齿数 ) * 轮周长 / 脉冲读取时间)
// 入口参数：无
// 返回值：无
// **************************************************************************/
void Encoder_odometer(void)
{
 static float Angle_Bias = 0;//角度误差
 static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

 Angle_Bias = (90 - Angle_Z) * PI / 180;//弧度制，车身姿态发生偏转

/****求编码器速度******/
 V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637可以再精确多三位
 V_enco[1] = 0.2636719 * PI * (float)encoder[1];
 V_enco[2] = 0.2636719 * PI * (float)encoder[2];
 V_enco[3] = 0.2636719 * PI * (float)encoder[3];
 //下面的是原来的，没有负号
 //    Vx_enco=(V_enco[0]-V_enco[1]-V_enco[2]+V_enco[3])/4;//右移为正
 //    Vy_enco=(V_enco[0]+V_enco[1]+V_enco[2]+V_enco[3])/4;//前进为正
 Vx_enco = -(V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //右移为正，这里回去看看车轮
 Vy_enco = -(V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //前进为正
 //    Car_dis_x+=Vx_enco*0.01;
 //    Car_dis_y+=Vy_enco*0.01;

// #if 1
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
// #endif
//  Car_dis_x += Vx_world * 0.01;
//  Car_dis_y += Vy_world * 0.01;

//  Car_dis_x2 += Vx_world * 0.01;
//  Car_dis_y2 += Vy_world * 0.01;
}
