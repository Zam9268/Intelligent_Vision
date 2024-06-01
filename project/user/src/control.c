#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#define CONTROL_FREQUENCY  100//编码器读取周期(0.01s 10ms)
#define Turn_limiting  40//转向速度输出限幅
#define Car_go         0 //寻迹
#define Car_find_card  1 //找卡片
#define Car_stop       2 //停车
float Car_H = 0.8;//车长
float Car_W = 0.6; // 车宽
float Vx,Vy,Vz;
float ahead_speed = 40.0;//直行速度
int encoder[4];   // 编码器数据
float correct_x_speed;//x轴上的修正速度
float correct_z_speed;//z轴上的修正速度
float correct_move_speed = 5;//x轴修正速度
float correct_turn_speed = 25;//x轴修正速度
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
float bili_act_turn = 1.4;//1.28
float loc_kp = 1.20;//位置式pd，方便调参使用 1.35位置式暂时最优 24/4/4     1.26
float loc_kd = 0.72;                         //0.80                     //0.72
int test_count=0;
float dt=0.005;
float turn_error = 2;//可接受的角度误差
float Turn_KP = 0.2;   //角度PID//
float Turn_KD = 0.0; //角度PID//
// float Turn_KI[1] = {30};  //角度PID//5
float final = 0.0F; //一阶低通滤波参数
float a = 0.25F;    //一阶低通滤波
float Vx_1, Vx_2, Vy_1, Vy_2;//对里程的cos，sin分解
float Vx_world, Vy_world;//世界坐标上的x，y
float Car_dis_x, Car_dis_y;//x轴，y轴行走距离
float Car_dis_x2, Car_dis_y2;
float Turn_Bias; 

pid_info Pos_turn_pid[4];//位置式pid

pid_info Speed[4]; // 增量式pid

pid_info Angle_turn_pid;

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
  encoder[1] = -encoder_get_count(ENCODER_LB); // 左后
  encoder[2] = encoder_get_count(ENCODER_RF); // 右前
  encoder[3] = encoder_get_count(ENCODER_RB); // 右后，正转读正
	
  for(uint8 i=0;i<4;i++)
  {
    Speed[i].now_speed = (encoder[i] * 0.2636719*PI); // 编码器数据转换成车轮速度，单位为cm/s
  } 
  
  // 编码器清空
  encoder_clear_count(ENCODER_LF);
  encoder_clear_count(ENCODER_LB);
  encoder_clear_count(ENCODER_RF);
  encoder_clear_count(ENCODER_RB);
}

/**
 * @brief 麦轮速度解算1
 * @param 顺时针为正
 * @return 无
 */
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = target_Vx + target_Vy - target_Vz; // 左前
  Speed[1].target_speed = -target_Vx + target_Vy - target_Vz; // 左后
  Speed[2].target_speed = -target_Vx + target_Vy + target_Vz; // 右前
  Speed[3].target_speed = target_Vx + target_Vy + target_Vz; // 右后
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
 * @brief 差速跑
 * @param 无
 * @return 无
 */
void car_run(float error)
{
	float move_error = (error/94)*(error/94); //横向比例系数,作归一化处理，94为188/2，半个屏幕的宽
  float turn_error = (error/94)*(error/94); //转向比例系数
  if(fabsf(error)<4)//误差很小时不作调整
  {
    correct_x_speed = 0;
    correct_z_speed = 0; 
  }
  if(fabsf(error)>4 && fabsd(error)<10)//误差不大的时候，认为它在直道上，仅做平移处理
  {
    for(int i=0; i<4; i++)
    {
      correct_x_speed = move_error * correct_move_speed;//平移的修正
      if(error>0)//在中线右侧
      {
        correct_x_speed = -correct_x_speed;//向左移动
      }
      else//在中线右侧
      {
        correct_x_speed = correct_x_speed;//向右移动
      }
    }
  }
  else if(fabsf(error)>10)//误差大，认为有转弯
  {
    for(int i=0; i<4; i++)
    {
      correct_z_speed = turn_error * correct_turn_speed;//转弯的修正
      if(error>0)//左转弯
      {
        correct_z_speed = -correct_z_speed;
      }
      else//右转弯
      {
        correct_z_speed = correct_z_speed;
      }
    }
  }

  Car_Inverse_kinematics_solution(correct_x_speed, ahead_speed, correct_z_speed);//速度解算赋值
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
  Pos_turn_pid[0].kp = loc_kp;   //0.5对应速度40   0.3//  3/30   1.0  24/4/4纯p
  Pos_turn_pid[0].kd = loc_kd;   //0.5对应速度40   0.8           
  //左后
  Pos_turn_pid[1].kp = loc_kp;
  Pos_turn_pid[1].kd = loc_kd;
  //右前
  Pos_turn_pid[2].kp = loc_kp;
  Pos_turn_pid[2].kd = loc_kd;
  //右后
  Pos_turn_pid[3].kp = loc_kp;
  Pos_turn_pid[3].kd = loc_kd; //PD赋值

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
  Speed[0].kp = -16.3;  //-16.3  -26
  Speed[0].ki = -3.3;  //-3.0 -1.80
  // ???
  Speed[1].kp = -14.0;    //-14.0 -34.5
  Speed[1].ki = -2.5;   //-2.5  -0.98
  // ???
  Speed[2].kp = -15.0;    //-15 -28.75
  Speed[2].ki = -3.0;   //-3.0   -0.6
  //???
  Speed[3].kp = -16.3;    //-16.0  -28.75
  Speed[3].ki = -2.8; //PI赋值 -2.8  -1.0
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
      //    
      Speed[i].error = Speed[i].target_speed - Speed[i].now_speed; //计算本次误差
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //增量式处理
		
			Speed[i].lastlastError = Speed[i].lastError;  //记录上上次误差
      Speed[i].lastError = Speed[i].error;          //记录上次误差
		
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //限幅
  }
}
/**
 * @brief 位置式pid(单环)
 * @param pid_info *pid:pid结构体pwm
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
	abs_loc_err = fabsf(Err_Handle())*bili_act_turn;   //Err_Handle()

   if(abs_loc_err < 2.0 )//设置中线绝对值阈值，小于这个值时，位置式不再起调整作用
  {
    loc_Finish_flag = 1;//位置式完成标志
    clear_encoder_sum();//清空编码器累计值
    int i = 0;
    for(i = 0;i < 4; i++) 
    {
			target_encoder_sum[i] = 0;//目标
      loc_target[i] = 0;//各个轮子的位置式输出速度归零
    }
  }
   
	if(loc_err > 0)       //右转弯识别出的误差是<0
     Turn_Left_flag = 1;//左转标志位
  if(loc_err < 0)
     Turn_Right_flag =1;//右转标志位

  if(loc_Finish_flag == 0)//位置式调整未完成
  {
    Set_Distence_m(abs_loc_err);//转换中线误差

    encoder_sum[0] += fabsf(encoder[0]);//编码器累加值
    encoder_sum[1] += fabsf(encoder[1]);
    encoder_sum[2] += fabsf(encoder[2]);
    encoder_sum[3] += fabsf(encoder[3]);
	
    LF_Target = Location_pid(&Pos_turn_pid[0], encoder_sum[0], target_encoder_sum[0]);
    LB_Target = Location_pid(&Pos_turn_pid[1], encoder_sum[1], target_encoder_sum[1]);
    RF_Target = Location_pid(&Pos_turn_pid[2], encoder_sum[2], target_encoder_sum[2]);
    RB_Target = Location_pid(&Pos_turn_pid[3], encoder_sum[3], target_encoder_sum[3]);//位置式处理，尝试给同一个速度
            
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
  else if(Turn_Right_flag==1)//右转，或者在中线
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

	for(uint8 i=0;i<4;i++)
	{
	loc_target[i] = PIDInfo_Limit(loc_target[i], 40.0);//输出速度限幅
	}
  }
}
/**
 * @brief 对pwm的一阶低通滤波
 * @param 无
 * @return 无
 */
float first_order_filter(float data)
	{
		final = a*data + (1-a)*final;    //两次数据乘上各自的权重
		return  (final);
 }
/**
 * @brief 串级pid 双环(位置环+速度环)
 * @param 无
 * @return 无
 */
void turnloc_pid(void)
{

  for(uint8 i=0;i<4;i++)
  {
      //速度环
      Speed[i].lastlastError = Speed[i].lastError;  //记录上上次输出
      Speed[i].lastError = Speed[i].error;          //记录上次输出
      Speed[i].error = Speed[i].target_speed + loc_target[i] - Speed[i].now_speed; //改变目标速度
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //输出pwm
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //限幅
  }

  Turn_Left_flag  = 0;
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
    else //反转
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
 * @brief 任意角度旋转(闭环)
 * @param 输入：Tar_angle_Z
 * @return 无
 * @attention 
 */
void Turn_Angle_PD(float Tar_angle_Z)
{
  static float Last_Turn_bias = 0, Last_last_Turn_bias = 0, Turn = 0;
  Turn_Bias = Tar_angle_Z - Angle_Z; //Angle_Z为当前角度偏差，由陀螺仪获取

  if (abs((int)Turn_Bias) < turn_error)//当前角度和目标角度相差绝对值在这个范围内是认为转向成功
  {
    Vz = 0;
  }
  else
  {
    Turn = Turn_KP * Turn_Bias + Turn_KD * (Turn_Bias - Last_Turn_bias);//原来是增量式处理，现在变更为位置式PD输出速度
    if (Turn > Turn_limiting)
      Turn = Turn_limiting;
    if (Turn < -Turn_limiting)
      Turn = -Turn_limiting;
    Vz = Turn;
    Last_Turn_bias = Turn_Bias;
    // Last_last_Turn_bias = Last_Turn_bias;
  }

  if ((abs((int)Turn_Bias) < turn_error + 2) && abs((int)Vz) < 10 && abs((int)Vz) > 0) //误差很小时的速度补偿,这里不确定要不要
  {
    if (Vz < 0)
      Vz -= 5;
    else if (Vz > 0)
      Vz += 8;
  }
  Car_Inverse_kinematics_solution(Vx, Vy, Vz);   //麦轮控制，为target_speed赋值
}
/**
 * @brief 里程计算距离
 * @param 
 * @return 
 * @attention 
 */
void Encoder_odometer(void)
{
  static float Angle_Bias = 0;
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  Angle_Bias = Angle_Z * PI / 180;//转换成弧度制，Angle_Z为转向角度

  V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637可以再精确多三位，计算车轮路程
  V_enco[1] = 0.2636719 * PI * (float)encoder[1];
  V_enco[2] = 0.2636719 * PI * (float)encoder[2];
  V_enco[3] = 0.2636719 * PI * (float)encoder[3];

  Vx_enco = (V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //前进为正，根据麦轮速度解算公式得出的底盘x轴位移量
  Vy_enco = (V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //左移为正，根据麦轮速度解算公式得出的底盘y轴位移量

#if 1
  if (Angle_Bias >= 0)//旋转角度(参照x轴)大于0时
  {
    Vx_1 = Vx_enco * sin(Angle_Bias);
    Vx_2 = Vx_enco * cos(Angle_Bias);
    Vy_1 = Vy_enco * cos(Angle_Bias);
    Vy_2 = Vy_enco * sin(Angle_Bias); //分解到世界坐标上
    Vx_world = Vx_2 - Vy_2;//简单的分解计算
    Vy_world = Vx_1 + Vy_1;
  }
  if (Angle_Bias < 0)
  {
    Angle_Bias = -Angle_Bias;
    Vx_1 = Vx_enco * sin(Angle_Bias);
    Vx_2 = Vx_enco * cos(Angle_Bias);
    Vy_1 = Vy_enco * cos(Angle_Bias);
    Vy_2 = Vy_enco * sin(Angle_Bias); //分解到世界坐标上
    Vx_world = Vx_2 + Vy_2;//简单的分解计算
    Vy_world = -Vx_1 + Vy_1;
  }
#endif
  Car_dis_x += Vx_world * 0.005;
  Car_dis_y += Vy_world * 0.005;

  Car_dis_x2 += Vx_world * 0.005;
  Car_dis_y2 += Vy_world * 0.005;
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