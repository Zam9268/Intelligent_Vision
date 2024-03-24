
#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#define Row   180 //148
#define Col   180
float Vx, Vy, Vz;//麦轮速度解算得出的坐标
float target_motor[4];//目标电机的pwm
float Car_H = 0.8;//车长
float Car_W = 0.6; // 车宽，单位：m，用于计算转向角度
float Velocity_KP = 0; // 速度PID的比例系数
float Velocity_KI = 0; // 速度PID的积分系数，用于调节速度PID的响应速度
float turn_angle; // 转向角度
int spin; // 旋转方向
int translation = 0; // 平移方向，0表示不平移
int encoder[4]; // 编码器示数
int pid_motor[4]; // PID控制后的电机输出
int midline[Row]; // 中线位置数组
float PID_Bias[4]={0.0}, PID_Last_bias[4]={0.0};
float Keep_Bias; // 保持偏差
int test_count=0;
float dt=0.005;
pid_info LF_motor_pid; // 左前电机PID
pid_info RF_motor_pid; // 右前电机PID
pid_info LB_motor_pid; // 左后电机PID
pid_info RB_motor_pid; // 右后电机PID

pid_info Pos_turn_pid[4];//位置式处理用的pid

pid_info Speed[4]; // 速度信息数组，0表示左前电机，1表示左后电机，2表示右前电机，3表示右后电机

/**
 * @brief 电机初始化函数
 * @param 无
 * @return 无
 */
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化左前电机方向引脚
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化左后电机方向引脚
  gpio_init(DIR_RF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化右前电机方向引脚
  gpio_init(DIR_RB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化右后电机方向引脚

  pwm_init(motor_LF, 15000, 0); // 初始化左前电机的PWM
  pwm_init(motor_LB, 15000, 0); // 初始化左后电机的PWM
  pwm_init(motor_RF, 15000, 0); // 初始化右前电机的PWM
  pwm_init(motor_RB, 15000, 0); // 初始化右后电机的PWM

  
}


/**
 * @brief 编码器初始化函数
 * @param 无
 * @return 无?
 */
void Encoder_Init(void)
{
  encoder_dir_init(ENCODER_LF, ENCODER_LF_LSB, ENCODER_LF_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_LB, ENCODER_LB_LSB, ENCODER_LB_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_RF, ENCODER_RF_LSB, ENCODER_RF_DIR); // 初始化编码器模块与引脚 方向编码器模式
  encoder_dir_init(ENCODER_RB, ENCODER_RB_LSB, ENCODER_RB_DIR); // 初始化编码器模块与引脚 方向编码器模式

  for(uint8 i=0;i<4;i++)
  {
    encoder[i]=0;//编码器的示数清零
  }
}

/**
 * @brief 读取编码器示数
 * @param 无
 * @return 无
 */
void Read_Encoder(void)
{
  // 读取编码器计数值
  encoder[0] = -encoder_get_count(ENCODER_LF); // 左前编码器计数值
  encoder[1] = encoder_get_count(ENCODER_LB); // 左后编码器计数值
  encoder[2] = encoder_get_count(ENCODER_RF); // 右前编码器计数值
  encoder[3] = -encoder_get_count(ENCODER_RB); // 右后编码器计数值

  // 计算对应轮子的速度
  for(uint8 i=0;i<4;i++)
  {
    Speed[i].now_speed = (encoder[i] * 0.2636719*PI); // 0.2637为编码器的转速比例系数，求出来的速度单位是cm/s
    Speed[i].delta_speed = Speed[i].now_speed - Speed[i].last_speed;//算出速度差值
    Speed[i].last_speed = Speed[i].now_speed;
  } 
  
  // 清空编码器计数
  encoder_clear_count(ENCODER_LF);
  encoder_clear_count(ENCODER_LB);
  encoder_clear_count(ENCODER_RF);
  encoder_clear_count(ENCODER_RB);
}

/**
 * @brief 麦轮解算
 * @param 无
 * @return 无
 */
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = -(+target_Vx + target_Vy + target_Vz); // 左前电机目标速度
  Speed[1].target_speed = -(-target_Vx + target_Vy + target_Vz); // 左后电机目标速度
  Speed[2].target_speed = -(-target_Vx + target_Vy - target_Vz); // 右前电机目标速度
  Speed[3].target_speed = -(+target_Vx + target_Vy - target_Vz); // 右后电机目标速度
}

/**
 * @brief 移动转换函数，将目标速度转换为电机速度
 * @param target_Vx 目标X轴速度
 * @param target_Vy 目标Y轴速度
 * @param target_Vz 目标Z轴速度
 * @return 无
 * @attention // 注意：x轴为正前方，y轴为正左方，z轴为正上方
 */
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); // 左前电机目标速度
  Speed[1].target_speed = -target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); // 左后电机目标速度
  Speed[2].target_speed = -target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); // 右前电机目标速度
  Speed[3].target_speed = target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); // 右后电机目标速度
}
         
/**
 * @brief 位置式Pid初始化（不用放在中断，只需要初始化一次即可）
 * @param 无
 * @return 无
 */
void Pos_PidInit(void)
{
  for(uint8 i=0;i<4;i++)
  {
    Pos_turn_pid[i].target_speed = 0.00;
    Pos_turn_pid[i].target_pwm = 0;
    Pos_turn_pid[i].kd= 0.00;
    Pos_turn_pid[i].ki        = 0.00;
    Pos_turn_pid[i].kd        = 0.00;
    Pos_turn_pid[i].error     = 0.00;
    Pos_turn_pid[i].lastError = 0.00;
    Pos_turn_pid[i].dError    = 0.00;
    Pos_turn_pid[i].output    = 0.00;
    Pos_turn_pid[i].output_last   = 0.00;
    Pos_turn_pid[i].xuhao=i; //序号赋值
  }

  // 设置PI参数(还没调)
  Pos_turn_pid[0].kp = -22.5;
  Pos_turn_pid[0].ki = -1.50;
  // 设置PI参数
  Pos_turn_pid[1].kp = -30;
  Pos_turn_pid[1].ki = -0.5;
  // 设置PI参数
  Pos_turn_pid[2].kp = -25;
  Pos_turn_pid[2].ki = -0.5;
  // 设置PI参数
  Pos_turn_pid[3].kp = -25;
  Pos_turn_pid[3].ki = -0.8; // 设置pi参数

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
    Speed[i].xuhao=i; //序号赋值
  }

  // 设置PI参数
  Speed[0].kp = -22.5;
  Speed[0].ki = -1.50;
  // 设置PI参数
  Speed[1].kp = -30;
  Speed[1].ki = -0.5;
  // 设置PI参数
  Speed[2].kp = -25;
  Speed[2].ki = -0.5;
  // 设置PI参数
  Speed[3].kp = -25;
  Speed[3].ki = -0.8; // 设置pi参数

}
/**
 * @brief 实现PID控制器
 * @param pid_info *pid 控制器参数结构体
 * @return 控制器输出值
 */
void increment_pid(void)
{
  for(uint8 i=0;i<4;i++)
  {
      //更新误差
      Speed[i].lastlastError = Speed[i].lastError;  //误差更新为上上次的误差
      Speed[i].lastError = Speed[i].error;          //误差更新为上次的误差
      Speed[i].error = Speed[i].target_speed - Speed[i].now_speed; //计算当前pwm和编码器值的误差
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //增量式PI控制器
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //限幅
  }
}
/**********************************************************************************************************
*	函 数 名：qianzhan_2()
*	功能说明：获取前瞻
*   输    入：无
*   输    出：20行的平均前瞻
*   备    注：左转弯前瞻为负，右转弯前瞻为正，最好取前一点
**********************************************************************************************************/ 
int qianzhan_2()
{
	uint8_t i;
	int sum=0;
	
	for(i=0;i<20;i++)
	{
	sum+=(center[IMAGE_HEIGHT/2+15-i]-IMAGE_WIDTH/2); //获取第55-75行的前瞻
	}	
	return (sum/20);//求平均值
}
/**********************************************************************************************************
*	函 数 名：Position_PID
*	功能说明：位置式PID控制
*   输    入：与标准中线的误差值,图像处理后的
*   输    出：PID控制值，直接赋值给执行函数
**********************************************************************************************************/ 
float Position_PID(pid_info *pid, float err)
{
 
    float  iError,     //当前误差
           pwm_output;
 
    iError = err;                   //计算当前误差
			
    pid->output = pid->kp * iError                        //比例P            
           + pid->kd * (iError - pid->lastError);   //微分D
	
	pid->lastError = iError;		  	                     //更新上次误差，用于下次计算
  pwm_output =(pid->output - pid->output_last)/dt;
  pid->output_last=pid->output;                        //记录本次输出值 
	return pwm_output;	//返回控制输出值
}

/**********************************************************************************************************
*	函 数 名：turnpos_pid
*	功能说明：差速控制(实现寻迹)
*   输    入：位置式输出的pwm
*   输    出：无
**********************************************************************************************************/ 
void turnpos_pid(void)
{
  int err = qianzhan_2();
  for(uint8 i=0;i<4;i++)
  {
      //更新误差
      Speed[i].lastlastError = Speed[i].lastError;  //误差更新为上上次的误差
      Speed[i].lastError = Speed[i].error;          //误差更新为上次的误差
      //尝试加入编码器处理后得出的速度，改变目标速度来实现闭环，当增量式输出减小时，所读出来的编码器速度增加量会趋向于0
      Speed[i].error = Speed[i].target_speed + Speed[i].delta_speed - Speed[i].now_speed; 
      if(i<2)
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error - Position_PID( &Pos_turn_pid[i], err); //处理后的pwm直接放入
      else
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error + Position_PID( &Pos_turn_pid[i], err); 
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //限幅
  }
}
/**
 * @brief 目标电机速度计算函数
 * @param 无
 * @return 无
 * @attention 根据目标速度计算PID_motor[i]，并输出到pwm控制电机
 *            对应的引脚进行修改，方便后续对应：
 */
void motor_close_control(void)
{
  int j;
  for (j = 0; j < 4; j++) //???
  {
    pid_motor[j]=Speed[j].output;//pid输出赋值，其实这里就没必要限幅了
    // Speed[j].output=0;
    if (pid_motor[j] > AMPLITUDE_MOTOR)
      pid_motor[j] = AMPLITUDE_MOTOR;
    if (pid_motor[j] < -AMPLITUDE_MOTOR)
      pid_motor[j] = -AMPLITUDE_MOTOR;
  }
    if (pid_motor[0] > 0) //坐下
    {
      gpio_set_level(DIR_LF, 0);                 // DIR设置为正转
      pwm_set_duty(motor_LF, (int)pid_motor[0]); // 设置PWM占空比
    }
    else //反转
    {
      gpio_set_level(DIR_LF, 1);
      pwm_set_duty(motor_LF, (int)-pid_motor[0]);
    }

    if (pid_motor[1] > 0) //左下
    {
      gpio_set_level(DIR_LB, 0);
      pwm_set_duty(motor_LB, (int)pid_motor[1]);
    }
    else //反转
    {
      gpio_set_level(DIR_LB, 1);
      pwm_set_duty(motor_LB, (int)-pid_motor[1]);
    }

    if (pid_motor[2] > 0) //右上
    {
      gpio_set_level(DIR_RF, 1);
      pwm_set_duty(motor_RF, (int)pid_motor[2]);
    }
    else //反转
    {
      gpio_set_level(DIR_RF, 0); //设置为反转
      pwm_set_duty(motor_RF, (int)-pid_motor[2]);
    }

    if (pid_motor[3] > 0) //右下
    {
      gpio_set_level(DIR_RB, 1);
      pwm_set_duty(motor_RB, (int)pid_motor[3]);
    }
    else //反转
    {
      gpio_set_level(DIR_RB, 0);
      pwm_set_duty(motor_RB, (int)-pid_motor[3]);
    }
  }
/**
 * @brief 目标电机控制函数
 * @param 无
 * @return 无
 * @attention 根据目标电机速度计算PID_motor[i]，并输出到pwm控制电机
 */
void motor_control(void)
{
  for (int j= 0; j < 4; j++) //循环四次
  {
    if (pid_motor[j] > AMPLITUDE_MOTOR)
      pid_motor[j] = AMPLITUDE_MOTOR;
    if (pid_motor[j] < -AMPLITUDE_MOTOR)
      pid_motor[j] = -AMPLITUDE_MOTOR;
  }
  if (pid_motor[0] > 0) //电机1   正转
  {
    gpio_set_level(DIR_LF, 0);                 // DIR设置为正转
    pwm_set_duty(motor_LF, (int)pid_motor[0]); // 设置PWM占空比
  }
  else //电机1   反转
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-pid_motor[0]);
  }

  if (pid_motor[1] > 0) //电机2   正转
  {
    gpio_set_level(DIR_LB, 0);
    pwm_set_duty(motor_LB, (int)pid_motor[1]);
  }
  else //电机2   反转
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-pid_motor[1]);
  }

  if (pid_motor[2] > 0) //电机3   正转
  {
    gpio_set_level(DIR_RF, 1);
    pwm_set_duty(motor_RF, (int)pid_motor[2]);
  }
  else //电机3   反转
  {
    gpio_set_level(DIR_RF, 0);
    pwm_set_duty(motor_RF, (int)-pid_motor[2]);
  }

  if (pid_motor[3] > 0) //电机4   正转
  {
    gpio_set_level(DIR_RB, 1);
    pwm_set_duty(motor_RB, (int)pid_motor[3]);
  }
  else //电机4   反转
  {
    gpio_set_level(DIR_RB, 0);
    pwm_set_duty(motor_RB, (int)-pid_motor[3]);
  }
}

/**
 * @brief 目标速度控制函数
 * @param Vx_Speed X轴速度
 * @param Vy_Speed Y轴速度
 * @param Vz_Speed Z轴速度
 * @return 无
 * @attention 根据目标速度计算PID_motor[i]，并输出到pwm控制电机
 */
void Speed_Control(float Vx_Speed, float Vy_Speed, float Vz_Speed)
{
// 	  Car_Inverse_kinematics_solution(Vx_Speed, Vy_Speed, Vz_Speed);
  Move_Transfrom(Vx_Speed, Vy_Speed, Vz_Speed);//移动变换
//     motor_control();
  Car_Inverse_kinematics_solution(Vx_Speed, Vy_Speed, Vz_Speed);
  // motor_close_control();//电机闭环控制
}


/**
 * @brief 移动函数，用于控制车辆的移动
 * @param mode 1代表直行，2代表转弯
 * @param value 目标速度
 * @param stra_speed 直行速度
 * @param turn_speed 转弯速度
 * @return 无
 */
void move(int8 mode, float value, int16 stra_speed, int16 turn_speed)
{
  int k = 0;
  translation = midline[Row - 1] - Col / 2; //计算偏移量
  for (k = Row; k < 65; k--)
  {
    spin = spin + midline[k]; //计算旋转角度之和
    spin = spin / (Row - k) - Col / 2; //计算平均旋转角度
  }
}

/**
 * @brief 18电机编码器里程计算函数，根据编码器脉冲数计算车辆行驶距离，单位：m
 * @param 无
 * @return 无
 * @attention 行驶距离计算公式 = (编码器脉冲数 / 编码器分辨率) * (2 * π * 轮子半径) / 轮子周长
 */
void Encoder_odometer(void)
{
  static float Angle_Bias = 0; //角度偏差
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  // Angle_Bias = (90 - Angle_Z) * PI / 180; //计算角度偏差

/****计算车辆速度******/
  // V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637为编码器分辨率
  // V_enco[1] = 0.2636719 * PI * (float)encoder[1];
  // V_enco[2] = 0.2636719 * PI * (float)encoder[2];
  // V_enco[3] = 0.2636719 * PI * (float)encoder[3];
  // Vx_enco=(V_enco[0]-V_enco[1]-V_enco[2]+V_enco[3])/4; //计算X轴速度
  // Vy_enco=(V_enco[0]+V_enco[1]+V_enco[2]+V_enco[3])/4; //计算Y轴速度
  // Vx_enco = -(V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //计算X轴速度（修正）
  // Vy_enco = -(V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //计算Y轴速度（修正）
  // Car_dis_x += Vx_enco * 0.01; //计算车辆X轴行驶距离
  // Car_dis_y += Vy_enco * 0.01; //计算车辆Y轴行驶距离

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
//  Car_dis_x += Vx_world * 0.01; //计算车辆X轴行驶距离（修正）
//  Car_dis_y += Vy_world * 0.01; //计算车辆Y轴行驶距离（修正）

//  Car_dis_x2 += Vx_world * 0.01;
//  Car_dis_y2 += Vy_world * 0.01;
}

/**
 * @brief PID限幅函数
 *
 * @param Value 输入值
 * @param MaxValue 最大值
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
