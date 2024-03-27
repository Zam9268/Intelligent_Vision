#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#define Row   180 //148
#define Col   180
#define CONTROL_FREQUENCY  100//编码器读取频率(即周期为0.01s 10ms)
float Vx, Vy, Vz;//三轴目标速度
float target_motor[4];//目标pwm
float Car_H = 0.8;//车长
float Car_W = 0.6; // 车宽
float turn_angle; // 转向角度
int spin; // 旋转量
int translation = 0; // 横移量
int encoder[4];   // 四个编码器读出的值
int encoder_sum[4];//四个编码器的累积数值
float target_encoder_sum[4];//四个编码器的目标累计数值
float loc_target[4];//位置式处理后输出的目标速度
int Turn_Left_flag,Turn_Right_flag;//左转标志，右转标志
int loc_Finish_flag = 0;//位置式调整完成标志
int pid_motor[4]; // PID输出的pwm
int midline[Row]; // 中线位置数组
float err_mid;//中线偏移误差
float PID_Bias[4]={0.0}, PID_Last_bias[4]={0.0};
float Keep_Bias; //车头角度误差
int test_count=0;
float dt=0.005;

pid_info Pos_turn_pid[4];//位置式处理后的pid

pid_info Speed[4]; // 速度环pid

/**
 * @brief 电机初始化
 * @param 无
 * @return无
 */
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // gpio口给高电平
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 
  gpio_init(DIR_RF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 
  gpio_init(DIR_RB, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 

  pwm_init(motor_LF, 15000, 0); // PWM通道初始化
  pwm_init(motor_LB, 15000, 0); //
  pwm_init(motor_RF, 15000, 0); // 
  pwm_init(motor_RB, 15000, 0); // 

  
}


/**
 * @brief 编码器初始化
 * @param 鏃?
 * @return 鏃??
 */
void Encoder_Init(void)
{
  encoder_dir_init(ENCODER_LF, ENCODER_LF_LSB, ENCODER_LF_DIR); // 
  encoder_dir_init(ENCODER_LB, ENCODER_LB_LSB, ENCODER_LB_DIR); // 
  encoder_dir_init(ENCODER_RF, ENCODER_RF_LSB, ENCODER_RF_DIR); // 
  encoder_dir_init(ENCODER_RB, ENCODER_RB_LSB, ENCODER_RB_DIR); // 

  for(uint8 i=0;i<4;i++)
  {
    encoder[i]=0;//清空编码器计数
  }
}

/**
 * @brief 获取编码器数值
 * @param 无
 * @return 无
 */
void Read_Encoder(void)
{
  // 读编码器
  encoder[0] = -encoder_get_count(ENCODER_LF); // 左前
  encoder[1] = encoder_get_count(ENCODER_LB); // 左后
  encoder[2] = encoder_get_count(ENCODER_RF); // 右前
  encoder[3] = -encoder_get_count(ENCODER_RB); // 右后

  // 计算车轮实际速度
  for(uint8 i=0;i<4;i++)
  {
    Speed[i].now_speed = (encoder[i] * 0.2636719*PI); // 0.2637获取车轮速度，单位为m/s
  } 
  
  // 清空编码器计数
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
  Speed[0].target_speed = -(+target_Vx + target_Vy + target_Vz); // 左前轮
  Speed[1].target_speed = -(-target_Vx + target_Vy + target_Vz); // 左后
  Speed[2].target_speed = -(-target_Vx + target_Vy - target_Vz); // 右前
  Speed[3].target_speed = -(+target_Vx + target_Vy - target_Vz); // 右后
}

/**
 * @brief 麦轮速度解算2
 * @param target_Vx x轴目标速度
 * @param target_Vy y轴目标速度
 * @param target_Vz 自旋速度
 * @return 无
 * @attention //
 */
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz)
{
  Speed[0].target_speed = target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); // 左前
  Speed[1].target_speed = -target_Vx + target_Vy - target_Vz * (Car_H/2 + Car_W/2); // 左后
  Speed[2].target_speed = -target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); // 右前
  Speed[3].target_speed = target_Vx + target_Vy + target_Vz * (Car_H/2 + Car_W/2); // 右后
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
    Pos_turn_pid[i].xuhao=i; //编码器序号
  }

  //左前
  Pos_turn_pid[0].kp = 0.0;
  Pos_turn_pid[0].kd = 0.0;
  //左后
  Pos_turn_pid[1].kp = 0.0;
  Pos_turn_pid[1].kd = 0.0;
  //右前
  Pos_turn_pid[2].kp = 0.0;
  Pos_turn_pid[2].kd = 0.0;
  //右后
  Pos_turn_pid[3].kp = 0.0;
  Pos_turn_pid[3].kd = 0.0; //各个轮子分开调参

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
    Speed[i].xuhao=i; //编码器序号
  }

  // 左前
  Speed[0].kp = -22.5;
  Speed[0].ki = -1.50;
  // 左后
  Speed[1].kp = -30;
  Speed[1].ki = -0.5;
  // 右前
  Speed[2].kp = -25;
  Speed[2].ki = -0.5;
  //右后
  Speed[3].kp = -25;
  Speed[3].ki = -0.8; ////各个轮子分开调参 

}
/**
 * @brief 增量式速度环(内环)
 * @param pid_info *pid 
 * @return 各个轮子的pwm
 */
void increment_pid(void)
{
  for(uint8 i=0;i<4;i++)
  {
      //速度环
      Speed[i].lastlastError = Speed[i].lastError;  //记录上上次误差
      Speed[i].lastError = Speed[i].error;          //记录上次误差
      Speed[i].error = Speed[i].target_speed - Speed[i].now_speed; //本次误差
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //增量式处理
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //限幅
  }
}
/**
 * @brief 位置式pid处理
 * @param 输入：pid_info *pid四个轮子的pwm值 Target：目标距离的处理后的编码器总累计值 encoder：编码器累计所读的值
 * @return 输出的是速度pid->output，准备输入至速度环闭环
 */
int Location_pid(pid_info *pid, float Encoder, float Target)
{
    static float Bias,Speed;

    Bias= Target - Encoder; //Calculate the deviation //计算偏差
    
    pid->output = pid->kp*Bias + pid->kd * (Bias-pid->lastError); //原来是+=，现改为=      2024/3/26
    
    pid->lastError=Bias;//保存上次偏差
    return pid->output;
}
/**
 * @brief 清空编码器累计值
 * @param 无
 * @return 无
 */
void clear_encoder_sum(void)
{
  encoder_sum[0] = 0;//编码器累计值
  encoder_sum[1] = 0;
  encoder_sum[2] = 0;
  encoder_sum[3] = 0;
}

void Set_Distence_m(uint16_t distance)
{
  clear_encoder_sum();    

  target_encoder_sum[0] = (distance/PI) * 100/0.2636719;//这里将目标距离转换成脉冲累计
  target_encoder_sum[1] = target_encoder_sum[0];
  target_encoder_sum[2] = target_encoder_sum[0];
  target_encoder_sum[3] = target_encoder_sum[0];
}

/**************************************************************************
位置环输出速度
**************************************************************************/
void Drive_Motor()
{
  float LF_Target,LB_Target,RF_Target,RB_Target;
  //这里是算出来的脉冲 要传入速度环还需将位置环计算出来的脉冲转换为速度
  encoder_sum[0] += encoder[0];//编码器累计值
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

  if(Turn_Left_flag==1)//左转
  {
    loc_target[0]=-fabsf(loc_target[0]);
    loc_target[1]=-fabsf(loc_target[1]);
    loc_target[2]=fabsf(loc_target[2]);
    loc_target[3]=fabsf(loc_target[3]);
  }
  else if(Turn_Right_flag==1)//右转
  {
    loc_target[0]=fabsf(loc_target[0]);
    loc_target[1]=fabsf(loc_target[1]);
    loc_target[2]=-fabsf(loc_target[2]);
    loc_target[3]=-fabsf(loc_target[3]);
  }
}
void turnloc_pid(void)
{
  if(Err_Handle()<2)//设定一个阈值范围，小于这个阈值时视为不再需要矫正,只用速度环做目标速度闭环
  {
    loc_Finish_flag = 1;//位置调整完成标志
    clear_encoder_sum();//编码器累加值清0
  }
  if(loc_Finish_flag == 0)//位置调整未完成
  {
  Set_Distence_m(Err_Handle());//输入速度转换成误差,还要调整一下参数
  if(Err_Handle() < 0)
     Turn_Left_flag = 1;
  if(Err_Handle() > 0)
     Turn_Right_flag =1;
  Drive_Motor();//位置环输出速度

  for(uint8 i=0;i<4;i++)
  {
      //输入速度环处理
      Speed[i].lastlastError = Speed[i].lastError;  //记录上上次误差
      Speed[i].lastError = Speed[i].error;          //记录上次误差
      Speed[i].error = Speed[i].target_speed + loc_target[i] - Speed[i].now_speed; //本次误差,可能会不加原本的目标速度
      Speed[i].output += Speed[i].kp*(Speed[i].error-Speed[i].lastError)+Speed[i].ki*Speed[i].error; //增量式处理输出pwm
      Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); //限幅
  }
  Turn_Left_flag = 0;
  Turn_Right_flag = 0;//标志位清零
  }
}
/**
 * @brief 速度闭环控制
 * @param 无
 * @return 无
 * @attention 
 */
void motor_close_control(void)
{
  int j;
  for (j = 0; j < 4; j++) //???
  {
    pid_motor[j]=Speed[j].output;//各轮子pwm赋值
    // Speed[j].output=0;
  }
    if (pid_motor[0] > 0) //正转
    {
      gpio_set_level(DIR_LF, 0);                 // DIR0
      pwm_set_duty(motor_LF, (int)pid_motor[0]); // 左前
    }
    else //鍙嶈浆
    {
      gpio_set_level(DIR_LF, 1);
      pwm_set_duty(motor_LF, (int)-pid_motor[0]);
    }

    if (pid_motor[1] > 0) //正转
    {
      gpio_set_level(DIR_LB, 0);
      pwm_set_duty(motor_LB, (int)pid_motor[1]);
    }
    else //反转
    {
      gpio_set_level(DIR_LB, 1);
      pwm_set_duty(motor_LB, (int)-pid_motor[1]);
    }

    if (pid_motor[2] > 0) //正转
    {
      gpio_set_level(DIR_RF, 1);
      pwm_set_duty(motor_RF, (int)pid_motor[2]);
    }
    else //鍙嶈浆
    {
      gpio_set_level(DIR_RF, 0); //反转
      pwm_set_duty(motor_RF, (int)-pid_motor[2]);
    }

    if (pid_motor[3] > 0) //正转
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
 * @brief 电机开环控制
 * @param 无
 * @return 无
 * @attention 
 */
void motor_control(void)
{
  for (int j= 0; j < 4; j++) //电机pwm赋值
  {
    if (pid_motor[j] > AMPLITUDE_MOTOR)
      pid_motor[j] = AMPLITUDE_MOTOR;
    if (pid_motor[j] < -AMPLITUDE_MOTOR)
      pid_motor[j] = -AMPLITUDE_MOTOR;
  }
  if (pid_motor[0] > 0) //左前
  {
    gpio_set_level(DIR_LF, 0);                 // DIR0
    pwm_set_duty(motor_LF, (int)pid_motor[0]); // 正转
  }
  else //DIR1   反转
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-pid_motor[0]);
  }

  if (pid_motor[1] > 0) //左后
  {
    gpio_set_level(DIR_LB, 0);               // DIR0
    pwm_set_duty(motor_LB, (int)pid_motor[1]);// 正转
  }
  else //DIR1  反转
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-pid_motor[1]);
  }

  if (pid_motor[2] > 0) //右前
  {
    gpio_set_level(DIR_RF, 1);
    pwm_set_duty(motor_RF, (int)pid_motor[2]);// 正转
  }
  else //DIR0 反转
  {
    gpio_set_level(DIR_RF, 0);
    pwm_set_duty(motor_RF, (int)-pid_motor[2]);
  }

  if (pid_motor[3] > 0) //DIR1  
  {
    gpio_set_level(DIR_RB, 1);
    pwm_set_duty(motor_RB, (int)pid_motor[3]);// 正转
  }
  else //DIR0  反转
  {
    gpio_set_level(DIR_RB, 0);
    pwm_set_duty(motor_RB, (int)-pid_motor[3]);
  }
}

/**
 * @brief 18鐢垫満缂栫爜鍣ㄩ噷绋嬭?绠楀嚱鏁帮紝鏍规嵁缂栫爜鍣ㄨ剦鍐叉暟璁＄畻杞﹁締琛岄┒璺濈?锛屽崟浣嶏細m
 * @param 鏃?
 * @return 鏃?
 * @attention 琛岄┒璺濈?璁＄畻鍏?紡 = (缂栫爜鍣ㄨ剦鍐叉暟 / 缂栫爜鍣ㄥ垎杈ㄧ巼) * (2 * 蟺 * 杞?瓙鍗婂緞) / 杞?瓙鍛ㄩ暱
 */
void Encoder_odometer(void)
{
  static float Angle_Bias = 0; //瑙掑害鍋忓樊
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  // Angle_Bias = (90 - Angle_Z) * PI / 180; //璁＄畻瑙掑害鍋忓樊

/****璁＄畻杞﹁締閫熷害******/
  // V_enco[0] = 0.2636719 * PI * (float)encoder[0]; // 0.2637涓虹紪鐮佸櫒鍒嗚鲸鐜?
  // V_enco[1] = 0.2636719 * PI * (float)encoder[1];
  // V_enco[2] = 0.2636719 * PI * (float)encoder[2];
  // V_enco[3] = 0.2636719 * PI * (float)encoder[3];
  // Vx_enco=(V_enco[0]-V_enco[1]-V_enco[2]+V_enco[3])/4; //璁＄畻X杞撮�熷害
  // Vy_enco=(V_enco[0]+V_enco[1]+V_enco[2]+V_enco[3])/4; //璁＄畻Y杞撮�熷害
  // Vx_enco = -(V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; //璁＄畻X杞撮�熷害锛堜慨姝ｏ級
  // Vy_enco = -(V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; //璁＄畻Y杞撮�熷害锛堜慨姝ｏ級
  // Car_dis_x += Vx_enco * 0.01; //璁＄畻杞﹁締X杞磋?椹惰窛绂?
  // Car_dis_y += Vy_enco * 0.01; //璁＄畻杞﹁締Y杞磋?椹惰窛绂?

// #if 1
//  if (Angle_Bias >= 0)
//  {
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //璁＄畻杞﹁締閫熷害鍦ㄤ笘鐣屽潗鏍囩郴涓嬬殑鍒嗛噺
//    Vx_world = Vx_2 - Vy_2;
//    Vy_world = Vx_1 + Vy_1;
//  }
//  if (Angle_Bias < 0)
//  {
//    Angle_Bias = -Angle_Bias;
//    Vx_1 = Vx_enco * sin(Angle_Bias);
//    Vx_2 = Vx_enco * cos(Angle_Bias);
//    Vy_1 = Vy_enco * cos(Angle_Bias);
//    Vy_2 = Vy_enco * sin(Angle_Bias); //璁＄畻杞﹁締閫熷害鍦ㄤ笘鐣屽潗鏍囩郴涓嬬殑鍒嗛噺
//    Vx_world = Vx_2 + Vy_2;
//    Vy_world = -Vx_1 + Vy_1;
//  }
// #endif
//  Car_dis_x += Vx_world * 0.01; //璁＄畻杞﹁締X杞磋?椹惰窛绂伙紙淇??锛?
//  Car_dis_y += Vy_world * 0.01; //璁＄畻杞﹁締Y杞磋?椹惰窛绂伙紙淇??锛?

//  Car_dis_x2 += Vx_world * 0.01;
//  Car_dis_y2 += Vy_world * 0.01;
}

/**
 * @brief PID限幅
 *
 * @param Value    pid处理后的pwm
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