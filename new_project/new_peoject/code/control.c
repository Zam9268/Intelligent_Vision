#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#include "take.h"
#include "communication.h"

float Car_H = 0.8; // 车长
float Car_W = 0.6; // 车宽
float Vx, Vy, Vz;
float err_watch;
float move_error;
float turn_error;
float angle;
float now_angle = 0;                 // 转向前的初始角度，默认为0
float catch_angle = 0;               // 捕获搭配卡片的时候，车辆的偏转角，范围为(-90,90)左正右负
float last_catch_angle = 0;          // 上次的偏转角
float turn_angle = 0;                // 转向模式时的目标转向角度，默认为0
int turn_angle_flag = 0;             // 角度旋转标志位
float ahead_speed = 40.0;            // 直行速度
float correct_x_speed = 0;           // x轴上的修正速度
float correct_z_speed = 0;           // z轴上的修正速度
float correct_move_speed = 3;        // x轴修正速度
float correct_turn_speed = 7;        // x轴修正速度
int encoder[4];                      // 编码器数据
int encoder_test[4];                 // 暂时代替的编码器数值
float encoder_sum[4];                // 编码器累加值
float target_encoder_sum[4];         // 目标编码器累加值
float loc_target[4];                 // 位置式处理后的速度
float loc_last_target[4];            // 上次位置式处理后的速度
int Turn_Left_flag, Turn_Right_flag; // 左转右转标志
int loc_Finish_flag = 0;             // 位置式处理完成标志
int Location_pid_flag = 1;           // 位置式处理允许标志
float loc_err;                       // 位置式输入误差
float abs_loc_err;                   // 位置式输入误差绝对值
int pid_motor[4];                    // PID处理后的电机pwm
float bili_act_turn = 1.4;           // 1.28
float loc_kp = 1.24;                 // 位置式pd，方便调参使用 1.35位置式暂时最优 24/4/4     1.26
float loc_kd = 0.72;                 // 0.80                     //0.72
int test_count = 0;
float dt = 0.005;
float turn_error = 2;      //可接受的角度误差
float Turn_KP = 0.5;       // 角度PID//
float Turn_KD = 0.0;       // 角度PID//
// float Turn_KI[1] = {30};  //角度PID//5
float Vx_1, Vx_2, Vy_1, Vy_2;//对里程的cos，sin分解
float Vx_car_1, Vx_car_2, Vy_car_1, Vy_car_2;//对底盘坐标的cos，sin分解
float Vx_world, Vy_world;  //世界坐标上的x，y
float Vx_card, Vy_card;    //相对于车底盘的更新坐标
float Card_dis_car_x=0;
float Card_dis_car_y=0;    //相对于车的更新坐标
float Car_dis_x, Car_dis_y;//x轴，y轴行走距离
float Car_dis_x2, Car_dis_y2;
float Turn_Bias;
float dis_kp = 1.0;       //距离环kp
float dis_kd = 0.0;       //距离环kd
float dis_error;
float dis_change[4];      //存放距离环输出结果
double card_y[10];        //存放卡片y轴坐标
double card_x[10];        //存放卡片y轴坐标
float card_distance;      //存放卡片的合成距离
int only_one = 1;
int target_type = 0;      //测试使用,观察模式
int delta_x,delta_y;      //总钻风识别的卡片中心坐标
float speed_k = 1;        //校正的速度
int CSI_correct_flag = 0; //总钻风判断标志
int Put_flag = 0;         //图片放置标志位
int test_csi;             //延时计数
int car_mode = 0;         //车辆运动模式
float card_angle = 0;     //卡片的解算角度
int catch_card_flag = 0;  //捕获到卡片的标志位
int find_car_flag = 0;    //到达卡片位置的标志位
double delta_card_y,delta_card_x;//卡片x,y坐标与新y里程和x里程的差值
double delta_angle;       //计算出来的即时偏转角
int correct_x_flag = 0;
int correct_y_flag = 0;

pid_info Pos_turn_pid[4];//位置式pid

pid_info Speed[4]; // 增量式pid

pid_info Angle_turn_pid; // 角度环pid

pid_info distance_pid[4]; // 距离环pid

float last_error1 = 0.0f;

/**
 * @brief 电机初始化
 * @param  无
 * @return 无
 */
void Motor_Init(void)
{
  gpio_init(DIR_LF, GPO, GPIO_HIGH, GPO_PUSH_PULL); // gpio给高电平
  gpio_init(DIR_LB, GPO, GPIO_HIGH, GPO_PUSH_PULL); //
  gpio_init(DIR_RF, GPO, GPIO_LOW, GPO_PUSH_PULL); //
  gpio_init(DIR_RB, GPO, GPIO_LOW, GPO_PUSH_PULL); //

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
 
  for (uint8 i = 0; i < 4; i++)
  {
    encoder[i] = 0; // 编码器清零
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
  encoder[2] = encoder_get_count(ENCODER_RF);  // 右前
  encoder[3] = encoder_get_count(ENCODER_RB);  // 右后，正转读正

  for (uint8 i = 0; i < 4; i++)
  {
    Speed[i].now_speed = (encoder[i] * 0.2636719 * PI); // 编码器数据转换成车轮速度，单位为cm/s
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
  Speed[0].target_speed = target_Vx + target_Vy - target_Vz;  // 左前
  Speed[1].target_speed = -target_Vx + target_Vy - target_Vz; // 左后
  Speed[2].target_speed = -target_Vx + target_Vy + target_Vz; // 右前
  Speed[3].target_speed = target_Vx + target_Vy + target_Vz;  // 右后
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
  Speed[0].target_speed = target_Vx + target_Vy - target_Vz * (Car_H / 2 + Car_W / 2);  // 左前
  Speed[1].target_speed = -target_Vx + target_Vy - target_Vz * (Car_H / 2 + Car_W / 2); // 左后
  Speed[2].target_speed = -target_Vx + target_Vy + target_Vz * (Car_H / 2 + Car_W / 2); // 右前
  Speed[3].target_speed = target_Vx + target_Vy + target_Vz * (Car_H / 2 + Car_W / 2);  // 右后
}
/**
 * @brief 停车
 * @param 无
 * @return 无
 */
void car_stop(void)
{
  Speed[0].target_speed = 0; // 左前轮
  Speed[1].target_speed = 0;
  Speed[2].target_speed = 0;
  Speed[3].target_speed = 0;
}
/**
 * @brief 对速度预处理
 * @param 中线误差
 * @return 无
 */
void car_run(void)
{
  err_watch = Err_Handle();
  move_error = err_watch / 94.0f; // 横向比例系数,作归一化处理，94为188/2，半个屏幕的宽

  float kp = 0.85f, kd = 0.2f;//1.0对应速度30

  angle = kp * move_error + kd * (move_error - last_error1); // 原本的+=，现在改成=      2024/3/26

  if (angle > 1.0f)
  {
    angle = 1.0f;
  }
  else if (angle < -1.0f)
  {
    angle = -1.0f;
  }

  last_error1 = move_error; // 记录下上次误差

  Speed[0].target_speed = 10 * (1 - angle);
  Speed[1].target_speed = 10 * (1 - angle);
  Speed[2].target_speed = 10 * (1 + angle);
  Speed[3].target_speed = 10 * (1 + angle);
  // Car_Inverse_kinematics_solution(0, ahead_speed + correct_x_speed, correct_z_speed);//速度解算赋值
}

/**
 * @brief 距离环pid初始化
 * @param 无 对kp kd赋值
 * @return 无
 */
void Distance_PidInit(void)
{
  for (uint8 i = 0; i < 4; i++)
  {
    distance_pid[i].target_speed = 0.00;
    distance_pid[i].target_pwm = 0;
    distance_pid[i].kp = 0.00;
    distance_pid[i].ki = 0.00;
    distance_pid[i].kd = 0.00;
    distance_pid[i].error = 0.00;
    distance_pid[i].lastError = 0.00;
    distance_pid[i].dError = 0.00;
    distance_pid[i].output = 0.00;
    distance_pid[i].output_last = 0.00;
    distance_pid[i].xuhao = i; // 序号
  }

  // 左前
  distance_pid[0].kp = dis_kp;
  distance_pid[0].kd = dis_kd;
  // 左后
  distance_pid[1].kp = dis_kp;
  distance_pid[1].kd = dis_kd;
  // 右前
  distance_pid[2].kp = dis_kp;
  distance_pid[2].kd = dis_kd;
  // 右后
  distance_pid[3].kp = dis_kp;
  distance_pid[3].kd = dis_kd; // PD赋值
}
/**
 * @brief 位置式pid初始化
 * @param 无
 * @return 无
 */
void Pos_PidInit(void)
{
  for (uint8 i = 0; i < 4; i++)
  {
    Pos_turn_pid[i].target_speed = 0.00;
    Pos_turn_pid[i].target_pwm = 0;
    Pos_turn_pid[i].kp = 0.00;
    Pos_turn_pid[i].ki = 0.00;
    Pos_turn_pid[i].kd = 0.00;
    Pos_turn_pid[i].error = 0.00;
    Pos_turn_pid[i].lastError = 0.00;
    Pos_turn_pid[i].dError = 0.00;
    Pos_turn_pid[i].output = 0.00;
    Pos_turn_pid[i].output_last = 0.00;
    Pos_turn_pid[i].xuhao = i; // 序号
  }

  // 左前
  Pos_turn_pid[0].kp = loc_kp; // 0.5对应速度40   0.3//  3/30   1.0  24/4/4纯p
  Pos_turn_pid[0].kd = loc_kd; // 0.5对应速度40   0.8
  // 左后
  Pos_turn_pid[1].kp = loc_kp;
  Pos_turn_pid[1].kd = loc_kd;
  // 右前
  Pos_turn_pid[2].kp = loc_kp;
  Pos_turn_pid[2].kd = loc_kd;
  // 右后
  Pos_turn_pid[3].kp = loc_kp;
  Pos_turn_pid[3].kd = loc_kd; // PD赋值
}

void PidInit(void)
{
  for (uint8 i = 0; i < 4; i++)
  {
    Speed[i].target_speed = 0.00;
    Speed[i].target_pwm = 0;
    Speed[i].kd = 0.00;
    Speed[i].ki = 0.00;
    Speed[i].kd = 0.00;
    Speed[i].error = 0.00;
    Speed[i].lastError = 0.00;
    Speed[i].dError = 0.00;
    Speed[i].output = 0.00;
    Speed[i].output_last = 0.00;
    Speed[i].xuhao = i; // 序号
  }

  // ???
  Speed[0].kp = -16.3; //-16.3  -26
  Speed[0].ki = -3.3;  //-3.0 -1.80
  // ???
  Speed[1].kp = -14.0; //-14.0 -34.5
  Speed[1].ki = -2.5;  //-2.5  -0.98
  // ???
  Speed[2].kp = -15.0; //-15 -28.75
  Speed[2].ki = -3.0;  //-3.0   -0.6
  //???
  Speed[3].kp = -16.3; //-16.0  -28.75
  Speed[3].ki = -2.8;  // PI赋值 -2.8  -1.0
}
/**
 * @brief 增量式pid(单环pid)速度环
 * @param pid_info *pid
 * @return pwm
 */
void increment_pid(void)
{
  for (uint8 i = 0; i < 4; i++)
  {
    //
    Speed[i].error = Speed[i].target_speed - Speed[i].now_speed;                                           // 计算本次误差
    Speed[i].output += Speed[i].kp * (Speed[i].error - Speed[i].lastError) + Speed[i].ki * Speed[i].error; // 增量式处理

    Speed[i].lastlastError = Speed[i].lastError; // 记录上上次误差
    Speed[i].lastError = Speed[i].error;         // 记录上次误差

    Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR); // 限幅
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
  pid->error = Target - Encoder; // Calculate the deviation //

  pid->output = pid->kp * pid->error + pid->kd * (pid->error - pid->lastError); // 原本的+=，现在改成=      2024/3/26

  pid->lastError = pid->error; // 记录下上次误差

  return pid->output;
}
/**
 * @brief 清空编码器累加值
 * @param 无
 * @return 无
 */
void clear_encoder_sum(void)
{
  encoder_sum[0] = 0; // 编码器累加值归零
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
  target_encoder_sum[0] = (distance / PI) * 100 / 0.2636719; // 目标编码器累计脉冲值(即目标脉冲值)
  target_encoder_sum[1] = target_encoder_sum[0];
  target_encoder_sum[2] = target_encoder_sum[0];
  target_encoder_sum[3] = target_encoder_sum[0];
}

/**************************************************************************
位置环处理，直线上可使用,基本弃用
**************************************************************************/
void Drive_Motor()
{
  float LF_Target, LB_Target, RF_Target, RB_Target; // 各个轮子处理输出的脉冲值
  loc_err = Err_Handle();
  abs_loc_err = fabsf(Err_Handle()) * bili_act_turn; // Err_Handle()

  if (abs_loc_err < 4.0) // 设置中线绝对值阈值，小于这个值时，位置式不再起调整作用
  {
    loc_Finish_flag = 1; // 位置式完成标志
    clear_encoder_sum(); // 清空编码器累计值
    int i = 0;
    for (i = 0; i < 4; i++)
    {
      target_encoder_sum[i] = 0; // 目标
      loc_target[i] = 0;         // 各个轮子的位置式输出速度归零
    }
  }

  if (loc_err > 0)      // 右转弯识别出的误差是<0
    Turn_Left_flag = 1; // 左转标志位
  if (loc_err < 0)
    Turn_Right_flag = 1; // 右转标志位

  if (loc_Finish_flag == 0) // 位置式调整未完成
  {
    Set_Distence_m(abs_loc_err); // 转换中线误差

    encoder_sum[0] += fabsf(encoder[0]); // 编码器累加值
    encoder_sum[1] += fabsf(encoder[1]);
    encoder_sum[2] += fabsf(encoder[2]);
    encoder_sum[3] += fabsf(encoder[3]);

    LF_Target = Location_pid(&Pos_turn_pid[0], encoder_sum[0], target_encoder_sum[0]);
    LB_Target = Location_pid(&Pos_turn_pid[1], encoder_sum[1], target_encoder_sum[1]);
    RF_Target = Location_pid(&Pos_turn_pid[2], encoder_sum[2], target_encoder_sum[2]);
    RB_Target = Location_pid(&Pos_turn_pid[3], encoder_sum[3], target_encoder_sum[3]); // 位置式处理，尝试给同一个速度

    loc_target[0] = LF_Target * 0.2636719 * PI / 100; // 将脉冲数转换成编码器速度
    loc_target[1] = LB_Target * 0.2636719 * PI / 100;
    loc_target[2] = RF_Target * 0.2636719 * PI / 100;
    loc_target[3] = RB_Target * 0.2636719 * PI / 100; // 单位为cm/s

    if (Turn_Left_flag == 1) // 左转，或者在中线右侧
    {
      loc_target[0] = -fabsf(loc_target[0]);
      loc_target[1] = -fabsf(loc_target[1]);
      loc_target[2] = fabsf(loc_target[2]);
      loc_target[3] = fabsf(loc_target[3]);
    }
    else if (Turn_Right_flag == 1) // 右转，或者在中线
    {
      loc_target[0] = fabsf(loc_target[0]);
      loc_target[1] = fabsf(loc_target[1]);
      loc_target[2] = -fabsf(loc_target[2]);
      loc_target[3] = -fabsf(loc_target[3]);
    }

    loc_last_target[0] = loc_target[0]; // 记录上次位置式处理速度
    loc_last_target[1] = loc_target[1];
    loc_last_target[2] = loc_target[2];
    loc_last_target[3] = loc_target[3];

    for (uint8 i = 0; i < 4; i++)
    {
      loc_target[i] = PIDInfo_Limit(loc_target[i], 40.0); // 输出速度限幅
    }
  }
}
/**
 * @brief 串级pid 双环(位置环+速度环)
 * @param 无
 * @return 无
 */
void turnloc_pid(void)
{

  for (uint8 i = 0; i < 4; i++)
  {
    // 速度环
    Speed[i].lastlastError = Speed[i].lastError;                                                           // 记录上上次输出
    Speed[i].lastError = Speed[i].error;                                                                   // 记录上次输出
    Speed[i].error = Speed[i].target_speed + loc_target[i] - Speed[i].now_speed;                           // 改变目标速度
    Speed[i].output += Speed[i].kp * (Speed[i].error - Speed[i].lastError) + Speed[i].ki * Speed[i].error; // 输出pwm
    Speed[i].output = PIDInfo_Limit(Speed[i].output, AMPLITUDE_MOTOR);                                     // 限幅
  }

  Turn_Left_flag = 0;
  Turn_Right_flag = 0; // 左转/右转标志位清零

  loc_Finish_flag = 0; // 位置式完成标志清零
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
  for (j = 0; j < 4; j++) // 各个电机的pwm赋值
  {
    pid_motor[j] = Speed[j].output;
    // Speed[j].output=0;
  }
  if (pid_motor[0] > 0) // 左前轮正转
  {
    gpio_set_level(DIR_LF, 0);                 // DIR0
    pwm_set_duty(motor_LF, (int)pid_motor[0]); // 左前
  }
  else // 反转
  {
    gpio_set_level(DIR_LF, 1);
    pwm_set_duty(motor_LF, (int)-pid_motor[0]);
  }

  if (pid_motor[1] > 0) //左后轮
  {
    gpio_set_level(DIR_LB, 0);
    pwm_set_duty(motor_LB, (int)pid_motor[1]);
  }
  else //???
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-pid_motor[1]);
  }

  if (pid_motor[2] > 0) //右前轮，正转
  {
    gpio_set_level(DIR_RF, 0);//0
    pwm_set_duty(motor_RF, (int)pid_motor[2]);
  }
  else // 反转
  {
    gpio_set_level(DIR_RF, 1); //1
    pwm_set_duty(motor_RF, (int)-pid_motor[2]);
  }

  if (pid_motor[3] > 0) //右后轮
  {
    gpio_set_level(DIR_RB, 0);//正转
    pwm_set_duty(motor_RB, (int)pid_motor[3]);
  }
  else //???
  {
    gpio_set_level(DIR_RB, 1);//反转
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
  static float Last_Turn_bias = 0, Turn = 0;
  Turn_Bias = Tar_angle_Z - Angle_Z; // Angle_Z为当前角度偏差，由陀螺仪获取

  if (fabsf(Turn_Bias) < turn_error) // 当前角度和目标角度相差绝对值在这个范围内是认为转向成功
  {
    Vz = 0;
  }
  else
  {
    Turn = Turn_KP * Turn_Bias + Turn_KD * (Turn_Bias - Last_Turn_bias); // 原来是增量式处理，现在变更为位置式PD输出速度
    if (Turn > Turn_limiting)
      Turn = Turn_limiting;
    if (Turn < -Turn_limiting)
      Turn = -Turn_limiting;
    Vz = Turn;
    Last_Turn_bias = Turn_Bias;
  }
  Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
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
  static float Angle_bias = 0; // 用于识别到卡片时候的角度误差
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  Angle_Bias = Angle_Z * PI / 180; // 转换成弧度制，Angle_Z为转向角度

  V_enco[0] = 0.5273438 * PI * encoder[0]; // 0.2637可以再精确多三位，计算车轮路程
  V_enco[1] = 0.5273438 * PI * encoder[1];
  V_enco[2] = 0.5273438 * PI * encoder[2];
  V_enco[3] = 0.5273438 * PI * encoder[3];

  Vx_enco = (V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; // 前进为正，根据麦轮速度解算公式得出的底盘x轴位移量
  Vy_enco = (V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; // 左移为正，根据麦轮速度解算公式得出的底盘y轴位移量

#if 1
  if (Angle_Bias >= 0) // 旋转角度(参照x轴)大于0时
  {
    Vx_1 = Vx_enco * sin(Angle_Bias);
    Vx_2 = Vx_enco * cos(Angle_Bias);
    Vy_1 = Vy_enco * cos(Angle_Bias);
    Vy_2 = Vy_enco * sin(Angle_Bias); // 分解到世界坐标上
    Vx_world = Vx_2 - Vy_2;           // 简单的分解计算
    Vy_world = Vx_1 + Vy_1;
  }
  if (Angle_Bias < 0)
  {
    Angle_Bias = -Angle_Bias;
    Vx_1 = Vx_enco * sin(Angle_Bias);
    Vx_2 = Vx_enco * cos(Angle_Bias);
    Vy_1 = Vy_enco * cos(Angle_Bias);
    Vy_2 = Vy_enco * sin(Angle_Bias); // 分解到世界坐标上
    Vx_world = Vx_2 + Vy_2;           // 简单的分解计算
    Vy_world = -Vx_1 + Vy_1;
  }
#endif
  if (catch_card_flag == 1)
  {
    Angle_bias = Angle_z * PI / 180;

  if (Angle_bias >= 0)//旋转角度(参照x轴)大于0时
  {
    Vx_car_1 = Vx_enco * sin(Angle_bias);
    Vx_car_2 = Vx_enco * cos(Angle_bias);
    Vy_car_1 = Vy_enco * cos(Angle_bias);
    Vy_car_2 = Vy_enco * sin(Angle_bias); //分解到车辆底盘坐标上
    Vx_card = Vx_2 - Vy_2;//简单的分解计算
    Vy_card = Vx_1 + Vy_1;
  }
  if (Angle_bias < 0)
  {
    Angle_bias = -Angle_bias;
    Vx_car_1 = Vx_enco * sin(Angle_bias);
    Vx_car_2 = Vx_enco * cos(Angle_bias);
    Vy_car_1 = Vy_enco * cos(Angle_bias);
    Vy_car_2 = Vy_enco * sin(Angle_bias); //分解到车辆底盘坐标上
    Vx_card = Vx_2 + Vy_2;//简单的分解计算
    Vy_card = -Vx_1 + Vy_1;
  }
  Card_dis_car_x += Vx_card * 0.005;
  Card_dis_car_y += Vy_card * 0.005;//分解出卡片所需的里程，用于找卡片
}
  Car_dis_x += Vx_world * 0.005;//用于全局坐标
  Car_dis_y += Vy_world * 0.005;

  Car_dis_x2 += Vx_world * 0.005;//用于总钻风微调
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
/**
 * @brief 距离环
 *
 * @param pid_info *pid 距离pid结构体
 * @param delta_distance 实际距离
 * @return float
 */
float Distance_pid(pid_info *pid, int error)
{
  pid->error = error;                               // Calculate the deviation //
  pid->output = pid->kp * pid->error + pid->kd * (pid->error - pid->lastError); // 距离闭环输出一个速度
  pid->output = PIDInfo_Limit(pid->output, Distance_output);                    // 输出速度限幅，mm/s
  pid->lastError = pid->error;                                                  // 记录下上次误差
  return pid->output;
}
/**************************************************************************
函数功能：总钻风距离校正(未调参) 旧版
入口参数：cor_x，cor_y（要校正的x和y），art识别出来的坐标一般有偏差，所以要再次识别中心点的x,y坐标输入矫正函数
返回值：
**************************************************************************/
// void CSI_dis_correct(float cor_x, float cor_y)
// {
//   delta_x = cor_x; // 要换算，与现实坐标有差别(可能)
//   delta_y = cor_y; // 要换算

//   if (delta_x > 6 || delta_x < -6 && abs((int)delta_y) > 40) // 误差太大，需要校正(一般情况)
//   {
//     Vx = 7 * (cor_x / 10) * speed_k;
//     Vy = 7 * (cor_y / 42) * speed_k;

//     Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 输入至麦轮解算
//   }
//   else if ((delta_x == 0 && delta_y == 0) || abs((int)delta_y) < 35) // 图片中无目标板，或者离目标板太近，往后退
//   {
//     Vx = 0;
//     if (abs((int)delta_y) < 20)
//       Vy = -10 * speed_k; // 
//     else if (abs((int)delta_y) > 20)
//       Vy = -8 * speed_k; // 降速

//     Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 输入至麦轮解算
//   }
//   else if (delta_x <= 6 && delta_x >= -6 && abs((int)delta_y) <= 40 && abs((int)delta_y) >= 35) // 不需要校正
//   {
//     delta_x = 0;
//     delta_y = 0; // 清零x,y坐标
//     Vx = 0;
//     Vy = 0; // x，y速度归零

//     CSI_correct_flag = CSI_CORRECT_DONE; // 校正完成
//     Car_Inverse_kinematics_solution(Vx, Vy, Vz);
//     if (Put_flag == 0)
//     {
//       arm_pick_flag = ARM_PICK_NOT_DONE;
//       arm_state_flag = ARM_STATE_ON; // 打开机械臂拾取功能
//     }
//     test_csi = 0;
//   }
//   Car_Inverse_kinematics_solution(Vx, Vy, Vz);
// }
/**************************************************************************
函数功能：总钻风距离校正(未调参) 新版
入口参数：cor_x，cor_y（要校正的x和y），art识别出来的坐标一般有偏差，所以要再次识别中心点的x,y坐标输入矫正函数，新版加上距离闭环
返回值：
**************************************************************************/
void CSI_dis_new_correct(int cor_x, int cor_y)
{
  delta_x = (cor_x-23)/10; // 
  delta_y = cor_y/10-21; //计算出中心坐标,y可能需要调整
  //调整x方向
  if(correct_x_flag==0 && correct_y_flag==0)
  {
     if(abs(delta_x)>1&&correct_x_flag==0)//x距离过大，需要矫正
   {
     Vx=Distance_pid(&distance_pid[0], delta_x);
     Vy=0;
		 correct_x_flag=0;
		 correct_y_flag=0;
   }
     else if(delta_x<1 && delta_x>-1)//已调整完毕 
   {
     Vx=0;
     Vy=0;
     correct_x_flag=1;//x方向调整完毕
   }
  }
	//调整y方向
  if(correct_x_flag==1 && correct_y_flag==0)//y距离过大，需要矫正
  {
    if(delta_y>1&&correct_y_flag==0)//y距离过大，需要矫正
   {
     Vy=Distance_pid(&distance_pid[0], delta_y);
     Vx=0;
		 correct_y_flag=0;
   }
    else if(delta_y<1 && delta_y>-1)//已调整完毕
   {
     Vx=0;
     Vy=0;
    correct_y_flag=1;//y方向调整完毕
   }
  }
  if(correct_y_flag==1 && correct_x_flag==1)
	{
    CSI_correct_flag=1;//总钻风调整完毕
		correct_x_flag=0;//调整标志位清0
		correct_y_flag=0;
	}
  Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 输入至麦轮解算
}
/**
 * @brief 小车模式切换，打包函数
 * @param mode为模式选择
 * @param
 * @return 无
 */
void car_findcard(int *mode)
{
  //******************************正常循迹*****************************//
  if (*mode == Car_go) // 寻迹模式，对赛道进行处理,默认设置
  {
    if (now_distance_y > 0 && now_distance_y < 800) // art识别到卡片，设定识别区间
    {
      if (only_one) // 只执行一次
      {
				Card_dis_car_y=0;//底座坐标清零
        card_y[0] = now_distance_y/10;//记录下第一次传进来的数据
        card_x[0] = now_distance_x/10;//存放卡片y轴坐标
        catch_card_flag = 1;//捕获成功，记得要重新关闭,打开里程计的第二种模式
        Angle_z = 0;//角度清0
        only_one = 0;//测试使用
        *mode = Car_find_card_y;//转变小车运动模式
				target_type = *mode;//测试变量使用
      }			
     }
		else
		{
      car_run();//正常巡线模式
			*mode=Car_go;
		}
  }
  //******************************找卡片*****************************//
  if (*mode == Car_find_card_y) 
  {
    if (find_car_flag == 1) // 到达卡片附近，原本是4
    {
			if(only_one)
			{
         now_angle = Angle_Z;//记录下转向前的角度
			   if(delta_card_x<=0)//卡片相对于小车在左边时
        {
          turn_angle=90+now_angle;//向左转90度 
        }
        else
        {
          turn_angle=-90+now_angle;//向右转向90度
        }
        only_one = 0;     // 只执行一次
        *mode = Car_turn; // 模式转变
        pick_up_mode = 1; //摄像头模式变为总钻风识别
        target_type = *mode;
      }
    }
		else
		{
			only_one=1;//重新打开only_one
      car_run();//正常循迹跑
      delta_card_y = card_y[0]-(double)Card_dis_car_y;//算出在更新后的坐标轴下的y差值
      delta_card_x = card_x[0]-(double)Card_dis_car_x;//算出在更新后的坐标轴下的x差值
			if(delta_card_x == 0)
				delta_angle = 90.0;//当delta_x刚好为0值时(此时tan值无意义)，把这时的角度就认为为90
			else
        delta_angle = atan((double)(delta_card_y/delta_card_x))/PI*180*1.0;//算出即时偏移角
        //因为车身姿态与采样频率的问题，有且只有一个相交点，给出在符合角度的波动区间
      if(delta_angle-Angle_z<10 && delta_angle-Angle_z>-15 && fabsf(Angle_z)>10.0)//当底盘坐标需要偏角较大的时候,一般在弯道
      {
        car_stop();//停车
				system_delay_ms(1000);
        find_car_flag = 1;
        *mode=Car_find_card_y;
      }
      else if(delta_angle-Angle_z<1 && delta_angle-Angle_z>-1 && fabsf(Angle_z)<3.0)//当底盘坐标需要偏角较小的时候,一般在直道
      {
        car_stop();//停车
				system_delay_ms(1000);
        find_car_flag = 1;
        *mode=Car_find_card_y;
      }
		}
  }
//******************************向卡片方向转向*****************************//
  if (*mode == Car_turn) 
  {
    if (fabsf(Angle_Z - turn_angle) < 1) // 陀螺仪转向识别
    {
      // Vz = 0;//清0Vz
      Car_dis_x2 = 0;
      Car_dis_y2 = 0;//用于总钻风微调，清0为下一步做准备
      
      *mode = Car_find_card_cor; // 模式转变
      target_type = *mode;
    }
    else
    {
      Turn_Angle_PD(turn_angle);
      *mode = Car_turn;
    }
  }
//******************************总钻风对正*****************************//
  if (*mode == Car_find_card_cor) // 总钻风微调识别
  {
    if (CSI_correct_flag == 1) // 总钻风坐标对正
    {
      *mode = Pick_up_card;//模式转变
			target_type = *mode;
    }
		else
		{
      CSI_dis_new_correct(center_x, center_y);//总钻风坐标对正
			*mode = Car_find_card_cor;
		}
	}
  //******************************卡片拾取*****************************//
    if (*mode == Pick_up_card) // 捡卡片
    {
     if(arm_pick_flag==ARM_PICK_DONE)//卡片已被拾取
	   {
       catch_card_flag=0;//退出里程计第二种模式
       card_x[0]=0;//卡片坐标清空
       card_y[0]=0;
       *mode = Car_turn_again;//模式转变为转向回正
	   }
      else//卡片未被拾取
	   {
		  arm_control(2);//捡卡片
	    arm_control(3);//默认模式
		  arm_pick_flag=ARM_PICK_DONE;
      *mode = Pick_up_card;
	   }
    }
  //******************************车头回正*****************************//
    if (*mode == Car_turn_again) 
    {
    if (fabsf(Angle_Z - now_angle) < 1) // 陀螺仪转向识别
    {
    // Vz = 0;//清0Vz
     *mode = Car_go; //重新变为寻迹
     pick_up_mode=1; //摄像头变为寻迹模式
     target_type = *mode;
    }
    else
    {
     Turn_Angle_PD(now_angle);//向原先的角度转向回正
     *mode = Car_turn_again;
    }
   }
}