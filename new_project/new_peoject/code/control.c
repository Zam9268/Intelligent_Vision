#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#include "take.h"
#include "communication.h"

extern uint8 card_left_up_find_flag;          // the lef up corner of the card lying on the side of the road is found
extern uint8 card_right_up_find_flag;  
extern char uart_4_begin[];
extern int near_card_x;
extern int near_card_y;
extern fifo_struct uart_data_fifo; // UART数据FIFO结构体
extern uint8 uart_get_data[64];
extern RoadType Road_Type;
extern uint8 Last_Longest_White_Column_Left[2];
extern uint8 Longest_White_Column_Left[2];
pid_info Speed[4]; // 增量式pid
extern int record_now_distance_x;
extern unsigned int record_now_distance_y;
extern char uart_1_begin[];    // UART4开始字符串
extern char uart_1_stop[]; // UART4开始字符串abc
/******************坡道绕行函数所需变量*****************/
uint8 find_ramp;//用于切换坡道里程计调整
uint8 ramp_step;//用于坡道绕行函数的步数调整
uint8 ramp_finish=0;//坡道绕行完成标志
/*******************************************************/
uint8 chance=0;
float Vx, Vy, Vz;
/**************差速所用变量*************************/
float err_watch;
float move_error;
float angle;
float ahead_speed = 40.0;            // 直行速度
/***************************************************/
/*********************初始串级pid所需变量(已经弃用)******************/
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
float bili_act_turn = 1.4;           // 1.28
float loc_kp = 1.24;                 // 位置式pd，方便调参使用 1.35位置式暂时最优 24/4/4     1.26
float loc_kd = 0.72;                 // 0.80                     //0.72
/***********************************************************/
int test_count = 0;
float dt = 0.005;
/*******************角度环所需变量***************************/
float turn_error = 3;      //可接受的角度误差
float Turn_KP = 1.0;       // 角度PID//
float Turn_KD = 0.6;       // 角度PID//
// float Turn_KI[1] = {30};  //角度PID//5
/************************************************************/
//*****************里程计所用变量****************//
float Car_dis_x1,Car_dis_y1;//
float Car_dis_x2, Car_dis_y2;

uint8 Island_classify_flag=0;

float Vx_1, Vx_2, Vy_1, Vy_2;//对里程的cos，sin分解
float Vx_car_1, Vx_car_2, Vy_car_1, Vy_car_2;//对底盘坐标的cos，sin分解
float Vx_correct_1,Vx_correct_2,Vy_correct_1,Vy_correct_2;//用于总钻风微调的里程计
float Vx_ramp_1,Vx_ramp_2,Vy_ramp_1,Vy_ramp_2;        //用于坡道绕行
float Vx_Island_1,Vx_Island_2,Vy_Island_1,Vy_Island_2;//用于环岛分类

float Vx_correct,Vy_correct;//用于总钻风的里程计
float Vx_world, Vy_world;   //世界坐标上的x，y
float Vx_card, Vy_card;     //相对于车底盘的更新坐标
float Vx_ramp,Vy_ramp;      //坡道绕行时使用的速度
float Vx_Island,Vy_Island; //环岛分类合成的速度

float Card_dis_car_x=0;
float Card_dis_car_y=0;     //相对于车的更新坐标
float Car_dis_x, Car_dis_y; //x轴，y轴行走距离
float correct_x,correct_y;  //修正的x和y
float ramp_x,ramp_y;        //坡道绕行的里程计
float Island_x,Island_y;    //环岛绕行的里程计

//***********************************************//
int   car_world_distance;//车辆在全局坐标上与原点的距离
float car_world_angle;//卡片世界坐标解算出的世界方位角
float Turn_Bias;
/****************距离环所需变量*****************/
float dis_kp = 1.5;       //距离环kp
float dis_kd = 0.4;       //距离环kd
float dis_error;
float dis_change[4];      //存放距离环输出结果
/***********************************************/
/***************************************总的打包函数所需变量*************************************************/
int card_y[10];           //存放卡片y轴坐标
int card_x[10];           //存放卡片y轴坐标
float card_distance;      //存放卡片的合成距离
int only_one = 1;
int target_type = 0;      //测试使用,观察模式
int delta_x,delta_y;      //总钻风识别的卡片中心坐标
int now_count=0;          //现在是第几张卡片
int CSI_correct_flag = 0; //总钻风判断标志
int Put_flag = 0;         //图片放置标志位
int test_csi;             //延时计数
int car_mode = 0;         //车辆运动模式
float now_angle = 0;      //转向前的初始角度，默认为0
float turn_angle = 0;     //转向模式时的目标转向角度，默认为0
float card_angle = 0;     //卡片的解算角度
int catch_card_flag = 0;  //捕获到卡片的标志位
int arrive_card_flag;     //打开修正里程计标志
int find_car_flag = 0;    //到达卡片位置的标志位
int correct_art2_flag;    //打开art4的中断标志位
double delta_card_y,delta_card_x;//卡片x,y坐标与新y里程和x里程的差值
double delta_angle;       //计算出来的即时偏转角
float ahead_distance;     //前进的距离
/**************************************************************************************************************/
/*********************用于卡片分类的变量*****************************/
uint8 Traffic=1;         //交通工具类
uint8 Weapon =2;         //武器类
uint8 Supply =3;		     //物资类
int Traffic_count=0;     //拾取的交通工具卡片数
int Weapon_count=0;      //拾取的武器总卡片数
int Supply_count=0;      //拾取的物资的总卡片数
/********************************************************************/
/******************用于总钻风修正的变量*******************/
int card_center_x;
int card_center_y;//卡片中心坐标
uint8 card_classify=0;    //记录卡片的分类
int correct_x_flag = 0;
int correct_y_flag = 0;
int correct_step=1;//校正步数
int ahead_flag=0;
/*********************************************************/
//*********新的行进函数***********/
int now_card=0;//当前的卡片
int card_car_x,card_car_y;//世界坐标下卡片与车辆的x坐标差值和y坐标差值
int card_car_other_angle=0;//由上面两个差值解算出的角度
/*********************************/
/*********************用于上边线寻迹所需的变量*****************************/
float top_error,last_top_error;         //与目标行数的加权误差
/************************************************************************/
//*********斑马线分类函数***************/
int once_time=1;
int classify_mode = 0;
float Now_angle;
int num_card_x,num_card_y;
uint8 class_step;
uint8 classify_correct_finish;
uint8 numcard_classify;
int delta_class_x,delta_class_y;
int classify_art2_flag;
int classify_type;//用于观察分类的模式
uint8 Find_num=0;   //识别完毕的标志位
int put_out_count;  //需要放出的卡片数目
uint8 put_out_card_flag=0;      //所有卡片是否放出的标志位
uint8 Traffic_Finish=0;         //交通工具类
uint8 Weapon_Finish =0;         //武器类
uint8 Supply_Finish =0;		     //物资类
int banmaxian_finish;          //斑马线处理完成与否的标志位
/*************************************/
/*************************各种pid*****************************/
int pid_motor[4];

pid_info Pos_turn_pid[4];//位置式pid

pid_info Angle_turn_pid; // 角度环pid

pid_info distance_pid[4]; // 距离环pid
/*************************************************************/
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

  float kp = 1.0f, kd = 0.5f;//1.0对应速度30  0.9响应10
  float target_all_speed = 10;

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

  Speed[0].target_speed = target_all_speed * (1 - angle);
  Speed[1].target_speed = target_all_speed * (1 - angle);
  Speed[2].target_speed = target_all_speed * (1 + angle);
  Speed[3].target_speed = target_all_speed * (1 + angle);
  // Car_Inverse_kinematics_solution(0, ahead_speed + correct_x_speed, correct_z_speed);//速度解算赋值
}
/**
 * @brief 对速度预处理
 * @param 中线误差
 * @return 无
 */
void car_run_upline(void)
{
  Top_Line_Search();//对上边线扫线，为上边线数组做准备
  top_error=Top_Line_Err(95)/25;//对上边线扫线输出一个误差,作归一化处理
  if (top_error > 1.0f)
  {
    top_error = 1.0f;
  }
  else if (top_error < -1.0f)
  {
    top_error = -1.0f;
  }
  float top_kp = 5.0f, top_kd = 1.0f;//1.0对应速度30  0.9响应10
  Vy= top_kp * top_error + top_kd * (top_error - last_top_error);//输出为Vy速度
  last_top_error = top_error; // 记录下上次误差
  Vx=10;//给固定的横移速度
  // Car_Inverse_kinematics_solution(Vx, Vy, 0);//速度解算赋值
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
  Speed[0].kp = -16.15; //-16.15  -26
  Speed[0].ki = -3.13;  //-3.13 -1.80
  // ???
  Speed[1].kp = -15.19; //-15.15 -34.5
  Speed[1].ki = -3.3;  //-3.3  -0.98
  // ???
  Speed[2].kp = -15.0; //-15 -28.75
  Speed[2].ki = -3.2;  //-3.2   -0.6
  //???
  Speed[3].kp = -17.3; //-17.3  -28.75
  Speed[3].ki = -3.2;  // PI赋值 -3.2  -1.0
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
    gpio_set_level(DIR_RF, 1);//0
    pwm_set_duty(motor_RF, (int)pid_motor[2]);
  }
  else // 反转
  {
    gpio_set_level(DIR_RF, 0); //1
    pwm_set_duty(motor_RF, (int)-pid_motor[2]);
  }

  if (pid_motor[3] > 0) //右后轮
  {
    gpio_set_level(DIR_RB, 1);//正转
    pwm_set_duty(motor_RB, (int)pid_motor[3]);
  }
  else //???
  {
    gpio_set_level(DIR_RB, 0);//反转
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
  // Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
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
  static float Angle_correct_bias;//用于总钻风修正
  static float Angle_ramp_bias;   //用于坡道绕行
  static float Angle_Island_bias; //用于环岛分类
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  Angle_Bias = Angle_Z * PI / 180; // 转换成弧度制，Angle_Z为转向角度

  V_enco[0] = 0.5273438 * PI * encoder[0]; // 0.2637可以再精确多三位，计算车轮路程
  V_enco[1] = 0.5273438 * PI * encoder[1];
  V_enco[2] = 0.5273438 * PI * encoder[2];
  V_enco[3] = 0.5273438 * PI * encoder[3];

  Vx_enco = (V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; // 前进为正，根据麦轮速度解算公式得出的底盘x轴位移量
  Vy_enco = (V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; // 左移为正，根据麦轮速度解算公式得出的底盘y轴位移量

#if 1//默认设置
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
  Car_dis_x += Vx_world * 0.005;//用于全局坐标
  Car_dis_y += Vy_world * 0.005;

  car_world_distance = sqrt(Car_dis_x * Car_dis_x + Car_dis_y * Car_dis_y);//车辆与原点的距离
  car_world_angle = atan2(Car_dis_y,Car_dis_x)/PI*180*1.0;//角度制，车辆相对于原点解算出来的角度
  /***********************************************************/
  car_card_angle = card_world_angle - car_world_angle;//角度制
  car_card_diatance = (int)sqrt((card_world_distance * card_world_distance)					 //卡片与原点距离的平方
                                 +(car_world_distance * car_world_distance)          //车辆与原点距离的平方
                                 -2*card_world_distance*car_world_distance*cos(car_card_angle/180*PI));
 
  Car_dis_x2 += Vx_world * 0.005;//用于赛道回正
  Car_dis_y2 += Vy_world * 0.005;
/**************************搜索卡片的时候使用********************************/
  if (catch_card_flag == 1)//捕获到卡片时的里程计
  {
    Angle_bias = Angle_z * PI / 180;

  if (Angle_bias >= 0)//旋转角度(参照x轴)大于0时
  {
    Vx_car_1 = Vx_enco * sin(Angle_bias);
    Vx_car_2 = Vx_enco * cos(Angle_bias);
    Vy_car_1 = Vy_enco * cos(Angle_bias);
    Vy_car_2 = Vy_enco * sin(Angle_bias); //分解到车辆底盘坐标上
    Vx_card = Vx_car_2 - Vy_car_2;//简单的分解计算
    Vy_card = Vx_car_1 + Vy_car_1;
  }
  if (Angle_bias < 0)
  {
    Angle_bias = -Angle_bias;
    Vx_car_1 = Vx_enco * sin(Angle_bias);
    Vx_car_2 = Vx_enco * cos(Angle_bias);
    Vy_car_1 = Vy_enco * cos(Angle_bias);
    Vy_car_2 = Vy_enco * sin(Angle_bias); //分解到车辆底盘坐标上
    Vx_card = Vx_car_2 + Vy_car_2;//简单的分解计算
    Vy_card = -Vx_car_1 + Vy_car_1;
  }
  Card_dis_car_x += Vx_card * 0.005;
  Card_dis_car_y += Vy_card * 0.005;//分解出卡片所需的里程，用于找卡片
}
/*************************总钻风调整时使用***************************************/
 if (arrive_card_flag == OPEN)             //总钻风微调，总钻风微调时的里程计
  {
    Angle_correct_bias = Angle_arrive_card * PI / 180;

  if (Angle_correct_bias >= 0)//旋转角度(参照x轴)大于0时
  {
    Vx_correct_1 = Vx_enco * sin(Angle_correct_bias);
    Vx_correct_2 = Vx_enco * cos(Angle_correct_bias);
    Vy_correct_1 = Vy_enco * cos(Angle_correct_bias);
    Vy_correct_2 = Vy_enco * sin(Angle_correct_bias); //用于总钻风微调
    Vx_correct = Vx_correct_2 - Vy_correct_2;//简单的分解计算
    Vy_correct = Vx_correct_1 + Vy_correct_1;
  }
  if (Angle_correct_bias < 0)
  {
    Angle_correct_bias = -Angle_correct_bias;
    Vx_correct_1 = Vx_enco * sin(Angle_correct_bias);
    Vx_correct_2 = Vx_enco * cos(Angle_correct_bias);
    Vy_correct_1 = Vy_enco * cos(Angle_correct_bias);
    Vy_correct_2 = Vy_enco * sin(Angle_correct_bias); //用于总钻风微调
    Vx_correct = Vx_correct_2 + Vy_correct_2;//简单的分解计算
    Vy_correct = -Vx_correct_1 + Vy_correct_1;
  }
  correct_x += Vx_correct * 0.005;
  correct_y += Vy_correct * 0.005;//分解出卡片所需的里程，用于找卡片
}
/***************************坡道绕行时使用***************************************/
if (find_ramp == OPEN)             //坡道调整
{
    Angle_ramp_bias = Angle_ramp * PI / 180;

  if (Angle_ramp_bias >= 0)//旋转角度(参照x轴)大于0时
  {
    Vx_ramp_1 = Vx_enco * sin(Angle_ramp_bias);
    Vx_ramp_2 = Vx_enco * cos(Angle_ramp_bias);
    Vy_ramp_1 = Vy_enco * cos(Angle_ramp_bias);
    Vy_ramp_2 = Vy_enco * sin(Angle_ramp_bias); //用于坡道调整
    Vx_ramp = Vx_ramp_2 - Vy_ramp_2;//简单的分解计算
    Vy_ramp = Vx_ramp_1 + Vy_ramp_1;
  }
  if (Angle_ramp_bias < 0)
  {
    Angle_ramp_bias = -Angle_ramp_bias;
    Vx_ramp_1 = Vx_enco * sin(Angle_ramp_bias);
    Vx_ramp_2 = Vx_enco * cos(Angle_ramp_bias);
    Vy_ramp_1 = Vy_enco * cos(Angle_ramp_bias);
    Vy_ramp_2 = Vy_enco * sin(Angle_ramp_bias); //用于总钻风微调
    Vx_ramp = Vx_ramp_2 + Vy_ramp_2;//简单的分解计算
    Vy_ramp = -Vx_ramp_1 + Vy_ramp_1;
  }
  ramp_x += Vx_ramp * 0.005;
  ramp_y += Vy_ramp * 0.005;//分解出卡片所需的里程，用于找卡片
}
/***************************环岛分类时使用***************************************/
if (Island_classify_flag == OPEN)             //坡道调整
{
    Angle_Island_bias = Angle_Island * PI / 180;

  if (Angle_Island_bias >= 0)//旋转角度(参照x轴)大于0时
  {
    Vx_Island_1 = Vx_enco * sin(Angle_Island_bias);
    Vx_Island_2 = Vx_enco * cos(Angle_Island_bias);
    Vy_Island_1 = Vy_enco * cos(Angle_Island_bias);
    Vy_Island_2 = Vy_enco * sin(Angle_Island_bias); //用于坡道调整
    Vx_Island = Vx_Island_2 - Vy_Island_2;//简单的分解计算
    Vy_Island = Vx_Island_1 + Vy_Island_1;
  }
  if (Angle_Island_bias < 0)
  {
    Angle_Island_bias = -Angle_Island_bias;
    Vx_Island_1 = Vx_enco * sin(Angle_Island_bias);
    Vx_Island_2 = Vx_enco * cos(Angle_Island_bias);
    Vy_Island_1 = Vy_enco * cos(Angle_Island_bias);
    Vy_Island_2 = Vy_enco * sin(Angle_Island_bias); //用于总钻风微调
    Vx_Island = Vx_Island_2 + Vy_Island_2;//简单的分解计算
    Vy_Island = -Vx_Island_1 + Vy_Island_1;
  }
  Island_x += Vx_Island * 0.005;
  Island_y += Vy_Island * 0.005;//分解出卡片所需的里程，用于找卡片
}
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
float Distance_pid(pid_info *pid, int target_diantance, int actual_distance)
{
  pid->error = target_diantance-actual_distance;                               // Calculate the deviation //
  pid->output = pid->kp * pid->error + pid->kd * (pid->error - pid->lastError); // 距离闭环输出一个速度
  pid->output = PIDInfo_Limit(pid->output, Distance_output);                    // 输出速度限幅，mm/s
  pid->lastError = pid->error;                                                  // 记录下上次误差
  return pid->output;
}
/**************************************************************************
函数功能：总钻风距离校正(未调参) 开环
入口参数：cor_x，cor_y（要校正的x和y），art识别出来的坐标一般有偏差，所以要再次识别中心点的x,y坐标输入矫正函数，新版加上距离闭环
返回值：
**************************************************************************/
void CSI_dis_new_correct(int cor_x, int cor_y)
{
  delta_x = (cor_x-10)/10-(int)correct_x; //单位为cm
  delta_y = cor_y/10-(int)correct_y; //计算出中心坐标,y可能需要调整，参数暂定
  //调整x方向
switch (correct_step)
 {

 case 1:                                                       //调整垂直方向
    if(abs(delta_y)>0 && correct_y_flag==0 && correct_step==1)//y距离过大，需要矫正，默认为第一步
   {
		 if(delta_y>23)
		 {
			 Vx=0;//水平不动
			 Vy=5;//向前移动
		 }
		 else if(delta_y<18)
		 {
			 Vx=0;//水平不动
			 Vy=-5;//向后移动
		 }
     else if(delta_y<=23 && delta_y>=18)//已调整完毕 
   {
     Vx=0;
     Vy=0;//速度清零
     correct_y_flag=1;//y方向调整完毕
		 correct_step=2;//调整步数置2
   }
   }			 
   break;

 case 2: //
	 if(correct_y_flag==1 && correct_x_flag==0 && correct_step==2)//x距离过大，需要矫正，且步数为第二步
   {
      if(abs(delta_x)>2&& correct_x_flag==0)//x距离过大，需要矫正
    {
      Vy=0;
			if(delta_x>0)
        Vx=5;
			else
				Vx=-5;
		  correct_x_flag=0;
    }
      else if(delta_x<=2 && delta_x>=-2)//已调整完毕
    {
      Vx=0;
      Vy=0;                             //速度清零
      correct_x_flag=1;                 //x方向调整完毕
    }
		  if(correct_y_flag==1 && correct_x_flag==1)
		{
			CSI_correct_flag=1;               //总钻风调整完毕
			correct_x_flag=0;
			correct_y_flag=0;                 //调整标志位清0
			correct_step=1;                   //步数回归到第一步
		}
  }
   break;
}
}
extern uint8 type;
/**
 * @brief 坡道绕行函数，记得要在前面对Angle_ramp,ramp_x,ramp_y清零
 * @param Traverse_distance横移距离
 * @param Straight_distance直行距离
 * @return 无
 */
void ramp_cross(int Traverse_distance, int Straight_distance)
{
    switch(ramp_step)
    {
     case 1://向右横移出赛道
      Turn_Angle_PD(Angle_Z);//锁住现在车头的位置，提供速度Vz
		 if(ramp_x<=Traverse_distance && ramp_step==1)
		 {
      Vx=30;//左移出赛道
		  Vy=0;
		 }
		 else
		 {
			 Vx=0;//停车
		   Vy=0;
		 }
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      if(abs(Traverse_distance-(int)ramp_x)<2)
      {
        ramp_step=2;
      }
     break;
     case 2:
      Turn_Angle_PD(Angle_Z);//锁住现在车头的位置，提供速度Vz
      if(ramp_y<=Straight_distance && ramp_step==2)
		 {
      Vx=0;
		  Vy=30;//前进
		 }
		 else
		 {
			 Vx=0;//穿过坡道
		   Vy=0;
		 }
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      if(abs(Straight_distance-(int)ramp_y)<2)
      {
        ramp_step=3;
      }
       break;
      case 3:
			if(abs((int)ramp_x)<2)
      {
        ramp_step=4;
      }
      Turn_Angle_PD(Angle_Z);//锁住现在车头的位置，提供速度Vz
      if(ramp_x>=0 && ramp_step==3)
		 {
      Vx=-30;//右移回归赛道
		  Vy=0;
		 }
		 else
		 {
			 Vx=0;//停车
		   Vy=0;
		 }
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
			break;
      case 4:
       car_stop();//清空速度
       system_delay_ms(200);
			 ramp_finish=1;
       ramp_step=0;//回归到初始状态,需要使用时再赋值
      break;
  }
}
/**
 * @brief 卡片分类函数
 * @param Traverse_distance横移距离
 * @param Straight_distance直行距离
 * @return 无
 */
void find_classify(int Traverse_class_distance, int Straight_class_distance)
{
  delta_class_x=(Traverse_class_distance)/10+3-(int)Card_dis_car_x; //单位为cm
  delta_class_y=Straight_class_distance/10-32-(int)Card_dis_car_y; //计算出中心坐标,y可能需要调整，参数暂定
    switch(class_step)
    {
			case 1://前进道合适区域
      Turn_Angle_PD(Angle_Z);//锁住现在车头的位置，提供速度Vz
		 if(delta_class_y>3 && class_step==1)//y距离过大，要前进
		 {
      Vx=0;//前进
		  Vy=10;
		 }
		 else if(delta_class_y<0 && class_step==1)//y距离过大，要前进
		 {
			 Vx=0;//停车
		   Vy=-10;
		 }
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      if(delta_class_y<=3 && delta_class_y>=0)
      {
        Vx=0;//停车
		    Vy=0;
        class_step=2;//切换至调整x距离
      }
     break;
     case 2:
      Turn_Angle_PD(Angle_Z);//锁住现在车头的位置，提供速度Vz
      if(delta_class_x>3 && class_step==2)//距离过大，右移
		 {
      Vx=10;//右移
		  Vy=0;//前进
		 }
		 else if(delta_class_x<0 && class_step==2)//超过，左移
		 {
			 Vx=-10;//左移
		   Vy=0;
		 }
     else if(delta_class_x<=3 && delta_class_x>=0 && class_step==2)
     {
			 Vx=0;//
		   Vy=0;//停车
       class_step=3;
		 }
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
       break;
      case 3:
       car_stop();//清空速度
       system_delay_ms(200);
			 classify_correct_finish=1;
       class_step=0;//回归到初始状态,需要使用时再赋值
      break;
  }
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
    if (now_distance_y > 0 && now_distance_y < 800 && abs(now_distance_x)<500) // art识别到卡片，设定识别区间，不能离赛道太远避免识别杂物
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
    if (find_car_flag == 1) // 到达卡片附近
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
        //因为车身姿态与采样频率8596的问题，有且只有一个相交点，给出在符合角度的波动区间
      if(delta_angle-Angle_z<15 && delta_angle-Angle_z>-15)//当底盘坐标需要偏角较大的时候,一般在弯道
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
    if (fabsf(Angle_Z - turn_angle) <= 3) // 陀螺仪转向识别
    {
      // Vz = 0;//清0Vz
			*mode = Car_find_card_cor; //模式转变
      only_one=1;
			car_stop();//清空速度
      correct_x=0;
      correct_y=0;//清零修正的x，y距离
			delta_x=0;
			delta_y=0;
      Angle_arrive_card=0;//清零角度
			pick_up_mode=OPEN;//防止卡死
			correct_art2_flag = OPEN;//打开art2识别中断
      NVIC_SetPriority(LPUART1_IRQn, 2);                                // 降低UART1中断优先级
      arrive_card_flag=OPEN;//打开总钻风微调时的里程计计数
      CSI_correct_flag =  NOT_FINISH;//总钻风调整完毕标志清除
      *mode = Car_find_card_cor; //模式转变
      target_type = *mode;
    }
    else
    {
      Turn_Angle_PD(turn_angle);//准备Vz转速
      Vx=0;
      Vy=0;//x,y静止
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      *mode = Car_turn;
    }
  }
//******************************总钻风对正*****************************//
  if (*mode == Car_find_card_cor) // 总钻风微调识别
  {
    if (CSI_correct_flag == FINISH) // 总钻风坐标对正
    {

      car_stop();//清空速度
			system_delay_ms(500);
      arrive_card_flag=CLOSE;//关闭总钻风微调时的里程计计数
      correct_x=0;
      correct_y=0;//清零修正的x，y距离
			delta_x=0;
			delta_y=0;
      Angle_arrive_card=0;//清零角度
			only_one=OPEN;//重新打开only_one
			CSI_correct_flag=NOT_FINISH; //总钻风微调标志清零
			near_card_x=0;
			near_card_y=0;
			card_center_x=0;
			card_center_y=0;//清空记录的卡片中心坐标
			card_classify=0;
			card_type=0;
      *mode = Pick_up_card;//模式转变
			target_type = *mode;
    }
		else
		{
			   *mode = Car_find_card_cor;
				if(near_card_x!=0 && near_card_y!=0)
				{
//          if(near_card_y>150)//有时候会发错坐标，要设定区间来截取正确的坐标
//          {
					  if(only_one)
				   {
					  card_center_x=near_card_x;
					  card_center_y=near_card_y;
            card_classify=card_type;
					  correct_art2_flag = CLOSE;//立即关闭art4发数据，防止堵塞数据缓冲区
					  only_one=0;//只记录一次
            
				   }
//          }
				}
          CSI_dis_new_correct(card_center_x, card_center_y);//总钻风坐标对正，准备x,y速度
          Turn_Angle_PD(turn_angle);//准备Vz转速，作用是锁住车头方向
          Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
          if(card_classify==1 || card_classify==2 || card_classify==7 || card_classify==13)//交通工具类
          {
            classify_360(Traffic);
            Traffic_count++;
          }
          else if(card_classify==4 || card_classify==5 || card_classify==6 || card_classify==8 ||card_classify==14)//武器类
          {
            classify_360(Weapon);
            Weapon_count++;
          }
          else if(card_classify==3 || card_classify==9 || card_classify==10 || card_classify==11 || card_classify==12 || card_classify==15)//物资类
          {
            classify_360(Supply);
            Supply_count++;
          }
		}
	}
  //******************************卡片拾取*****************************//
    if (*mode == Pick_up_card) // 捡卡片
    {
     if(arm_pick_flag==ARM_PICK_DONE)//卡片已被拾取
	   {
			 correct_x=0;
       correct_y=0;//清零修正的x，y距离,为下一步倒车做准备
			 Angle_arrive_card=0;//清零角度
       arrive_card_flag=OPEN;//开启总钻风微调时的里程计计数
       *mode = Car_turn_again;//模式转变为转向回正
	   }
      else//卡片未被拾取
	   {
		  arm_control(2);//捡卡片
	    arm_control(4);//默认模式
		  arm_pick_flag=ARM_PICK_DONE;//打开中断
      *mode = Pick_up_card;
	   }
    }
  //******************************车头回正*****************************//
    if (*mode == Car_turn_again) 
    {
    if (fabsf(Angle_Z - now_angle) <= 4) // 陀螺仪转向识别
    {
      pick_up_mode=0;     //摄像头变为寻迹模式
			*mode = Car_go; //重新变为寻迹
			car_stop();//清空速度
			system_delay_ms(1000);//停车0.5s
			/*****清空标志位****/
      
      test();
			catch_card_flag=0;  //退出里程计第二种模式
			CSI_correct_flag=0; //总钻风微调标志清零
      find_car_flag=0;    //找到卡片标志位清零
      arrive_card_flag=CLOSE;//关闭里程计修正计数
      arm_pick_flag=ARM_PICK_NOT_DONE;
			pick_up_mode=CLOSE;
			ahead_flag=0;
			/****清空各种坐标和角度****/
			now_distance_x=0;
			now_distance_y=0;
      record_now_distance_x=0;//清空now_distance_x
			record_now_distance_y=0;//清空now_distance_y,避免直接进入模式2
			Angle_arrive_card=0;//清零角度
      correct_x=0;
      correct_y=0;//清零修正的x，y距离
      Card_dis_car_x=0;
      Card_dis_car_y=0;//卡片里程计清空
			Angle_z=0;
			delta_angle=0;//算出的偏移角清0
      card_x[0]=0;//卡片坐标清空
      card_y[0]=0;
			only_one=1;//重新打开only_one
      NVIC_SetPriority(LPUART1_IRQn, 0);//恢复art1的中断优先级
      // uart_write_string(UART_1, uart_1_begin);
      target_type = *mode;
    }
    else
    {
     Turn_Angle_PD(now_angle);//向原先的角度转向回正
		 Vx=0;
		  Vy=Distance_pid(&distance_pid[0], -13, (int)correct_y);//后退
		 Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
     *mode = Car_turn_again;
    }
   }
}
// /**
// * @brief 小车模式切换，打包函数
// * @param mode为模式选择
// * @param
// * @return 无
// */
void car_new_findcard(int *mode)
{
  //******************************正常循迹*****************************//
  if (*mode == Car_go) // 寻迹模式，对赛道进行处理,默认设置
  {
    if (card_position[now_card].card_word_ready==YES) //当卡片数组更新，当前卡片坐标已存入后,可以进入距离判断，避免从原点就开始发癫
    {
        *mode = Car_find_card_y;//转变小车运动模式
				target_type = *mode;//测试变量使用
    }			
     }
		else
		{
      car_run();//正常巡线模式
			*mode=Car_go;
		}
  //******************************找卡片*****************************//
  if (*mode == Car_find_card_y) 
  {
    if(car_card_diatance<35 && car_card_diatance>25) // 到达卡片附近
    {
      car_stop();//清空速度
			system_delay_ms(1000);
      now_angle = Angle_Z;//记录下此时角度
      card_car_x = card_position[now_card].x_distance - (int)Car_dis_x;//世界坐标上卡片与车辆的x距离
      card_car_y = card_position[now_card].y_distance - (int)Car_dis_y;//世界坐标上卡片与车辆的y距离
      card_car_other_angle =atan2( card_car_y, card_car_x)/PI*180;//转换成角度制
      if(abs(card_car_other_angle-(int)Angle_Z) < 4)//当解算出的角度与旋转角Angle_Z相差不大时
      {
        turn_angle=-90+now_angle;//向右转90度 
      }
      if(abs(card_car_other_angle-(int)Angle_Z-180) < 4)//当解算出的角度与旋转角Angle_Z翻转180度后的角度相差不大时,向左转
      {
        turn_angle=90+now_angle;//向左转90度
      }
      *mode = Car_turn; //模式转变
    }
		else
		{
			car_run();//正常寻迹跑
      //实时计算
      car_card_angle = card_world_angle - car_world_angle;//角度制
      car_card_diatance = (int)sqrt((card_position[now_card].world_distance * card_position[now_card].world_distance)					 //卡片与原点距离的平方
                                 +(car_world_distance * car_world_distance)                                                    //车辆与原点距离的平方
                                 -2*card_position[now_card].world_distance*car_world_distance*cos(car_card_angle/180*PI));     //计算这时第now_card张卡片的角度
      *mode = Car_find_card_y;//保持该模式
		}
  }
//******************************向卡片方向转向*****************************//
  if (*mode == Car_turn) 
  {
    if (fabsf(Angle_Z - turn_angle) < 1) // 陀螺仪转向识别
    {
      // Vz = 0;//清0Vz
			car_stop();//清空速度
			system_delay_ms(1000);
      *mode = Car_find_card_cor; // 模式转变
      target_type = *mode;
			pick_up_mode = 1; //打开总钻风识别
    }
    else
    {
      Turn_Angle_PD(turn_angle);//准备Vz转速
      Vx=0;
      Vy=0;//x,y静止
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      *mode = Car_turn;
    }
  }
//******************************总钻风对正*****************************//
  if (*mode == Car_find_card_cor) // 总钻风微调识别
  {
    if (CSI_correct_flag == 1) // 总钻风坐标对正
    {
      car_stop();//清空速度
			system_delay_ms(500);
      *mode = Pick_up_card;//模式转变
			target_type = *mode;
    }
		else
		{
			*mode = Car_find_card_cor;
      CSI_dis_new_correct(center_x, center_y);//总钻风坐标对正，准备x,y速度
      Turn_Angle_PD(turn_angle);//准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
//			system_delay_ms(200);
		}
	}
  //******************************卡片拾取*****************************//
    if (*mode == Pick_up_card) // 捡卡片
    {
     if(arm_pick_flag==ARM_PICK_DONE)//卡片已被拾取
	   {
       catch_card_flag=0;//退出里程计第二种模式
       *mode = Car_turn_again;//模式转变为转向回正
	   }
      else//卡片未被拾取
	   {
		  arm_control(2);//捡卡片
	    arm_control(4);//默认模式
      card_position[now_card].pick_doen_flag=YES;//标记该张卡片已经被拾取完毕
		  arm_pick_flag=ARM_PICK_DONE;
      *mode = Pick_up_card;
	   }
    }
  //******************************车头回正*****************************//
    if (*mode == Car_turn_again) 
    {
    if (fabsf(Angle_Z - now_angle) <= 2) // 陀螺仪转向识别
    {
			*mode = Car_go; //重新变为寻迹
			car_stop();//清空速度
			system_delay_ms(500);//停车0.5s
      now_card++;//开始对比下一张卡片坐标
      target_type = *mode;
    }
    else
    {
     Turn_Angle_PD(now_angle);//向原先的角度转向回正
		 Vy=Distance_pid(&distance_pid[0], -10, (int)correct_y);
		 Vx=0;
		 Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
     *mode = Car_turn_again;
    }
   }
}
// /**
// * @brief 斑马线分类，打包函数
// * @param mode为模式选择
// * @param
// * @return 无
// */
void card_final_classify(int *classify_step)
{
 if(*classify_step==Find_banmaxian && banmaxian_finish==NOT_FINISH)         //找到斑马线,且处理还未完成
 {
   if(abs(Angle_Z-(-Now_angle-90))<3)       //转到了目标角度
   {
     *classify_step=Find_upline;            //转变成上边线寻迹
       once_time=1;                         //重新打开one_time
       correct_x=0;
       correct_y=0;                         //清零修正的x，y距离
			 Angle_arrive_card=0;                 //清零角度
       arrive_card_flag=CLOSE;              //开启总钻风微调时的里程计计数
		//  correct_art2_flag=OPEN;             //打开art4的中断识别
    //  NVIC_SetPriority(LPUART1_IRQn, 2);  // 降低art1的优先级
     classify_type = *classify_step;
   }
   else
   {
     if(once_time)
     {
       Now_angle=Angle_Z;                 //记录下当前角度
       once_time=0;			 
       correct_x=0;
       correct_y=0;                       //清零修正的x，y距离
			 Angle_arrive_card=0;               //清零角度
       arrive_card_flag=OPEN;             //开启总钻风微调时的里程计计数
     }
      Turn_Angle_PD(-90-Now_angle);       //此处可以根据实际情况修改
      // Vy=0;
		  // Vx=Distance_pid(&distance_pid[0], -10, (int)correct_x); //后退
		  Car_Inverse_kinematics_solution(Vx, Vy, Vz);            //麦轮控制，为target_speed赋值
     *classify_step=Find_banmaxian;
   }
 }
 if(*classify_step==Find_upline)//切换至上边线寻迹
 {
   if(now_distance_y > 200 && now_distance_y < 800 && now_distance_x<250 && now_distance_x>80)//art1识别到坐标,设置识别区间为右中平面，实在不行就直接上世界坐标解算来判断
   {
     num_card_x = now_distance_x;     //记录看到的x坐标
     num_card_y = now_distance_y;     //记录看到的y坐标
     catch_card_flag=OPEN;            //打开卡片捕获的里程计
     Angle_z=0;                       //清零角度
     Card_dis_car_x=0;
     Card_dis_car_y=0;                //卡片里程计清空
     class_step=1;                    //卡片分类区域修正步数初始化
     *classify_step=Catch_card;       //向卡片分类区域前进
     classify_type = *classify_step;
   }
   else if(Traffic_Finish==FINISH && Supply_Finish==FINISH && Weapon_Finish==FINISH)//三类区域都已经识别完成
   {
      *classify_step=Turn_back;        //转回正常寻迹
      Card_dis_car_x=0;
      Card_dis_car_y=0;                //卡片里程计清空
   }
   else
   {
    //方案一，上边线巡线，鲁棒性好
     car_run_upline();///上边线寻迹
     Turn_Angle_PD(-90-Now_angle);                //此处可以根据实际情况修改，提供Vz的车头修正速度
     Car_Inverse_kinematics_solution(Vx, Vy, Vz); //麦轮控制，为target_speed赋值
     *classify_step=Find_upline;
   }
 }
 if(*classify_step==Catch_card)					//寻找卡片分类区域
 {
   if(classify_correct_finish==1)				//到达了数字分类卡片区域，准备识别
   {
		 NVIC_SetPriority(LPUART1_IRQn, 2);	//降低art1的中断优先级
     classify_art2_flag=OPEN;  				 	//打开art4中断标志
     *classify_step=Watch_card;					//识别卡片上的数字
     classify_correct_finish=0;					//清除调整完毕的标志位
     num_card_x = 0;          					//清零记录数字卡片的x坐标
     num_card_y = 0;          					//清零记录数字卡片的y坐标
     now_distance_x=0;									//x坐标清零
		 now_distance_y=0;        				 	//y坐标清零
		 once_time=1;												//打开once_time
     classify_type = *classify_step;
   }
   else
   {
     Turn_Angle_PD(-90-Now_angle); //此处可以根据实际情况修改，提供Vz的车头修正速度
     find_classify(num_card_x, num_card_y);
     *classify_step=Catch_card;
   }
 }
  if(*classify_step==Watch_card)  //识别卡片分类区域
 {
   if(Find_num==READY)        //找到了数字分类卡片区域，准备识别
   {
     *classify_step=Putout_card;  //放卡片
      card_num=0;                 //清空已识别的数字
      numcard_classify=0;         //清空已记录的数字
      Find_num=NOT_READY;        //重置识别完毕的标志位
      once_time=1;                //重新打开once_time
     classify_type = *classify_step;
   }
   else
   {
      *classify_step = Watch_card;
			if(card_num!=0)                 //识别到卡片类型
				{
					if(once_time)
				 {
           numcard_classify=card_num; //数字类型
           Find_num=READY;            //识别出了卡片类型
					 classify_art2_flag = CLOSE;//立即关闭art4发数据，防止堵塞数据缓冲区
					 once_time=0;               //只记录一次
         } 
				}
      if(numcard_classify==1)     //武器放置类
      {
        classify_360(Weapon);
        put_out_count=Weapon_count;
        Weapon_Finish=FINISH;     //完成武器类的舵机转向
      }
      else if(numcard_classify==2)//物资放置类
      {
        classify_360(Supply);
        put_out_count=Supply_count;
        Supply_Finish=FINISH;     //完成物资类的分类
      }
      else if(numcard_classify==3)//交通工具类
      {
        classify_360(Traffic);
        put_out_count=Traffic_count;
        Traffic_Finish=FINISH;         //完成交通类的舵机转向
      }
   }
 }
 if(*classify_step==Putout_card)   //识别卡片分类区域
	{
		if(put_out_card_flag==FINISH)  //所有卡片均放出
    {
      put_out_card_flag=NOT_FINISH;
      put_out_count=0;				 //清零需要放出的卡片数目
      *classify_step=Go_back;  //后退
      classify_type = *classify_step;
    }
    else
    {
      for(uint8 i=put_out_count; i>0; i--)
      {
        arm_control(3); //放卡片
        if(i==1)        //最后一张
         put_out_card_flag=FINISH;
      }
      *classify_step=Putout_card;  //放卡片
    }
	}
 if(*classify_step==Go_back)   //后退至原本可用art1识别的位置
	{
		if(Card_dis_car_y<=2 && Card_dis_car_y>=-1)                 //设置容错区间
    {
      *classify_step = Find_upline;                             //继续向右平移找卡片
      classify_type = *classify_step;
      NVIC_SetPriority(LPUART1_IRQn, 0);	                      //恢复art1的中断优先级
      /******************清零各种变量***************/
      now_distance_x=0;									
		  now_distance_y=0;        				 	                        //此处清零主要是为了防止直接状态二判断成功，乱识别东西
      Card_dis_car_x=0;
      Card_dis_car_y=0;                                         //卡片里程计清空
      /******************关闭各种标志位*************/
      catch_card_flag=CLOSE;
    }
    else
    {
      *classify_step = Go_back;
      Vx=0;
      Turn_Angle_PD(-90-Now_angle);                             //此处可以根据实际情况修改，提供Vz的车头修正速度
      Vy=Distance_pid(&distance_pid[0], 0, (int)Card_dis_car_y);//向后退
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);              //麦轮控制，为target_speed赋值
    }
	}
  if(*classify_step==Turn_back)//转向回正
  {
   if(abs(Angle_Z-Now_angle)<2)
   {
     *classify_step=Find_banmaxian;        //返回至初始状态，直到下一次重新进去
     classify_type = *classify_step;
     Card_dis_car_x=0;
     Card_dis_car_y=0;
     banmaxian_finish=FINISH; //斑马线处理完成

   }
   else
   {
     Vx=0;
     Vy=0;
     Turn_Angle_PD(Now_angle);//转向回正
     *classify_step=Turn_back;
   }
  }
}
// /**
// * @brief 左环岛单搬策略
// * @param Island_step为步数选择
// * @param 该函数是判断出为左环岛类型时才使用
// * @return 无
// */
 int Left_Island_classify_zone_x;
 int Left_Island_classify_zone_y;
 int delta_find_Island_zero_x,delta_find_Island_zero_y;
 int delta_find_Island_zero_angle; 
 float now_Island_angle;
 float turn_IsLand_angel;
 void Left_Island_pick_and_move(int *Island_step)
 {
 if(*Island_step==Catch_zeropoint)                //找到环岛区域
 {
   if(now_distance_x>500 && now_distance_y>800)   //识别出了在左环岛最右侧的卡片
   {
     Left_Island_classify_zone_x = now_distance_x;
     Left_Island_classify_zone_y = now_distance_y;//记录识别出的环岛外分类区域的卡片坐标
     Angle_Island = 0;                            //第一次用来对停车的位置做一个初步定位
     Island_classify_flag=OPEN;                   //打开环岛里程计模式
     Island_x=0;
     Island_y=0;                                  //环岛里程计清零
     *Island_step=Arrive_zeropoint;               //切换模式
   }
   else
   {
     car_run();
     *Island_step=Catch_zeropoint;
   }
 }
  if(*Island_step==Arrive_zeropoint)              //去环岛的原点处
 {
   if(abs(delta_find_Island_zero_angle)<5)        //到达与最右侧的区域的环岛区域
   {
     car_stop();
     system_delay_ms(500);
     now_Island_angle = Angle_Z;      //记录下环岛转向前的角度
     turn_IsLand_angel = Angle_Z- 90; //向右转向90度
     *Island_step=Car_Island_turn;    //向圆环外部区域转向
   }
   else
   {
     car_run();
     delta_find_Island_zero_x = Left_Island_classify_zone_x - Island_x;
     delta_find_Island_zero_y = Left_Island_classify_zone_y - Island_y;
     delta_find_Island_zero_angle = atan2(delta_find_Island_zero_y, delta_find_Island_zero_x);
     *Island_step=Arrive_zeropoint;
   }
 }
   if(*Island_step==Car_Island_turn)           //车头转向环岛外侧
 {
   if(abs(turn_IsLand_angel-Angle_Z)<3)        //摄像头朝向环岛的外侧
   {
     car_stop();
     system_delay_ms(500);
     *Island_step=Car_Island_Find_Upline;     //用上边线寻迹进入圆环
   }
   else
   {
     Turn_Angle_PD(turn_IsLand_angel);
     Vx=0;
     Vy=0;
     *Island_step=Car_Island_turn;
   }
 }
 }