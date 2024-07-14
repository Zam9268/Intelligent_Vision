#include "zf_common_headfile.h"
#include "control.h"
#include "imu660ra.h"
#include "camera.h"
#include "image.h"
#include "math.h"
#include "take.h"
#include "communication.h"
extern uint8 normal_stop_flag;
extern uint8 card_left_up_find_flag; // the lef up corner of the card lying on the side of the road is found
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
extern char uart_1_begin[]; // UART4开始字符串
extern char uart_1_stop[];  // UART4开始字符串abc

uint8 record_abc_card_type = 0; // 记录的字母卡片类型

/******************坡道绕行函数所需变量*****************/
uint8 find_ramp;       // 用于切换坡道里程计调整
uint8 ramp_step;       // 用于坡道绕行函数的步数调整
uint8 ramp_finish = 0; // 坡道绕行完成标志
uint8 stop_detect_flag=0;//十字放置重复检测标志位
uint8 island_stop_flag=0;//环岛放置重复检测标志位
uint8 normal_stop_flag=0;
uint8 delay_place_flag=0;
/*******************************************************/
uint8 chance = 0;
float Vx, Vy, Vz;
/**************差速所用变量*************************/
float left_top_kp = 0.6f;
float left_top_kd = 0.30f;                              // 左上边线kp，kd
float right_top_kp = 0.76f;
float right_top_kd = 0.25f;                             // 右上边线kp,kd
float err_watch;
float move_error;
float angle;
float right_hengyi_angle;
float left_hengyi_angle;
float ahead_speed = 40.0; // 直行速度
float target_all_speed = 10.00; // 速度
float target_upline_speed = -6; // 上边线速度
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
float turn_error = 3; // 可接受的角度误差
float Turn_KP = 0.5;  // 角度PID//
float Turn_KD = 0.5;  // 角度PID//
// float Turn_KI[1] = {30};  //角度PID//5
/************************************************************/
//*****************里程计所用变量****************//
float Car_dis_x1, Car_dis_y1; //
float Car_dis_x2, Car_dis_y2;

uint8 Island_classify_flag = 0;
uint8 Island_back_flag = 0;
uint8 Crossing_classify_flag = CLOSE;
uint8 Zebra_classify_flag = CLOSE;
uint8 Zebra_dis_flag = CLOSE;

float Vx_1, Vx_2, Vy_1, Vy_2;                                 // 对里程的cos，sin分解
float Vx_car_1, Vx_car_2, Vy_car_1, Vy_car_2;                 // 对底盘坐标的cos，sin分解
float Vx_correct_1, Vx_correct_2, Vy_correct_1, Vy_correct_2; // 用于总钻风微调的里程计
float Vx_ramp_1, Vx_ramp_2, Vy_ramp_1, Vy_ramp_2;             // 用于坡道绕行
float Vx_Island_1, Vx_Island_2, Vy_Island_1, Vy_Island_2;     // 用于环岛分类
float Vx_Island_back_1, Vx_Island_back_2, Vy_Island_back_1, Vy_Island_back_2;//用于环岛后退
float Vx_Crossing_1, Vx_Crossing_2, Vy_Crossing_1, Vy_Crossing_2;     // 用于十字分类
float Vx_Zebra_1, Vx_Zebra_2, Vy_Zebra_1, Vy_Zebra_2;     // 用于斑马线分类
float Vx_Zebra_dis_1,Vx_Zebra_dis_2,Vy_Zebra_dis_1,Vy_Zebra_dis_2;//用于斑马线掉头

float Vx_correct, Vy_correct; // 用于总钻风的里程计
float Vx_world, Vy_world;     // 世界坐标上的x，y
float Vx_card, Vy_card;       // 相对于车底盘的更新坐标
float Vx_ramp, Vy_ramp;       // 坡道绕行时使用的速度
float Vx_Island, Vy_Island;   // 环岛分类合成的速度
float Vx_Island_back,Vy_Island_back;//环岛后退的速度
float Vx_Crossing,Vy_Crossing;      //十字分类的速度
float Vx_Zebra,Vy_Zebra;       //斑马线分类合成的速度
float Vx_Zebra_dis,Vy_Zebra_dis;//用于斑马线掉头的速度

float Card_dis_car_x = 0;
float Card_dis_car_y = 0;   // 相对于车的更新坐标
float Car_dis_x, Car_dis_y; // x轴，y轴行走距离
float correct_x, correct_y; // 修正的x和y
float ramp_x, ramp_y;       // 坡道绕行的里程计
float Island_x, Island_y;   // 环岛绕行的里程计
float Island_back_x, Island_back_y;   // 环岛绕行的里程计
float Crossing_x,Crossing_y;          //用于十字分类
float Zebra_x,Zebra_y;                //用于斑马线分类
float Zebra_dis_x,Zebra_dis_y;        //用于斑马线掉头

//***********************************************//
int car_world_distance; // 车辆在全局坐标上与原点的距离
float car_world_angle;  // 卡片世界坐标解算出的世界方位角
float Turn_Bias;
/****************距离环所需变量*****************/
float dis_kp = 1.5; // 距离环kp
float dis_kd = 0.4; // 距离环kd
float dis_error;
float dis_change[4]; // 存放距离环输出结果
/***********************************************/
/***************************************总的打包函数所需变量*************************************************/
uint8 find_card_allow = READY; //初始化为常开
int card_y[10];      // 存放卡片y轴坐标
int card_x[10];      // 存放卡片y轴坐标
float card_distance; // 存放卡片的合成距离
uint8 Find_Card_Allow_Flag = READY;//允许寻找卡片的标志位
int only_one = 1;
int target_type = 0;               // 测试使用,观察模式
float delta_x, delta_y;            // 总钻风识别的卡片中心坐标
int now_count = 0;                 // 现在是第几张卡片
int CSI_correct_flag = 0;          // 总钻风判断标志
int Put_flag = 0;                  // 图片放置标志位
int test_csi;                      // 延时计数
int car_mode = 0;                  // 车辆运动模式
float now_angle = 0;               // 转向前的初始角度，默认为0
float turn_angle = 0;              // 转向模式时的目标转向角度，默认为0
float card_angle = 0;              // 卡片的解算角度
int catch_card_flag = 0;           // 捕获到卡片的标志位
int arrive_card_flag;              // 打开修正里程计标志
int find_car_flag = 0;             // 到达卡片位置的标志位
int correct_art2_flag;             // 打开art4的中断标志位
double delta_card_y, delta_card_x; // 卡片x,y坐标与新y里程和x里程的差值
double delta_angle;                // 计算出来的即时偏转角
float ahead_distance;              // 前进的距离
/**************************************************************************************************************/
/*********************用于卡片分类的变量*****************************/
uint8 Traffic = 1;     // 交通工具类
uint8 Weapon = 2;      // 武器类
uint8 Supply = 3;      // 物资类
int Traffic_count = 0; // 拾取的交通工具卡片数
int Weapon_count = 0;  // 拾取的武器总卡片数
int Supply_count = 0;  // 拾取的物资的总卡片数
/********************************************************************/
/******************用于总钻风修正的变量*******************/
int card_center_x;
int card_center_y;       // 卡片中心坐标
uint8 card_classify = 0; // 记录卡片的分类
int correct_x_flag = 0;
int correct_y_flag = 0;
int correct_step = 1; // 校正步数
// int ahead_flag = 0;
/*********************************************************/
//*********新的行进函数***********/
int now_card = 0;             // 当前的卡片
int card_car_x, card_car_y;   // 世界坐标下卡片与车辆的x坐标差值和y坐标差值
int card_car_other_angle = 0; // 由上面两个差值解算出的角度
/*********************************/
/*********************用于上边线寻迹所需的变量*****************************/
float top_error,last_top_error;//与上边线整线的的误差与上次误差
float right_top_error, last_right_top_error; // 与目标行数的加权误差
float left_top_error, last_left_top_error;   // 与目标行数的加权误差
/************************************************************************/
//*********斑马线分类函数***************/
int car_run_mode = 0; // 总的行进函数的选择
uint8 banmaxian_allow_flag = READY;
int once_time = 1;
int classify_mode = 0;
float Now_angle;
int num_card_x, num_card_y;
uint8 class_step = 1;
uint8 classify_correct_finish = 0;
uint8 numcard_classify = 0;
int delta_class_x, delta_class_y;
int classify_art2_flag;
int classify_type;           // 用于观察分类的模式
uint8 Find_num = 0;          // 识别完毕的标志位
int put_out_count;           // 需要放出的卡片数目
uint8 put_out_card_flag = 0; // 所有卡片是否放出的标志位
uint8 Traffic_Finish = 0;    // 交通工具类
uint8 Weapon_Finish = 0;     // 武器类
uint8 Supply_Finish = 0;     // 物资类
int banmaxian_finish;        // 斑马线处理完成与否的标志位
uint8 Longest_Column_Fixed = 0;
/*************************************/
/**************十字分类函数************/
uint8 Cross_Allow_flag = READY;               // 十字允许标志位
uint8 Island_Allow_flag = READY;              // 环岛允许标志位
uint8 Zebra_Allow_flag = READY;               // 斑马线允许标志位
float delta_crossing_x, delta_crossing_y;     // 用于十字矫正的delta里程
uint8 CSI_crossing_correct_flag = NOT_FINISH; // art1十字对准的标志位
float delta_crossing_class_x;
float delta_crossing_class_y; // 十字区域调整的delta
/*************************各种pid*****************************/
int pid_motor[4];

pid_info Pos_turn_pid[4]; // 位置式pid

pid_info Angle_turn_pid; // 角度环pid

pid_info distance_pid[4]; // 距离环pid
/*************************************************************/
/*************************环岛和十字卡片结构体数组****************************/
card Island_card[5];

card cross_card[5];

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
  gpio_init(DIR_RF, GPO, GPIO_LOW, GPO_PUSH_PULL);  //
  gpio_init(DIR_RB, GPO, GPIO_LOW, GPO_PUSH_PULL);  //

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

  float kp = 1.0f, kd = 0.5f; // 1.0对应速度30  0.9响应10
  

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
//	循迹平移
// target_row为目标行误差，只提供Vx，Vy速度
void car_crosswise_right_run(int target_row)
{
	Top_Line_x_Search();                 // 对上边线扫线，为上边线数组做准备
  top_error = Top_Line_x_Err_Right(target_row)/25; // 对上边线扫线输出一个误差,作归一化处理
  if (top_error > 1.0f)
 {
   top_error = 1.0f;
 }
 else if (top_error < -1.0f)
 {
   top_error = -1.0f;
 }
  float top_kp = 1.0f, top_kd = 0.5f;                             // 1.0对应速度30  0.9响应10
  float Vx = 10.0;
  Vy = top_kp * top_error + top_kd * (top_error - last_top_error); // 输出为Vy的速度
  last_top_error = top_error;                                      // 记录下上次误差
  Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
}
/**
 * @brief 对速度预处理,平移右跑
 * @param 上边线误差，取前瞻 24/7/9 4:00(这b车是真不想调了)
 * @return 无
 */
void car_run_upline_right(int target_line)
{
  Top_Line_Search();                                      // 对上边线扫线，为上边线数组做准备
  right_top_error = Top_Line_Err_Right(target_line) / 25; // 对上边线扫线输出一个误差,作归一化处理
  if (right_top_error > 1.0f)
 {
   right_top_error = 1.0f;
 }
 else if (right_top_error < -1.0f)
 {
   right_top_error = -1.0f;
 }
  float right_top_kp = 0.76f, right_top_kd = 0.25f;                              // 1.0对应速度30  0.9响应10
   float target_upline_speed = 6;
 right_hengyi_angle= right_top_kp * right_top_error + right_top_kd * (right_top_error - last_right_top_error); // 输出为Vy速度
 Speed[0].target_speed = target_upline_speed * (1 - right_hengyi_angle);
 Speed[1].target_speed = -target_upline_speed * (1 + right_hengyi_angle);
 Speed[2].target_speed = -target_upline_speed * (1 - right_hengyi_angle);
 Speed[3].target_speed = target_upline_speed * (1 + right_hengyi_angle);
  last_right_top_error = right_top_error;                                      // 记录下上次误差
}
/**
 * @brief 对速度预处理,平移左跑
 * @param 上边线误差，取前瞻 24/7/9 4:00(这b车是真不想调了)
 * @return 无
 */
void car_run_upline_left(int target_line)
{
  Top_Line_Search();                                    // 对上边线扫线，为上边线数组做准备
  left_top_error = Top_Line_Err_Left(target_line) / 25; // 对上边线扫线输出一个误差,作归一化处理
  if (left_top_error > 1.0f)
 {
   left_top_error = 1.0f;
 }
 else if (left_top_error < -1.0f)
 {
   left_top_error = -1.0f;
 }
  float left_top_kp = 0.6f, left_top_kd = 0.30f;                              // 1.0对应速度30  0.9响应10
   float target_upline_speed = -6;
 left_hengyi_angle= left_top_kp * left_top_error + left_top_kd * (left_top_error - last_left_top_error); // 输出为Vy速度
 Speed[0].target_speed = target_upline_speed * (1 - left_hengyi_angle);
 Speed[1].target_speed = -target_upline_speed * (1 + left_hengyi_angle);
 Speed[2].target_speed = -target_upline_speed * (1 - left_hengyi_angle);
 Speed[3].target_speed = target_upline_speed * (1 + left_hengyi_angle);
  last_left_top_error = left_top_error;                                      // 记录下上次误差
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
  Speed[1].ki = -3.3;   //-3.3  -0.98
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

  if (pid_motor[1] > 0) // 左后轮
  {
    gpio_set_level(DIR_LB, 0);
    pwm_set_duty(motor_LB, (int)pid_motor[1]);
  }
  else //???
  {
    gpio_set_level(DIR_LB, 1);
    pwm_set_duty(motor_LB, (int)-pid_motor[1]);
  }

  if (pid_motor[2] > 0) // 右前轮，正转
  {
    gpio_set_level(DIR_RF, 1); // 0
    pwm_set_duty(motor_RF, (int)pid_motor[2]);
  }
  else // 反转
  {
    gpio_set_level(DIR_RF, 0); // 1
    pwm_set_duty(motor_RF, (int)-pid_motor[2]);
  }

  if (pid_motor[3] > 0) // 右后轮
  {
    gpio_set_level(DIR_RB, 1); // 正转
    pwm_set_duty(motor_RB, (int)pid_motor[3]);
  }
  else //???
  {
    gpio_set_level(DIR_RB, 0); // 反转
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
  static float Angle_bias = 0;     // 用于识别到卡片时候的角度误差
  static float Angle_correct_bias; // 用于总钻风修正
  static float Angle_ramp_bias;    // 用于坡道绕行
  static float Angle_Island_bias;  // 用于环岛分类
  static float Angle_Island_back_bias;//用来后退
  static float Angle_Crossing_bias;   //用于十字分类
  static float Angle_Zebra_bias;   //用于斑马线分类
  static float Angle_Zebra_dis_bias;//用于斑马线掉头 
  static float V_enco[4] = {0}, Vx_enco = 0, Vy_enco = 0;

  Angle_Bias = Angle_Z * PI / 180; // 转换成弧度制，Angle_Z为转向角度

  V_enco[0] = 0.5273438 * PI * encoder[0]; // 0.2637可以再精确多三位，计算车轮路程
  V_enco[1] = 0.5273438 * PI * encoder[1];
  V_enco[2] = 0.5273438 * PI * encoder[2];
  V_enco[3] = 0.5273438 * PI * encoder[3];

  Vx_enco = (V_enco[0] - V_enco[1] - V_enco[2] + V_enco[3]) / 4; // 前进为正，根据麦轮速度解算公式得出的底盘x轴位移量
  Vy_enco = (V_enco[0] + V_enco[1] + V_enco[2] + V_enco[3]) / 4; // 左移为正，根据麦轮速度解算公式得出的底盘y轴位移量

#if 1                  // 默认设置
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
  Car_dis_x += Vx_world * 0.005; // 用于全局坐标
  Car_dis_y += Vy_world * 0.005;

  car_world_distance = sqrt(Car_dis_x * Car_dis_x + Car_dis_y * Car_dis_y); // 车辆与原点的距离
  car_world_angle = atan2(Car_dis_y, Car_dis_x) / PI * 180 * 1.0;           // 角度制，车辆相对于原点解算出来的角度
  /***********************************************************/
  car_card_angle = card_world_angle - car_world_angle;                      // 角度制
  car_card_diatance = (int)sqrt((card_world_distance * card_world_distance) // 卡片与原点距离的平方
                                + (car_world_distance * car_world_distance) // 车辆与原点距离的平方
                                - 2 * card_world_distance * car_world_distance * cos(car_card_angle / 180 * PI));

  Car_dis_x2 += Vx_world * 0.005; // 用于赛道回正
  Car_dis_y2 += Vy_world * 0.005;
  /**************************搜索卡片的时候使用********************************/
 else if (catch_card_flag == OPEN) // 捕获到卡片时的里程计
  {
    Angle_bias = Angle_z * PI / 180;

    if (Angle_bias >= 0) // 旋转角度(参照x轴)大于0时
    {
      Vx_car_1 = Vx_enco * sin(Angle_bias);
      Vx_car_2 = Vx_enco * cos(Angle_bias);
      Vy_car_1 = Vy_enco * cos(Angle_bias);
      Vy_car_2 = Vy_enco * sin(Angle_bias); // 分解到车辆底盘坐标上
      Vx_card = Vx_car_2 - Vy_car_2;        // 简单的分解计算
      Vy_card = Vx_car_1 + Vy_car_1;
    }
    if (Angle_bias < 0)
    {
      Angle_bias = -Angle_bias;
      Vx_car_1 = Vx_enco * sin(Angle_bias);
      Vx_car_2 = Vx_enco * cos(Angle_bias);
      Vy_car_1 = Vy_enco * cos(Angle_bias);
      Vy_car_2 = Vy_enco * sin(Angle_bias); // 分解到车辆底盘坐标上
      Vx_card = Vx_car_2 + Vy_car_2;        // 简单的分解计算
      Vy_card = -Vx_car_1 + Vy_car_1;
    }
    Card_dis_car_x += Vx_card * 0.005;
    Card_dis_car_y += Vy_card * 0.005; // 分解出卡片所需的里程，用于找卡片
  }
  /*************************总钻风调整时使用***************************************/
  else if (arrive_card_flag == OPEN) // 总钻风微调，总钻风微调时的里程计
  {
    Angle_correct_bias = Angle_arrive_card * PI / 180;

    if (Angle_correct_bias >= 0) // 旋转角度(参照x轴)大于0时
    {
      Vx_correct_1 = Vx_enco * sin(Angle_correct_bias);
      Vx_correct_2 = Vx_enco * cos(Angle_correct_bias);
      Vy_correct_1 = Vy_enco * cos(Angle_correct_bias);
      Vy_correct_2 = Vy_enco * sin(Angle_correct_bias); // 用于总钻风微调
      Vx_correct = Vx_correct_2 - Vy_correct_2;         // 简单的分解计算
      Vy_correct = Vx_correct_1 + Vy_correct_1;
    }
    if (Angle_correct_bias < 0)
    {
      Angle_correct_bias = -Angle_correct_bias;
      Vx_correct_1 = Vx_enco * sin(Angle_correct_bias);
      Vx_correct_2 = Vx_enco * cos(Angle_correct_bias);
      Vy_correct_1 = Vy_enco * cos(Angle_correct_bias);
      Vy_correct_2 = Vy_enco * sin(Angle_correct_bias); // 用于总钻风微调
      Vx_correct = Vx_correct_2 + Vy_correct_2;         // 简单的分解计算
      Vy_correct = -Vx_correct_1 + Vy_correct_1;
    }
    correct_x += Vx_correct * 0.005;
    correct_y += Vy_correct * 0.005; // 分解出卡片所需的里程，用于找卡片
  }
  /***************************坡道绕行时使用***************************************/
  else if (find_ramp == OPEN) // 坡道调整
  {
    Angle_ramp_bias = Angle_ramp * PI / 180;

    if (Angle_ramp_bias >= 0) // 旋转角度(参照x轴)大于0时
    {
      Vx_ramp_1 = Vx_enco * sin(Angle_ramp_bias);
      Vx_ramp_2 = Vx_enco * cos(Angle_ramp_bias);
      Vy_ramp_1 = Vy_enco * cos(Angle_ramp_bias);
      Vy_ramp_2 = Vy_enco * sin(Angle_ramp_bias); // 用于坡道调整
      Vx_ramp = Vx_ramp_2 - Vy_ramp_2;            // 简单的分解计算
      Vy_ramp = Vx_ramp_1 + Vy_ramp_1;
    }
    if (Angle_ramp_bias < 0)
    {
      Angle_ramp_bias = -Angle_ramp_bias;
      Vx_ramp_1 = Vx_enco * sin(Angle_ramp_bias);
      Vx_ramp_2 = Vx_enco * cos(Angle_ramp_bias);
      Vy_ramp_1 = Vy_enco * cos(Angle_ramp_bias);
      Vy_ramp_2 = Vy_enco * sin(Angle_ramp_bias); // 用于总钻风微调
      Vx_ramp = Vx_ramp_2 + Vy_ramp_2;            // 简单的分解计算
      Vy_ramp = -Vx_ramp_1 + Vy_ramp_1;
    }
    ramp_x += Vx_ramp * 0.005;
    ramp_y += Vy_ramp * 0.005; // 分解出卡片所需的里程，用于找卡片
  }
  /***************************环岛分类时使用***************************************/
  else if (Island_classify_flag == OPEN) // 环岛调整
  {
    Angle_Island_bias = Angle_Island * PI / 180;

    if (Angle_Island_bias >= 0) // 旋转角度(参照x轴)大于0时
    {
      Vx_Island_1 = Vx_enco * sin(Angle_Island_bias);
      Vx_Island_2 = Vx_enco * cos(Angle_Island_bias);
      Vy_Island_1 = Vy_enco * cos(Angle_Island_bias);
      Vy_Island_2 = Vy_enco * sin(Angle_Island_bias); // 用于坡道调整
      Vx_Island = Vx_Island_2 - Vy_Island_2;          // 简单的分解计算
      Vy_Island = Vx_Island_1 + Vy_Island_1;
    }
    if (Angle_Island_bias < 0)
    {
      Angle_Island_bias = -Angle_Island_bias;
      Vx_Island_1 = Vx_enco * sin(Angle_Island_bias);
      Vx_Island_2 = Vx_enco * cos(Angle_Island_bias);
      Vy_Island_1 = Vy_enco * cos(Angle_Island_bias);
      Vy_Island_2 = Vy_enco * sin(Angle_Island_bias); // 用于总钻风微调
      Vx_Island = Vx_Island_2 + Vy_Island_2;          // 简单的分解计算
      Vy_Island = -Vx_Island_1 + Vy_Island_1;
    }
    Island_x += Vx_Island * 0.005;
    Island_y += Vy_Island * 0.005; // 分解出卡片所需的里程，用于找卡片
  }
  else if (Island_back_flag == OPEN) // 环岛调整
  {
    Angle_Island_back_bias = Angle_Island_back * PI / 180;

    if (Angle_Island_back_bias >= 0) // 旋转角度(参照x轴)大于0时
    {
      Vx_Island_back_1 = Vx_enco * sin(Angle_Island_back_bias);
      Vx_Island_back_2 = Vx_enco * cos(Angle_Island_back_bias);
      Vy_Island_back_1 = Vy_enco * cos(Angle_Island_back_bias);
      Vy_Island_back_2 = Vy_enco * sin(Angle_Island_back_bias); // 用于坡道调整
      Vx_Island_back = Vx_Island_back_2 - Vy_Island_back_2;     // 简单的分解计算
      Vy_Island_back = Vx_Island_back_1 + Vy_Island_back_1;
    }
    if (Angle_Island_back_bias < 0)
    {
      Angle_Island_back_bias = -Angle_Island_back_bias;
      Vx_Island_back_1 = Vx_enco * sin(Angle_Island_back_bias);
      Vx_Island_back_2 = Vx_enco * cos(Angle_Island_back_bias);
      Vy_Island_back_1 = Vy_enco * cos(Angle_Island_back_bias);
      Vy_Island_back_2 = Vy_enco * sin(Angle_Island_back_bias); // 用于总钻风微调
      Vx_Island_back = Vx_Island_back_2 + Vy_Island_back_2;     // 简单的分解计算
      Vy_Island_back = -Vx_Island_back_1 + Vy_Island_back_1;
    }
    Island_back_x += Vx_Island_back * 0.005;
    Island_back_y += Vy_Island_back * 0.005; // 分解出卡片所需的里程，用于找卡片
  }
  if (Crossing_classify_flag == OPEN)
  {
    Angle_Crossing_bias = Angle_Crossing * PI / 180;

    if (Angle_Crossing_bias >= 0) // 旋转角度(参照x轴)大于0时
    {
      Vx_Crossing_1 = Vx_enco * sin(Angle_Crossing_bias);
      Vx_Crossing_2 = Vx_enco * cos(Angle_Crossing_bias);
      Vy_Crossing_1 = Vy_enco * cos(Angle_Crossing_bias);
      Vy_Crossing_2 = Vy_enco * sin(Angle_Crossing_bias); // 用于坡道调整
      Vx_Crossing = Vx_Crossing_2 - Vy_Crossing_2;        // 简单的分解计算
      Vy_Crossing = Vx_Crossing_1 + Vy_Crossing_1;
    }
    if (Angle_Crossing_bias < 0)
    {
      Angle_Crossing_bias = -Angle_Crossing_bias;
      Vx_Crossing_1 = Vx_enco * sin(Angle_Crossing_bias);
      Vx_Crossing_2 = Vx_enco * cos(Angle_Crossing_bias);
      Vy_Crossing_1 = Vy_enco * cos(Angle_Crossing_bias);
      Vy_Crossing_2 = Vy_enco * sin(Angle_Crossing_bias); // 用于总钻风微调
      Vx_Crossing = Vx_Crossing_2 + Vy_Crossing_2;        // 简单的分解计算
      Vy_Crossing = -Vx_Crossing_1 + Vy_Crossing_1;
    }
    Crossing_x += Vx_Crossing * 0.005;
    Crossing_y += Vy_Crossing * 0.005; // 分解出卡片所需的里程，用于找卡片
  }
  if (Zebra_classify_flag == OPEN)
  {
    Angle_Zebra_bias = Angle_Zebra * PI / 180;

    if (Angle_Zebra_bias >= 0) // 旋转角度(参照x轴)大于0时
    {
      Vx_Zebra_1 = Vx_enco * sin(Angle_Zebra_bias);
      Vx_Zebra_2 = Vx_enco * cos(Angle_Zebra_bias);
      Vy_Zebra_1 = Vy_enco * cos(Angle_Zebra_bias);
      Vy_Zebra_2 = Vy_enco * sin(Angle_Zebra_bias); // 用于坡道调整
      Vx_Zebra = Vx_Zebra_2 - Vy_Zebra_2;           // 简单的分解计算
      Vy_Zebra = Vx_Zebra_1 + Vy_Zebra_1;
    }
    if (Angle_Zebra_bias < 0)
    {
      Angle_Zebra_bias = -Angle_Zebra_bias;
      Vx_Zebra_1 = Vx_enco * sin(Angle_Zebra_bias);
      Vx_Zebra_2 = Vx_enco * cos(Angle_Zebra_bias);
      Vy_Zebra_1 = Vy_enco * cos(Angle_Zebra_bias);
      Vy_Zebra_2 = Vy_enco * sin(Angle_Zebra_bias); // 用于总钻风微调
      Vx_Zebra = Vx_Zebra_2 + Vy_Zebra_2;           // 简单的分解计算
      Vy_Zebra = -Vx_Zebra_1 + Vy_Zebra_1;
    }
    Zebra_x += Vx_Zebra * 0.005;
    Zebra_y += Vy_Zebra * 0.005; // 分解出卡片所需的里程，用于找卡片
  }
  if(Zebra_dis_flag == OPEN)
   {
    Angle_Zebra_dis_bias = Angle_Zebra_dis * PI / 180;

    if (Angle_Zebra_dis_bias >= 0) // 旋转角度(参照x轴)大于0时
    {
      Vx_Zebra_dis_1 = Vx_enco * sin(Angle_Zebra_dis_bias);
      Vx_Zebra_dis_2 = Vx_enco * cos(Angle_Zebra_dis_bias);
      Vy_Zebra_dis_1 = Vy_enco * cos(Angle_Zebra_dis_bias);
      Vy_Zebra_dis_2 = Vy_enco * sin(Angle_Zebra_dis_bias); // 用于坡道调整
      Vx_Zebra_dis = Vx_Zebra_dis_2 - Vy_Zebra_dis_2;          // 简单的分解计算
      Vy_Zebra_dis = Vx_Zebra_dis_1 + Vy_Zebra_dis_1;
    }
    if (Angle_Zebra_dis_bias < 0)
    {
      Angle_Zebra_dis_bias = -Angle_Zebra_dis_bias;
      Vx_Zebra_dis_1 = Vx_enco * sin(Angle_Zebra_dis_bias);
      Vx_Zebra_dis_2 = Vx_enco * cos(Angle_Zebra_dis_bias);
      Vy_Zebra_dis_1 = Vy_enco * cos(Angle_Zebra_dis_bias);
      Vy_Zebra_dis_2 = Vy_enco * sin(Angle_Zebra_dis_bias); // 用于总钻风微调
      Vx_Zebra_dis = Vx_Zebra_dis_2 + Vy_Zebra_dis_2;          // 简单的分解计算
      Vy_Zebra_dis = -Vx_Zebra_dis_1 + Vy_Zebra_dis_1;
    }
    Zebra_dis_x += Vx_Zebra_dis * 0.005;
    Zebra_dis_y += Vy_Zebra_dis * 0.005; // 分解出卡片所需的里程，用于找卡片
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
  pid->error = target_diantance - actual_distance;                              // Calculate the deviation //
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
void CSI_dis_new_correct(float cor_x, float cor_y)
{
	if(cor_x!=0 && cor_y!=0)
	{
  delta_x = (cor_x)/10-correct_x; //单位为cm
  delta_y = cor_y/10-correct_y; //计算出中心坐标,y可能需要调整，参数暂定
	}
  //调整x方向
switch (correct_step)
 {

  case 1:                                                               // 调整垂直方向
    if (fabsf(delta_y) > 0 && correct_y_flag == 0 && correct_step == 1) // y距离过大，需要矫正，默认为第一步
    {
      if (delta_y > 20)
      {
        Vx = 0; // 水平不动
        Vy = 5; // 向前移动
      }
      else if (delta_y < 16)
      {
        Vx = 0;  // 水平不动
        Vy = -5; // 向后移动
      }
      else if (delta_y <= 20 && delta_y >= 16) // 已调整完毕 原本是21，现在调环岛修改为30
      {
        Vx = 0;
        Vy = 0;             // 速度清零
        correct_y_flag = 1; // y方向调整完毕
        correct_step = 2;   // 调整步数置2
      }
    }
    break;

  case 2:                                                                //
    if (correct_y_flag == 1 && correct_x_flag == 0 && correct_step == 2) // x距离过大，需要矫正，且步数为第二步
    {
      if (fabsf(delta_x) > 2 && correct_x_flag == 0) // x距离过大，需要矫正
      {
        Vy = 0;
        if (delta_x > 0)
          Vx = 5;
        else
          Vx = -5;
        correct_x_flag = 0;
      }
      else if (delta_x <= 2 && delta_x >= -2) // 已调整完毕
      {
        Vx = 0;
        Vy = 0;             // 速度清零
        correct_x_flag = 1; // x方向调整完毕
      }
    }
  	if(correct_y_flag==1 && correct_x_flag==1)//x,y均调整完成
		{
			correct_step=3;                   //步数回归到第一步
		}
   break;
   case 3:
    CSI_correct_flag=FINISH;//完成art4微调
    correct_x_flag=0;       //清零调整完成的标志位
    correct_y_flag=0;
    correct_step=1;         //回归到第一步
   break;
}
}
/**************************************************************************
函数功能：总钻风距离校正(未调参) 开环
入口参数：cor_x，cor_y（要校正的x和y），art识别出来的坐标一般有偏差，所以要再次识别中心点的x,y坐标输入矫正函数，新版加上距离闭环
返回值：
**************************************************************************/
int correct_island_center_step = 1;
float delta_island_center_x,delta_island_center_y;
int correct_island_center_x_flag,correct_island_center_y_flag; //x,y调整完毕标志位
uint8 CSI_island_center_correct_flag = NOT_FINISH;

void CSI_dis_island_correct(float cor_x, float cor_y)
{
	if(cor_x!=0 && cor_y!=0)
	{
  delta_island_center_x = (cor_x)/10-correct_x; //单位为cm
  delta_island_center_y = cor_y/10-correct_y; //计算出中心坐标,y可能需要调整，参数暂定
	}
  //调整x方向
switch (correct_island_center_step)
 {

 case 1:                                                       //调整垂直方向
    if(fabsf(delta_island_center_y)>0 && correct_island_center_y_flag==0 && correct_island_center_step==1)//y距离过大，需要矫正，默认为第一步
   {
		 if(delta_island_center_y>18)
		 {
			 Vx=0;//水平不动
			 Vy=5;//向前移动
		 }
		 else if(delta_island_center_y<14)
		 {
			 Vx=0;//水平不动
			 Vy=-5;//向后移动
		 }
     else if(delta_island_center_y<=18 && delta_island_center_y>=14)//已调整完毕 原本是21，现在调环岛修改为30
   {
     Vx=0;
     Vy=0;//速度清零
     correct_island_center_y_flag=1;//y方向调整完毕
		 correct_island_center_step=2;//调整步数置2
   }
   }			 
   break;

 case 2: //
	 if(correct_island_center_y_flag==1 && correct_island_center_x_flag==0 && correct_island_center_step==2)//x距离过大，需要矫正，且步数为第二步
   {
      if(fabsf(delta_island_center_x)>2&& correct_island_center_x_flag==0)//x距离过大，需要矫正
    {
      Vy=0;
			if(delta_island_center_x>0)
        Vx=5;
			else
				Vx=-5;
		  correct_island_center_x_flag=0;
    }
      else if(delta_island_center_x<=2 && delta_island_center_x>=-2)//已调整完毕
    {
      Vx=0;
      Vy=0;                             //速度清零
      correct_island_center_x_flag=1;                 //x方向调整完毕
    }
    }
  	if(correct_island_center_y_flag==1 && correct_island_center_x_flag==1)//x,y均调整完成
		{
			correct_island_center_step=3;                   //步数回归到第一步
		}
   break;
   case 3:
    CSI_island_center_correct_flag=FINISH;//完成art4微调
    correct_island_center_x_flag=0;       //清零调整完成的标志位
    correct_island_center_y_flag=0;
    correct_island_center_step=1;         //回归到第一步
   break;
}
}
/**************************************************************************
函数功能：art1环岛卡片定位(未调参) 开环
入口参数：island_center_card_x，island_center_card_y（要校正的x和y），art识别出来的坐标一般有偏差，所以要再次识别中心点的x,y坐标输入矫正函数
返回值：
**************************************************************************/
uint8 CSI_island_correct_flag = NOT_FINISH;
float delta_island_x, delta_island_y;
int correct_island_card_step = 1;
int correct_island_x_flag, correct_island_y_flag;
void CSI_correct_island_correct(float Island_center_card_x, float Island_center_card_y)
{
  if (Island_center_card_x != 0 && Island_center_card_y != 0) // 有坐标传入，再进行操作
  {
    delta_island_x = (Island_center_card_x + 20) / 10 - Island_x; // 单位为cm
    delta_island_y = Island_center_card_y / 10 - Island_y;        // 计算出环岛卡片定位需要调整的距离,y可能需要调整，参数暂定
  }
  // 调整y方向
  switch (correct_island_card_step)
  {

  case 1:                                                                                       // 调整垂直方向
    if (abs(delta_island_y) > 0 && correct_island_y_flag == 0 && correct_island_card_step == 1) // y距离过大，需要矫正，默认为第一步
    {
      if (delta_island_y > 38)
      {
        Vx = 0; // 水平不动
        Vy = 5; // 向前移动
      }
      else if (delta_island_y < 30)
      {
        Vx = 0;  // 水平不动
        Vy = -5; // 向后移动
      }
      else if (delta_island_y <= 38 && delta_island_y >= 30) // 已调整完毕 原本是21，现在调环岛修改为30
      {
        Vx = 0;
        Vy = 0;                       // 速度清零
        correct_island_y_flag = 1;    // y方向调整完毕
        correct_island_card_step = 2; // 调整步数置2
      }
    }
    break;

  case 2:                                                                                          //
    if (correct_island_y_flag == 1 && correct_island_x_flag == 0 && correct_island_card_step == 2) // x距离过大，需要矫正，且步数为第二步
    {
      if (delta_island_x < 5) // 车在卡片偏左，需要向右矫正
      {
        Vx = -5;
        Vy = 0;
      }
      else if (delta_island_x > 8) // 车在卡片偏右，需要向左矫正
      {
        Vx = 5;
        Vy = 0;
      }
      else if (delta_island_x <= 8 && delta_island_x >= 5) // 已调整完毕
      {
        Vx = 0;
        Vy = 0;                    // 速度清零
        correct_island_x_flag = 1; // x方向调整完毕
        correct_island_card_step = 3;
      }
    }
    break;
  case 3:
    if (correct_island_y_flag == 1 && correct_island_x_flag == 1 && correct_island_card_step == 3)
    {
      CSI_island_correct_flag = FINISH; // art1环岛卡片对正调整完成
      correct_island_x_flag = 0;
      correct_island_y_flag = 0;    // 调整标志位清0
      correct_island_card_step = 1; // 步数回归到第一步
    }
    break;
  }
}
/**************************************************************************
函数功能：art4距离校正(未调参) 开环
入口参数：cor_x，cor_y（要校正的x和y），art识别出来的坐标一般有偏差，所以要再次识别中心点的x,y坐标输入矫正函数，新版加上距离闭环
返回值：
**************************************************************************/
//float delta_island_center_x,delta_island_center_y;//算出的调整距离
//float  correct_island_center_x_flag,correct_island_center_y_flag;//x,y调整完成的标志位
//int correct_island_step = 1;//调整步数
//uint8 CSI_island_correct_finish_flag=NOT_FINISH;//环岛art4调整完成标志位,初始化为未完成
//void CSI_dis_island_correct(float cor_x, float cor_y)
//{
//	if(cor_x!=0 && cor_y!=0)
//	{
//  delta_island_center_x = (cor_x)/10-correct_x; //单位为cm
//  delta_island_center_y = cor_y/10-correct_y; //计算出中心坐标,y可能需要调整，参数暂定
//	}
//  //调整y方向
//switch (correct_island_step)
// {

// case 1:                                                       //调整垂直方向
//    if(fabsf(delta_island_center_y)>0 && correct_island_center_y_flag==0 && correct_island_step==1)//y距离过大，需要矫正，默认为第一步
//   {
//		 if(delta_island_center_y>18)
//		 {
//			 Vx=0;//水平不动
//			 Vy=5;//向前移动
//		 }
//		 else if(delta_island_center_y<14)
//		 {
//			 Vx=0;//水平不动
//			 Vy=-5;//向后移动
//		 }
//     else if(delta_island_center_y<=18 && delta_island_center_y>=14)//已调整完毕 原本是21，现在调环岛修改为30
//   {
//     Vx=0;
//     Vy=0;//速度清零
//     correct_island_center_y_flag=1;//y方向调整完毕
//		 correct_island_step=2;//调整步数置2
//   }
//   }			 
//   break;

// case 2: //
//	 if(correct_island_center_y_flag==1 && correct_island_center_x_flag==0 && correct_island_step==2)//x距离过大，需要矫正，且步数为第二步
//   {
//      if(fabsf(delta_island_center_x)>2&& correct_island_center_x_flag==0)//x距离过大，x要矫正
//    {
//      Vy=0;
//			if(delta_island_center_x>0)
//        Vx=5;
//			else
//				Vx=-5;
//		  correct_island_center_x_flag=0;
//    }
//      else if(delta_island_center_x<=2 && delta_island_center_x>=-2)//已调整完毕
//    {
//      Vx=0;
//      Vy=0;                             //速度清零
//      correct_island_center_x_flag=1;                 //x方向调整完毕
//    }
//    }
//  	if(correct_island_center_y_flag==1 && correct_island_center_x_flag==1)//x,y均调整完成
//		{
//			correct_island_step=3;                   //步数回归到第一步
//		}
//   break;
//   case 3:
//    CSI_island_correct_finish_flag=FINISH;//完成art4微调
//    correct_island_center_x_flag=0;       //清零调整完成的标志位
//    correct_island_center_y_flag=0;
//    correct_island_step=1;         //回归到第一步
//    break;
//}
//}
/**************************************************************************
函数功能：art1十字卡片定位   开环
入口参数：crossing_center_card_x，crossing_center_card_y（要校正的x和y），art识别出来的坐标一般有偏差，所以要再次识别中心点的x,y坐标输入矫正函数
返回值：
**************************************************************************/
int correct_crossing_card_step;
int correct_crossing_x_flag, correct_crossing_y_flag;
void CSI_correct_crossing_correct(float crossing_center_card_x, float crossing_center_card_y)
{
  if (crossing_center_card_x != 0 && crossing_center_card_y != 0) // 有坐标传入，再进行操作
  {
    delta_crossing_x = (crossing_center_card_x + 20) / 10 - Crossing_x; // 单位为cm
    delta_crossing_y = crossing_center_card_y / 10 - Crossing_y;        // 计算出环岛卡片定位需要调整的距离,y可能需要调整，参数暂定
  }
  // 调整y方向
  switch (correct_crossing_card_step)
  {

  case 1:                                                                                             // 调整垂直方向
    if (abs(delta_crossing_y) > 0 && correct_crossing_y_flag == 0 && correct_crossing_card_step == 1) // y距离过大，需要矫正，默认为第一步
    {
      if (delta_crossing_y > 38)
      {
        Vx = 0; // 水平不动
        Vy = 5; // 向前移动
      }
      else if (delta_crossing_y < 30)
      {
        Vx = 0;  // 水平不动
        Vy = -5; // 向后移动
      }
      else if (delta_crossing_y <= 38 && delta_crossing_y >= 30) // 已调整完毕 原本是21，现在调环岛修改为30
      {
        Vx = 0;
        Vy = 0;                         // 速度清零
        correct_crossing_y_flag = 1;    // y方向调整完毕
        correct_crossing_card_step = 2; // 调整步数置2
      }
    }
    break;

  case 2:                                                                                                // 调整水平方向
    if (correct_crossing_y_flag == 1 && correct_crossing_x_flag == 0 && correct_crossing_card_step == 2) // x距离过大，需要矫正，且步数为第二步
    {
      if (delta_crossing_x < 5) // 车在卡片偏左，需要向右矫正
      {
        Vx = -5;
        Vy = 0;
      }
      else if (delta_crossing_x > 8) // 车在卡片偏右，需要向左矫正
      {
        Vx = 5;
        Vy = 0;
      }
      else if (delta_crossing_x <= 8 && delta_crossing_x >= 5) // 已调整完毕
      {
        Vx = 0;
        Vy = 0;                      // 速度清零
        correct_crossing_x_flag = 1; // x方向调整完毕
        correct_crossing_card_step = 3;
      }
    }
    break;
  case 3:
    if (correct_crossing_y_flag == 1 && correct_crossing_x_flag == 1 && correct_crossing_card_step == 3)
    {
      CSI_crossing_correct_flag = FINISH; // art1环岛卡片对正调整完成
      correct_crossing_x_flag = 0;
      correct_crossing_y_flag = 0;    // 调整标志位清0
      correct_crossing_card_step = 1; // 步数回归到第一步
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
  switch (ramp_step)
  {
  case 1:                   // 向右横移出赛道
    Turn_Angle_PD(Angle_Z); // 锁住现在车头的位置，提供速度Vz
    if (ramp_x <= Traverse_distance && ramp_step == 1)
    {
      Vx = 30; // 左移出赛道
      Vy = 0;
    }
    else
    {
      Vx = 0; // 停车
      Vy = 0;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    if (abs(Traverse_distance - (int)ramp_x) < 2)
    {
      ramp_step = 2;
    }
    break;
  case 2:
    Turn_Angle_PD(Angle_Z); // 锁住现在车头的位置，提供速度Vz
    if (ramp_y <= Straight_distance && ramp_step == 2)
    {
      Vx = 0;
      Vy = 30; // 前进
    }
    else
    {
      Vx = 0; // 穿过坡道
      Vy = 0;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    if (abs(Straight_distance - (int)ramp_y) < 2)
    {
      ramp_step = 3;
    }
    break;
  case 3:
    if (abs((int)ramp_x) < 2)
    {
      ramp_step = 4;
    }
    Turn_Angle_PD(Angle_Z); // 锁住现在车头的位置，提供速度Vz
    if (ramp_x >= 0 && ramp_step == 3)
    {
      Vx = -30; // 右移回归赛道
      Vy = 0;
    }
    else
    {
      Vx = 0; // 停车
      Vy = 0;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    break;
  case 4:
    car_stop(); // 清空速度
    system_delay_ms(200);
    ramp_finish = 1;
    ramp_step = 0; // 回归到初始状态,需要使用时再赋值
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
  delta_class_x = (Traverse_class_distance) / 10 + 6 - (int)Card_dis_car_x; // 单位为cm
  delta_class_y = Straight_class_distance / 10 - 25 - (int)Card_dis_car_y;  // 计算出中心坐标,y可能需要调整，参数暂定
  switch (class_step)
  {
  case 1:                                     // 前进道合适区域
    if (delta_class_y > 3 && class_step == 1) // y距离过大，要前进
    {
      Vx = 0; // 前进
      Vy = 5;
    }
    else if (delta_class_y < 0 && class_step == 1) // y距离过小，要后退
    {
      Vx = 0; // 停车
      Vy = -5;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    if (delta_class_y <= 3 && delta_class_y >= 0)
    {
      Vx = 0; // 停车
      Vy = 0;
      class_step = 2; // 切换至调整x距离
    }
    break;
  case 2:
    if (delta_class_x > 3 && class_step == 2) // 距离过大，右移
    {
      Vx = 5; // 右移
      Vy = 0; // 前进
    }
    else if (delta_class_x < 0 && class_step == 2) // 超过，左移
    {
      Vx = -5; // 左移
      Vy = 0;
    }
    else if (delta_class_x <= 3 && delta_class_x >= 0 && class_step == 2)
    {
      Vx = 0; //
      Vy = 0; // 停车
      class_step = 3;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    break;
  case 3:
    car_stop(); // 清空速度
    system_delay_ms(200);
    classify_correct_finish = 1;
    class_step = 0; // 回归到初始状态,需要使用时再赋值
    break;
  }
}
/**
 * @brief 十字卡片分类区域对正函数
 * @param Traverse_distance横移距离
 * @param Straight_distance直行距离
 * @return 无
 */
int crossing_class_step = 1; // 校准步数初始化为1
float delta_crossing_class_x;
float delta_crossing_class_y;                        // 十字区域调整的delta
uint8 crossing_classify_correct_finish = NOT_FINISH; // 十字分类区域调整完成标志位
void find_crossing_classify(float Traverse_island_distance, float Straight_island_distance)
{
  delta_crossing_class_x = Traverse_island_distance / 10 - Crossing_x; // 单位为cm
  delta_crossing_class_y = Straight_island_distance / 10 - Crossing_y; // 计算出中心坐标,y可能需要调整，参数暂定
  switch (crossing_class_step)
  {
  case 1:                                     // 前进道合适区域
    Turn_Angle_PD(Angle_Z);                   // 锁住现在车头的位置，提供速度Vz
    if (delta_crossing_class_y > 40 && crossing_class_step == 1) // y距离过大，要前进
    {
      Vx = 0; // 前进
      Vy = 5;
    }
    else if (delta_crossing_class_y < 35 && crossing_class_step == 1) // y距离过小，要后退
    {
      Vx = 0; // 停车
      Vy = -5;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    if (delta_crossing_class_y <= 40 && delta_crossing_class_y >= 35)
    {
      Vx = 0; // 停车
      Vy = 0;
      crossing_class_step = 2; // 切换至调整x距离
    }
    break;
  case 2:
    Turn_Angle_PD(Angle_Z);                                       // 锁住现在车头的位置，提供速度Vz
    if (delta_crossing_class_x > -3 && crossing_class_step == 2) // 距离过大，右移
    {
      Vx = 5; // 右移
      Vy = 0; // 前进
    }
    else if (delta_crossing_class_x < -6 && crossing_class_step == 2) // 超过，左移
    {
      Vx = -5; // 左移
      Vy = 0;
    }
    else if (delta_crossing_class_x <= -3 && delta_crossing_class_x >= -6 && crossing_class_step == 2)
    {
      Vx = 0; //
      Vy = 0; // 停车
      crossing_class_step = 3;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    break;
  case 3:
    car_stop(); // 清空速度
    crossing_classify_correct_finish = FINISH;
    crossing_class_step = 0; // 回归到初始状态,需要使用时再赋值
    break;
  }
}
/**
 * @brief 环岛卡片分类函数
 * @param Traverse_distance横移距离
 * @param Straight_distance直行距离
 * @return 无
 */
float delta_island_class_x, delta_island_class_y;
int island_class_step = 1; // 初始化为1
void find_island_classify(float Traverse_island_distance, float Straight_island_distance)
{
  delta_island_class_x = Traverse_island_distance / 10 - Island_x; // 单位为cm
  delta_island_class_y = Straight_island_distance / 10 - Island_y; // 计算出中心坐标,y可能需要调整，参数暂定
  switch (island_class_step)
  {
  case 1:                                                    // 前进道合适区域
    Turn_Angle_PD(Angle_Z);                                  // 锁住现在车头的位置，提供速度Vz
    if (delta_island_class_y > 38 && island_class_step == 1) // y距离过大，要前进
    {
      Vx = 0; // 前进
      Vy = 5;
    }
    else if (delta_island_class_y < 35 && island_class_step == 1) // y距离过小，要后退
    {
      Vx = 0; // 停车
      Vy = -5;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    if (delta_island_class_y <= 38 && delta_island_class_y >= 35)
    {
      Vx = 0; // 停车
      Vy = 0;
      island_class_step = 2; // 切换至调整x距离
    }
    break;
  case 2:
    Turn_Angle_PD(Angle_Z);                                  // 锁住现在车头的位置，提供速度Vz
    if (delta_island_class_x > 0 && island_class_step == 2) // 距离过大，右移
    {
      Vx = 5; // 右移
      Vy = 0; // 前进
    }
    else if (delta_island_class_x < -4 && island_class_step == 2) // 超过，左移
    {
      Vx = -5; // 左移
      Vy = 0;
    }
    else if (delta_island_class_x <= 0 && delta_island_class_x >= -4 && island_class_step == 2)
    {
      Vx = 0; //
      Vy = 0; // 停车
      island_class_step = 3;
    }
    Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
    break;
  case 3:
    car_stop(); // 清空速度
    classify_correct_finish = FINISH;
    island_class_step = 0; // 回归到初始状态,需要使用时再赋值
    break;
  }
}
/**
 * @brief 环岛字母类型卡片转换
 * @param 输入：识别的字母类型
 * @param 输出：转化后的对应的卡片类型
 * @return 无
 */
void switch_abc_to_righttype(int Card_abc)
{
  switch (Card_abc)
  {
  case 1:
    record_abc_card_type = A_card;
    break;
  case 2:
    record_abc_card_type = B_card;
    break;
  case 3:
    record_abc_card_type = C_card;
    break;
  case 4:
    record_abc_card_type = D_card;
    break;
  case 5:
    record_abc_card_type = E_card;
    break;
  case 6:
    record_abc_card_type = F_card;
    break;
  case 7:
    record_abc_card_type = G_card;
    break;
  case 8:
    record_abc_card_type = H_card;
    break;
  case 9:
    record_abc_card_type = I_card;
    break;
  case 10:
    record_abc_card_type = J_card;
    break;
  case 11:
    record_abc_card_type = K_card;
    break;
  case 12:
    record_abc_card_type = L_card;
    break;
  case 13:
    record_abc_card_type = M_card;
    break;
  case 14:
    record_abc_card_type = N_card;
    break;
  case 15:
    record_abc_card_type = O_card;
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
    if (now_distance_y > 0 && now_distance_y < 1000 && abs(now_distance_x) < 400 && (normal_stop_flag == 0 || stop_detect_flag == 0)) // art识别到卡片，设定识别区间，不能离赛道太远避免识别杂物，出环岛停止检测解算坐标
    {
        if (only_one) // 只执行一次
        {
          Card_dis_car_x = 0;
          Card_dis_car_y = 0; // 底座坐标清零
          Angle_z = 0;        // 角度清0
          delta_card_x = 0;
          delta_card_y = 0; // 解算卡片位置的x,y
          delta_angle = 0;
          card_y[0] = now_distance_y / 10; // 记录下第一次传进来的数据
          card_x[0] = now_distance_x / 10; // 存放卡片y轴坐标
          catch_card_flag = OPEN;          // 捕获成功，记得要重新关闭,打开里程计的第二种模式
          only_one = 0;                    // 测试使用
          *mode = Car_find_card_y;         // 转变小车运动模式
          target_type = *mode;             // 测试变量使用
          return;
        }
//      }
        // return;
      // }
      return;
    }
    else
    {
      car_run_mode = 0;
      car_run(); // 正常巡线模式
      *mode = Car_go;
      return;
    }
  }
  //******************************找卡片*****************************//
  if (*mode == Car_find_card_y)
  {
    if (find_car_flag == 1) // 到达卡片附近
    {
      if (only_one)
      {
        now_angle = Angle_Z;       // 记录下转向前的角度
        find_car_flag = NOT_READY; // 清零到达标志位
        if (delta_card_x <= 0)     // 卡片相对于小车在左边时
        {
          turn_angle = 90 + now_angle; // 向左转90度
        }
        else
        {
          turn_angle = -90 + now_angle; // 向右转向90度
        }
        if (turn_angle >= 360)
          turn_angle = turn_angle - 360;
        if (turn_angle <= -360)
          turn_angle = turn_angle + 360;
        only_one = 0;     // 只执行一次
        *mode = Car_turn; // 模式转变
        target_type = *mode;
      }
      return;
    }
    else
    {
      only_one = 1;                                      // 重新打开only_one
      car_run();                                         // 正常循迹跑
      delta_card_y = card_y[0] - (double)Card_dis_car_y; // 算出在更新后的坐标轴下的y差值
      delta_card_x = card_x[0] - (double)Card_dis_car_x; // 算出在更新后的坐标轴下的x差值
      if (delta_card_x == 0)
        delta_angle = 90.0; // 当delta_x刚好为0值时(此时tan值无意义)，把这时的角度就认为为90
      else
        delta_angle = atan((double)(delta_card_y / delta_card_x)) / PI * 180 * 1.0; // 算出即时偏移角
      // 因为车身姿态与采样频率的问题，有且只有一个相交点，给出在符合角度的波动区间
      if (delta_angle - Angle_z < 18 && delta_angle - Angle_z > -10) // 当底盘坐标需要偏角较大的时候,一般在弯道
      {
        car_stop(); // 停车
        system_delay_ms(500);
        find_car_flag = 1;
      }
      *mode = Car_find_card_y;
      return;
    }
    return;
  }
  //******************************向卡片方向转向*****************************//
  if (*mode == Car_turn)
  {
    if (fabsf(Angle_Z - turn_angle) <= 4) // 陀螺仪转向识别
    {
      
      Vz = 0;                    // 清0Vz
      *mode = Car_find_card_cor; // 模式转变
      only_one = 1;
      car_stop();     // 清空速度
      turn_angle = 0; // 转向角度清零
      correct_x = 0;
      correct_y = 0; // 清零修正的x，y距离
      delta_x = 0;
      delta_y = 0;
      Angle_arrive_card = 0; // 清零角度
      // pick_up_mode=OPEN;//防止卡死
      correct_art2_flag = OPEN;          // 打开art2识别中断
      NVIC_SetPriority(LPUART1_IRQn, 2); // 降低UART1中断优先级
      arrive_card_flag = OPEN;           // 打开总钻风微调时的里程计计数
      CSI_correct_flag = NOT_FINISH;     // 总钻风调整完毕标志清除
      *mode = Car_find_card_cor;         // 模式转变
      target_type = *mode;
      return;
    }
    else
    {
      Island_allow_flag = NOT_READY;               //关闭环岛
      Cross_allow_flag =  NOT_READY;                //关闭十字
      banmaxian_allow_flag = NOT_READY;            //关闭斑马线
      // banmaxian_allow_flag = NOT_READY; // 此时关闭斑马线，因为转向外侧会判断成斑马线(不知道行不行)
      Turn_Angle_PD(turn_angle);        // 准备Vz转速
      Vx = 0;
      Vy = 0;                                      // x,y静止
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      *mode = Car_turn;
      return;
    }
    return;
  }
  //******************************总钻风对正*****************************//
  if (*mode == Car_find_card_cor) // 总钻风微调识别
  {
    if (CSI_correct_flag == FINISH) // 总钻风坐标对正
    {
      if (card_classify == 1 || card_classify == 2 || card_classify == 7 || card_classify == 13) // 交通工具类
      {
        Traffic_count++;
      }
      else if (card_classify == 4 || card_classify == 5 || card_classify == 6 || card_classify == 8 || card_classify == 14) // 武器类
      {
        Weapon_count++;
      }
      else if (card_classify == 3 || card_classify == 9 || card_classify == 10 || card_classify == 11 || card_classify == 12 || card_classify == 15) // 物资类
      {
        Supply_count++;
      }
      system_delay_ms(500);
      car_stop();               // 清空速度
      arrive_card_flag = CLOSE; // 关闭总钻风微调时的里程计计数
      correct_x = 0;
      correct_y = 0; // 清零修正的x，y距离
      delta_x = 0;
      delta_y = 0;
      Angle_arrive_card = 0;         // 清零角度
      only_one = OPEN;               // 重新打开only_one
      CSI_correct_flag = NOT_FINISH; // 总钻风微调标志清零
      near_card_x = 0;
      near_card_y = 0;
      card_center_x = 0;
      card_center_y = 0; // 清空记录的卡片中心坐标
      card_classify = 0;
      card_type = 0;
      *mode = Pick_up_card; // 模式转变
      target_type = *mode;
      return;
    }
    else
    {
      *mode = Car_find_card_cor;
      if (near_card_x != 0 && near_card_y != 0 && near_card_x!=666 && near_card_y!=666)
      {
        //          if(near_card_y>150)//有时候会发错坐标，要设定区间来截取正确的坐标
        //          {
        if (only_one)
        {
          card_center_x = near_card_x;
          card_center_y = near_card_y;
          card_classify = card_type;
          correct_art2_flag = CLOSE; // 立即关闭art4发数据，防止堵塞数据缓冲区
          only_one = 0;              // 只记录一次
        }
        //          }
      }
      CSI_dis_new_correct(card_center_x, card_center_y); // 总钻风坐标对正，准备x,y速度
      // Turn_Angle_PD(turn_angle);//准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);                                               // 麦轮控制，为target_speed赋值
      if (card_classify == 1 || card_classify == 2 || card_classify == 7 || card_classify == 13) // 交通工具类
      {
        classify_360(Traffic);
      }
      else if (card_classify == 4 || card_classify == 5 || card_classify == 6 || card_classify == 8 || card_classify == 14) // 武器类
      {
        classify_360(Weapon);
      }
      else if (card_classify == 3 || card_classify == 9 || card_classify == 10 || card_classify == 11 || card_classify == 12 || card_classify == 15) // 物资类
      {
        classify_360(Supply);
      }
      return;
    }
    return;
  }
  //******************************卡片拾取*****************************//
  if (*mode == Pick_up_card) // 捡卡片
  {
    if (arm_pick_flag == ARM_PICK_DONE) // 卡片已被拾取
    {
      correct_x = 0;
      correct_y = 0;           // 清零修正的x，y距离,为下一步倒车做准备
      Angle_arrive_card = 0;   // 清零角度
      arrive_card_flag = OPEN; // 开启总钻风微调时的里程计计数
      *mode = Car_turn_again;  // 模式转变为转向回正
      return;
    }
    else // 卡片未被拾取
    {
      arm_control(2);                // 捡卡片
      arm_control(4);                // 默认模式
      arm_pick_flag = ARM_PICK_DONE; // 打开中断
      *mode = Pick_up_card;
      return;
    }
  }
  //******************************车头回正*****************************//
  if (*mode == Car_turn_again)
  {
    if (fabsf(Angle_Z - now_angle) <= 4) // 陀螺仪转向识别
    {
      // pick_up_mode=0;     //摄像头变为寻迹模式
      *mode = Car_go; // 重新变为寻迹
      Center_line_deal_plus(80, 100);
      car_stop(); // 清空速度
      /*****清空标志位****/
      banmaxian_allow_flag = READY;            //开启斑马线
      Cross_allow_flag = READY;                //开启十字
      Island_allow_flag = READY;               //开启环岛
      catch_card_flag = 0;      // 退出里程计第二种模式
      CSI_correct_flag = 0;     // 总钻风微调标志清零
      find_car_flag = 0;        // 找到卡片标志位清零
      arrive_card_flag = CLOSE; // 关闭里程计修正计数
      arm_pick_flag = ARM_PICK_NOT_DONE;
      pick_up_mode = CLOSE;
      //      ahead_flag = 0;
      /****清空各种坐标和角度****/
      Longest_Column_Fixed = 1;
      Center_line_deal_plus(23,163);
      now_distance_x = 0;
      now_distance_y = 0;
      record_now_distance_x = 0; // 清空now_distance_x
      record_now_distance_y = 0; // 清空now_distance_y,避免直接进入模式2
      Angle_arrive_card = 0;     // 清零角度
      correct_x = 0;
      correct_y = 0; // 清零修正的x，y距离
      Card_dis_car_x = 0;
      Card_dis_car_y = 0; // 卡片里程计清空
      Angle_z = 0;
      delta_angle = 0; // 算出的偏移角清0
      card_x[0] = 0;   // 卡片坐标清空
      card_y[0] = 0;
      only_one = 1;                      // 重新打开only_one
      NVIC_SetPriority(LPUART1_IRQn, 0); // 恢复art1的中断优先级
      Island_allow_flag = READY;               //开启环岛
      Cross_allow_flag =  READY;               //开启十字
      banmaxian_allow_flag = READY;            //开启斑马线
			system_delay_ms(1000);//停车0.5s
      // uart_write_string(UART_1, uart_1_begin);
      return;
    }
    else
    {
      Turn_Angle_PD(now_angle); // 向原先的角度转向回正
      Vx = 0;
      Vy = Distance_pid(&distance_pid[0], -13, (int)correct_y); // 后退
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);              // 麦轮控制，为target_speed赋值
      *mode = Car_turn_again;
      return;
    }
    return;
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
    if (card_position[now_card].card_position_ready == YES) // 当卡片数组更新，当前卡片坐标已存入后,可以进入距离判断，避免从原点就开始发癫
    {
      *mode = Car_find_card_y; // 转变小车运动模式
      target_type = *mode;     // 测试变量使用
    }
  }
  else
  {
    car_run(); // 正常巡线模式
    *mode = Car_go;
  }
  //******************************找卡片*****************************//
  if (*mode == Car_find_card_y)
  {
    if (car_card_diatance < 35 && car_card_diatance > 25) // 到达卡片附近
    {
      car_stop(); // 清空速度
      system_delay_ms(1000);
      now_angle = Angle_Z;                                              // 记录下此时角度
      card_car_x = card_position[now_card].x_distance - (int)Car_dis_x; // 世界坐标上卡片与车辆的x距离
      card_car_y = card_position[now_card].y_distance - (int)Car_dis_y; // 世界坐标上卡片与车辆的y距离
      card_car_other_angle = atan2(card_car_y, card_car_x) / PI * 180;  // 转换成角度制
      if (abs(card_car_other_angle - (int)Angle_Z) < 4)                 // 当解算出的角度与旋转角Angle_Z相差不大时
      {
        turn_angle = -90 + now_angle; // 向右转90度
      }
      if (abs(card_car_other_angle - (int)Angle_Z - 180) < 4) // 当解算出的角度与旋转角Angle_Z翻转180度后的角度相差不大时,向左转
      {
        turn_angle = 90 + now_angle; // 向左转90度
      }
      *mode = Car_turn; // 模式转变
    }
    else
    {
      car_run(); // 正常寻迹跑
      // 实时计算
      car_card_angle = card_world_angle - car_world_angle;                                                                               // 角度制
      car_card_diatance = (int)sqrt((card_position[now_card].world_distance * card_position[now_card].world_distance)                    // 卡片与原点距离的平方
                                    + (car_world_distance * car_world_distance)                                                          // 车辆与原点距离的平方
                                    - 2 * card_position[now_card].world_distance * car_world_distance * cos(car_card_angle / 180 * PI)); // 计算这时第now_card张卡片的角度
      *mode = Car_find_card_y;                                                                                                           // 保持该模式
    }
  }
  //******************************向卡片方向转向*****************************//
  if (*mode == Car_turn)
  {
    if (fabsf(Angle_Z - turn_angle) < 1) // 陀螺仪转向识别
    {
      // Vz = 0;//清0Vz
      car_stop(); // 清空速度
      system_delay_ms(1000);
      *mode = Car_find_card_cor; // 模式转变
      target_type = *mode;
      pick_up_mode = 1; // 打开总钻风识别
    }
    else
    {
      Turn_Angle_PD(turn_angle); // 准备Vz转速
      Vx = 0;
      Vy = 0;                                      // x,y静止
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      *mode = Car_turn;
    }
  }
  //******************************总钻风对正*****************************//
  if (*mode == Car_find_card_cor) // 总钻风微调识别
  {
    if (CSI_correct_flag == 1) // 总钻风坐标对正
    {
      car_stop(); // 清空速度
      system_delay_ms(500);
      *mode = Pick_up_card; // 模式转变
      target_type = *mode;
    }
    else
    {
      *mode = Car_find_card_cor;
      CSI_dis_new_correct(center_x, center_y); // 总钻风坐标对正，准备x,y速度
      Vz = 0;
      //      Turn_Angle_PD(turn_angle);//准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      //			system_delay_ms(200);
    }
  }
  //******************************卡片拾取*****************************//
  if (*mode == Pick_up_card) // 捡卡片
  {
    if (arm_pick_flag == ARM_PICK_DONE) // 卡片已被拾取
    {
      catch_card_flag = 0;    // 退出里程计第二种模式
      *mode = Car_turn_again; // 模式转变为转向回正
    }
    else // 卡片未被拾取
    {
      arm_control(2);                               // 捡卡片
      arm_control(4);                               // 默认模式
      card_position[now_card].pick_doen_flag = YES; // 标记该张卡片已经被拾取完毕
      arm_pick_flag = ARM_PICK_DONE;
      *mode = Pick_up_card;
    }
  }
  //******************************车头回正*****************************//
  if (*mode == Car_turn_again)
  {
    if (fabsf(Angle_Z - now_angle) <= 2) // 陀螺仪转向识别
    {
      *mode = Car_go;       // 重新变为寻迹
      car_stop();           // 清空速度
      system_delay_ms(500); // 停车0.5s
      now_card++;           // 开始对比下一张卡片坐标
      target_type = *mode;
    }
    else
    {
      Turn_Angle_PD(now_angle); // 向原先的角度转向回正
      Vy = Distance_pid(&distance_pid[0], -10, (int)correct_y);
      Vx = 0;
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
int zebra_card_x,zebra_card_y;//解算出的斑马线小世界坐标
int last_zebra_card_x,last_zebra_card_y;//解算出的斑马线小世界坐标
int delta_zebra_x,delta_zebra_y;//差值
uint8 Zebra_card_count=0;       //斑马线识别出的区域数目
uint8 turn_other_side = OPEN;   //转向另一边的标志位
float turn_once_side_angle = 0;//转向另一侧的角度
float turn_other_side_angle = 0;//转向另一侧的角度
float max_turn_distance = 110;//转向的极限距离
void card_final_classify(int *classify_step)
{
  //******************************向右转向***************************//
  if (*classify_step == Find_banmaxian && banmaxian_finish == NOT_FINISH) // 找到斑马线,且处理还未完成
  {
    if (abs(Angle_Z - turn_once_side_angle) <= 4 && turn_once_side_angle!=0) // 转到了目标角度
    {
      *classify_step = Find_upline; // 转变成上边线寻迹
      car_stop();                   // 停车，清零速度
      Zebra_classify_flag = OPEN;   // 打开斑马线的小世界坐标里程计
      Angle_Zebra = 0;              // 角度清零
      Zebra_x = 0;
      Zebra_y = 0;                  // 清零里程计，画为原点
      now_distance_x = 0;
      now_distance_y = 0;
      // classify_art2_flag=OPEN;   // 打开art4分类中断标志
      once_time = 1; // 重新打开one_time
      correct_x = 0;
      correct_y = 0;            // 清零修正的x，y距离
      Angle_arrive_card = 0;    // 清零角度
      arrive_card_flag = CLOSE; // 开启总钻风微调时的里程计计数
      return;
    }
    else
    {
      find_card_allow = NOT_READY;    //关闭此时找卡片的标志位
      if (once_time)
      {
        Now_angle = Angle_Z; // 记录下当前角度
        turn_once_side_angle = Now_angle - 90;
        if(turn_once_side_angle>=360) 
          turn_once_side_angle=turn_once_side_angle-360;
	      if(turn_once_side_angle<=-360) 
        turn_once_side_angle=turn_once_side_angle+360;//限幅
        once_time = 0;
        correct_x = 0;
        correct_y = 0;           // 清零修正的x，y距离
        Angle_arrive_card = 0;   // 清零角度
        arrive_card_flag = OPEN; // 开启总钻风微调时的里程计计数
      }
      Turn_Angle_PD(turn_once_side_angle);              // 此处可以根据实际情况修改
      Vx = 0;
      Vy = 0;
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      *classify_step = Find_banmaxian;
      return;
    }
    return;
  }
   //******************************沿上边线平移***************************//
  if (*classify_step == Find_upline) // 切换至上边线寻迹
  {
    if (Traffic_Finish == FINISH && Supply_Finish == FINISH && Weapon_Finish == FINISH) // 三类区域都已经识别完成
    {
      *classify_step = Turn_back; // 转回正常寻迹
      Card_dis_car_x = 0;
      Card_dis_car_y = 0; // 卡片里程计清空
      return;
    }
    else if(Zebra_card_count<3 && Zebra_dis_x > max_turn_distance && turn_other_side==OPEN)//此时数不够数目卡片，且已经后退得足够远
    {
      turn_other_side = CLOSE;//只会掉头一次
      Zebra_dis_x = 0;
      Zebra_dis_flag = CLOSE;     //开启斑马线倒退计算标志位
      Angle_Zebra_dis = 0;        //斑马线角度清零
      Zebra_dis_x = 0;
      Zebra_dis_y = 0;            //累计清零
      turn_other_side_angle = Now_angle + 90; //向另一侧转向的角度
      if(turn_other_side_angle>=360) 
        turn_other_side_angle=turn_other_side_angle-360;
	    if(turn_other_side_angle<=-360) 
        turn_other_side_angle=turn_other_side_angle+360;//限幅
      *classify_step = Zebra_Turn_Other_Side;
    }
    else if (now_distance_y > 250 && now_distance_y < 600 && now_distance_x < 200 && now_distance_x > -200) // art1识别到坐标,设置识别区间为右中平面，实在不行就直接上世界坐标解算来判断
    {
      zebra_card_x = (int)Zebra_x + now_distance_x / 10;
      zebra_card_y = (int)Zebra_y + now_distance_y / 10; // 解算出的新坐标,单位为cm
      delta_zebra_x = zebra_card_x - last_zebra_card_x;  // 算出差值
      delta_zebra_y = zebra_card_y - last_zebra_card_y;  // 算出差值
      last_zebra_card_x = zebra_card_x;
      last_zebra_card_y= zebra_card_y;                 //记录下上次的解算出的世界坐标
      if(abs(delta_zebra_x) >= 30 || Zebra_card_count == 0)                     //主要靠delta_zebra_x来判断，可能会加上y，要大于某个阈值
      {
      num_card_x = now_distance_x; // 记录看到的x坐标
      num_card_y = now_distance_y; // 记录看到的y坐标
      // now_distance_x = 0;
      // now_distance_y = 0;
      delta_class_x = 0;
      delta_class_y = 0;           //下一步校准的delta值
      catch_card_flag = OPEN;      // 打开卡片捕获的里程计
      Angle_z = 0;                 // 清零角度
      Card_dis_car_x = 0;
      Card_dis_car_y = 0; // 卡片里程计清空
      classify_correct_finish = 0;
      class_step = 1;              // 卡片分类区域修正步数初始化
      *classify_step = Catch_card; // 向卡片分类区域前进
      // NVIC_SetPriority(LPUART1_IRQn, 2);	//降低art1的中断优先级
      classify_type = *classify_step;
      return;
      }
      else
      {
        now_distance_x = 0; // 清零坐标，防止误判
        now_distance_y = 0;
        *classify_step = Find_upline;
        return;
      }
      return;
    }
    else
    {
      *classify_step = Find_upline;
      // 方案一，上边线巡线，鲁棒性好
      car_run_upline_right(110);                 // 上边线寻迹平移
//      Turn_Angle_PD(Angle_Z);              // 此处可以根据实际情况修改，提供Vz的车头修正速度
//      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      return;
    }
    return;
  }
  //**************************卡片坐标校准****************************** */
  if (*classify_step == Catch_card) // 寻找卡片分类区域
  {
    if (classify_correct_finish == 1) // 到达了数字分类卡片区域，准备识别
    {
      car_stop();                        //清空速度
      classify_art2_flag = OPEN;         // 打开art4中断标志
      NVIC_SetPriority(LPUART1_IRQn, 3); // 降低art1的中断优先级
      Find_num = NOT_READY;              // 清零Find_num
      // card_num = 0;                      // 清零之前记录的卡片类型
      // numcard_classify = 0;              // 清零记录的数字类型
      /***************清除这一步用的变量*************/
      classify_correct_finish = 0;       // 清除调整完毕的标志位
      num_card_x = 0;                    // 清零记录数字卡片的x坐标
      num_card_y = 0;                    // 清零记录数字卡片的y坐标
      now_distance_x = 0;                // x坐标清零
      now_distance_y = 0;                // y坐标清零
      once_time = 1;                     // 打开once_time
      *classify_step = Watch_card;       // 变更为识别卡片
      return;
    }
    else
    {
      Turn_Angle_PD(Angle_Z); // 此处可以根据实际情况修改，提供Vz的车头修正速度
      find_classify(num_card_x, num_card_y);
      *classify_step = Catch_card;
      return;
    }
  }
// *****************************识别卡片分类区域*****************************//  
  if (*classify_step == Watch_card)
  {
    if (Find_num == READY) // 找到了数字分类卡片区域，准备识别
    {
      Zebra_card_count++;
      if(Zebra_card_count == 1)        //当第一张被捕获成功时
      {
        Zebra_dis_flag = OPEN;         //开启斑马线倒退计算标志位
        Angle_Zebra_dis = 0;        //斑马线角度清零
        Zebra_dis_x = 0;
        Zebra_dis_y = 0;            //累计清零
      }
      if (numcard_classify == 1)    // 武器放置类
      {
        put_out_count = Weapon_count;
      }
      else if (numcard_classify == 2) // 物资放置类
      {
        put_out_count = Supply_count;
      }
      else if (numcard_classify == 3) // 交通工具类
      {
        put_out_count = Traffic_count;
      }
      car_stop();
      system_delay_ms(1000); // 等舵机转到对应角度
      card_num = 0;          // 清空已识别的数字
      numcard_classify = 0;  // 清空已记录的数字
      put_out_card_flag = NOT_FINISH;
      Find_num = NOT_READY; // 重置识别完毕的标志位
      once_time = 1;        // 重新打开once_time
      *classify_step = Putout_card; // 放卡片
      return;
    }
    else
    {
      classify_art2_flag = OPEN;  // 打开art4中断标志
      if (card_num != 0)          // 识别到卡片类型
    {
        if (once_time)
        {
          numcard_classify = card_num; // 数字类型
          Find_num = READY;            // 识别出了卡片类型
          classify_art2_flag = CLOSE;  // 立即关闭art4发数据，防止堵塞数据缓冲区
          once_time = 0;               // 只记录一次
        }
        if (numcard_classify == 1) // 武器放置类
      {
        classify_360(Weapon);
        Weapon_Finish = FINISH; // 完成武器类的舵机转向
				return;
      }
      else if (numcard_classify == 2) // 物资放置类
      {
        classify_360(Supply);
        Supply_Finish = FINISH; // 完成物资类的分类
				return;
      }
      else if (numcard_classify == 3) // 交通工具类
      {
        classify_360(Traffic);
        Traffic_Finish = FINISH; // 完成交通类的舵机转向
				return;
      }
    }
      *classify_step = Watch_card;
      return;
    }
    return;
  }
  if (*classify_step == Putout_card) // 拿出卡片
  {
    if (put_out_count == 0) // 所有卡片均放出
    {
      put_out_card_flag = NOT_FINISH;
      put_out_count = 0;        // 清零需要放出的卡片数目
      *classify_step = Go_back; // 后退
      return;
    }
    else
    {
      if (put_out_count > 0)
      {
        arm_control(3); // 放卡片
        arm_control(4); // 默认模式
        put_out_count--;
      }
      *classify_step = Putout_card; // 放卡片
      return;
    }
    return;
  }
  if (*classify_step == Go_back) // 后退至原本可用art1识别的位置
  {
    if (Card_dis_car_y <= 2 && Card_dis_car_y >= -1) // 设置容错区间
    {
      classify_type = *classify_step;
      // NVIC_SetPriority(LPUART1_IRQn, 0); // 恢复art1的中断优先级
      /******************清零各种变量***************/
      now_distance_x = 0;
      now_distance_y = 0; // 此处清零主要是为了防止直接状态二判断成功，乱识别东西
      Card_dis_car_x = 0;
      Card_dis_car_y = 0; // 卡片里程计清空
      /******************关闭各种标志位*************/
      catch_card_flag = CLOSE;
      if (Traffic_Finish == FINISH && Supply_Finish == FINISH && Weapon_Finish == FINISH)
			{
        *classify_step = Turn_back;
        return;
			}
      else 
			{
        *classify_step = Find_upline; // 继续向右平移找卡片
        return;
			}
    }
    else
    {
      *classify_step = Go_back;
      Vx = 0;
      Turn_Angle_PD(Angle_Z);                              // 此处可以根据实际情况修改，提供Vz的车头修正速度
      Vy = Distance_pid(&distance_pid[0], 0, (int)Card_dis_car_y); // 向后退
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);                 // 麦轮控制，为target_speed赋值
      return;
    }
    return;
  }
  if (*classify_step == Turn_back) // 转向回正
  {
    if (abs(Angle_Z - Now_angle) <= 4)
    {
      *classify_step = Find_banmaxian; // 返回至初始状态，直到下一次重新进去
      classify_type = *classify_step;
      car_run_mode = 3;
      Card_dis_car_x = 0;
      Card_dis_car_y = 0;
      banmaxian_finish = FINISH; // 斑马线处理完成
      return;
    }
    else
    {
      Vx = 0;
      Vy = 0;
      Turn_Angle_PD(Now_angle);                    // 转向回正
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      *classify_step = Turn_back;
      return;
    }
    return;
  }
  if (*classify_step == Zebra_Turn_Other_Side) // 转向另一侧
  {
    if (abs(Angle_Z - turn_other_side_angle) <= 4)//转至容错区间
    {
      *classify_step = Find_upline; // 返回至初始状态，直到下一次重新进去
      Card_dis_car_x = 0;
      Card_dis_car_y = 0;        //清零调整里程
      return;
    }
    else
    {
      Vx = 0;
      Vy = 0;
      Turn_Angle_PD(turn_other_side_angle);                    // 转向另一侧
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      *classify_step = Zebra_Turn_Other_Side;
      return;
    }
    return;
  }
}
// /**
// * @brief 左环岛多搬策略
// * @param Island_step为步数选择
// * @param 该函数是判断出为左环岛类型时才使用
// * @return 无
// */
extern int Island_State;

int Island_mode=0;
uint8 Island_allow_flag = READY;
uint8 Left_Island_Done = NOT_FINISH;
uint8 allow_flag=OPEN;
double Left_Island_classify_zone_x,Left_Island_classify_zone_y;
double delta_find_Island_zero_angle; 
float now_Island_angle;
float turn_IsLand_angel;
float turn_outside_island_angle;
float record_island_zone_x, record_island_zone_y;
int record_abc_flag = NOT_FINISH; // 记录完成的标志位
extern int transform_buffer[16];
uint8 back_flag = NOT_FINISH;
uint8 ahead_flag = NOT_FINISH;
float turn_inside_island_zone_angle; // 回到中心后的转向角
int Island_Zone_count = 0;           // 识别出的环岛区域数目
int record_inside_x, record_inside_y;
int last_record_inside_x, last_record_inside_y;
int delta_inside_x, delta_inside_y;
float test_top_error;
int island_target_upline=105;
int island_found = NOT_READY;//环岛防置卡片区域的匹配标志位
float turn_out_Island_angle; //出环角度
/***************记录环岛的区域解算坐标和它的类型**************/
int center_card_island_distance; // 捕获到卡片时，卡片中心与车的距离
float Card_island_angle;         // 捕获到卡片时的偏转角，转换成角度制
float delta_card_island_angle;   // 连线偏角转化成弧度制
uint8 arrive_island_center_flag = NOT_READY;
float island_card_center_x, island_card_center_y;     // art4记录的环岛卡片坐标，矫正使用
uint8 Find_Island_Card_Position = NOT_READY;          // 记录完成的标志位,初始化为未完成
float island_correct_again_x, island_correct_again_y; // 环岛卡片微调的输入坐标
int island_card_type = 0;                             // 记录识别的卡片类型
uint8 island_find_oldcard_flag = NO;                  // 记录找到的卡片是否是旧卡片
int Island_card_type;
int right_lie_island_upline_position;//环岛扫上边线最右列的行坐标
int mid_lie_island_upline_position;//环岛扫上边线中间列行坐标
uint8 shabi_saoxian_step=0;//傻逼扫线的步数
int second_right_lie_island_upline_position;//扫两次的，防止直接扫到内圆
float record_island_catch_angle,last_record_island_catch_angle;//环岛内角度记录
float delta_record_island_angle;                               //环岛内捕获卡片时的角度差值
uint8 Left_Island_Finish = NOT_FINISH;
float turn_out_crossing_angle;
//float turn_outside_island_angle=-90;					//测试用的，一定要记得删掉！！！！！！！！！！！！！！！！

void Left_Island_pick_and_move(int *Island_step)
{
  /*********************去原点处的判断******************/
  if (*Island_step == Arrive_zeropoint) // 去环岛的原点处
  {
    if (arrive_island_center_flag == READY) // 到达与最右侧的区域的环岛区域
    {
      car_stop();
      system_delay_ms(500);
      arrive_island_center_flag = NOT_READY;
      now_Island_angle = Angle_Z;      //记录下环岛转向前的角度
      Angle_Island_Zero = 0;           //专门用来圆环转向的角度
     turn_IsLand_angel = Angle_Z+90;  //向左转向90度
      if(turn_IsLand_angel>=360) turn_IsLand_angel=turn_IsLand_angel-360;
	    if(turn_IsLand_angel<=-360) turn_IsLand_angel=turn_IsLand_angel+360;
     *Island_step=Car_Island_turn;    //向圆环中心区域转向
     return;
   }
   else
   {
    /***********************/
    banmaxian_allow_flag = NOT_READY; //关闭斑马线标志
    Cross_allow_flag = NOT_READY;//关闭十字
    car_run();
    if(Island_State==3)
    {
      arrive_island_center_flag=READY;
    }
     *Island_step=Arrive_zeropoint;
     return;
   }
   return;
 }
 /*********************转向环岛中心区域******************/
  else if(*Island_step==Car_Island_turn)           //车头转向环岛外侧
 {
   if(abs(turn_IsLand_angel-Angle_Z)<=3)        //摄像头朝向环岛的内侧
   {
     car_stop();
     system_delay_ms(500);
    //  correct_art2_flag=OPEN;                        //打开art4识别类型
     Find_Island_Card_Position=NOT_READY;           //进入前先清零调整完毕的坐标
     /********打开用来环岛修正的里程计(必须)*******/
     Island_classify_flag=OPEN;                  //打开环岛里程计模式
     Angle_Island = 0;                           //第一次用来对环岛中心卡片做定位
     Island_x=0;
     Island_y=0;                                 
     delta_island_x=0;                           
     delta_island_y=0;                           //为下一步art1定位做准备
     CSI_island_correct_flag =  NOT_FINISH;      //总钻风调整完毕标志清除
     /*********到时候有个总钻风识别坐标，可以动态调整，目前先用art4调坐标*********/
     island_card_center_x=0;
     island_card_center_y=0;
    //  now_distance_x=0;
    //  now_distance_y=0;
     *Island_step=Island_Card_Correct;              //转变模式 
     return;   
   }
   else
   {
     Turn_Angle_PD(turn_IsLand_angel);
     Vx=0;
     Vy=0;
     Car_Inverse_kinematics_solution(Vx, Vy, Vz);              //麦轮控制，为target_speed赋值
     *Island_step=Car_Island_turn;
     return;
   }
   return;
 }
/*********************对环岛卡片对正******************/
else if(*Island_step==Island_Card_Correct)  
 {
   if(CSI_island_correct_flag == FINISH)//定位完成
   {
     NVIC_SetPriority(LPUART1_IRQn, 2);	//降低art1的中断优先级,为下一步art4再矫正做准备
     car_stop();
     system_delay_ms(500);
     /**************清零变量与下一步的变量准备*************/
     CSI_island_correct_flag = NOT_FINISH;
     island_card_center_x=0;        //用完就清
     island_card_center_y=0;
     Island_classify_flag=CLOSE;                 //关闭环岛里程计模式
     Angle_Island = 0;                           //第二次用来对环岛中心卡片做定位
     Island_x=0;
     Island_y=0;                                 
     delta_island_x=0;                           
     delta_island_y=0;
     /****************************************/
     now_distance_x=0;
     now_distance_y=0;
     near_card_x=0;                 //准备用才清
     near_card_y=0;
     island_correct_again_x=0;
     island_correct_again_y=0;
     /**************为art4对准做准备*************/
     arrive_card_flag = OPEN;       //开启修正用的里程计
     Angle_arrive_card = 0;
     correct_x=0;
     correct_y=0;                   //清零修正里程
     delta_x=0;
     delta_y=0;                     //art修正用的变量
     CSI_island_center_correct_flag = NOT_FINISH; //art4完成修正的标志位
     /**************清零/重启标志位***************/
     allow_flag=OPEN;                             //重新打开allow_flag
     correct_art2_flag=OPEN;                      //打开art4中断
     Find_Island_Card_Position=NOT_READY;         //清空记录标志位
    /****************************************/
    *Island_step=Island_Card_Correct_again;       //环岛的卡片再次对正
    return;
   }
   else
   {
    if(now_distance_x != 0 && now_distance_y> 100 && now_distance_y <700)      //目前先使用art1做矫正
    {
      if(allow_flag==OPEN)
      {
        if(now_distance_x > 0)
        {
           island_card_center_x=(float)now_distance_x;           
        }
        if(now_distance_x < 0)
        {
           island_card_center_x=-(float)now_distance_x;           
        }
			island_card_center_y=(float)now_distance_y;                                   //记录下环岛的卡片坐标，做矫正使用
      // correct_art2_flag=CLOSE;                                            //关闭atr4的中断
      allow_flag=CLOSE;
      }
    }
      CSI_correct_island_correct(island_card_center_x, island_card_center_y);//art1坐标对正，准备x,y速度,只做一个初步校正
      Turn_Angle_PD(turn_IsLand_angel);                                      //准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);                           //麦轮控制，为target_speed赋值
    *Island_step=Island_Card_Correct;//保持模式
    return;
   }
   return;
 }
 /**************对环岛中心卡片用art4再对准一遍中心****************/
else if(*Island_step==Island_Card_Correct_again)
{
  if(CSI_island_center_correct_flag == FINISH)                  //art4对准完成
  {
     allow_flag=OPEN;
     car_stop();                    //清零速度，防止乱动
     /************清零使用过的变量**********/
     arrive_card_flag = CLOSE;      //关闭修正用的里程计
     correct_x=0;
     correct_y=0;                   //清零修正里程
     delta_x=0;
     delta_y=0;                     //art修正用的变量
     CSI_island_center_correct_flag = NOT_FINISH; //清零矫正完成的标志位
     island_card_type=0;
     card_type=0;                   //这个变量虽然没有用到但其实已经赋值，为下一步art4做准备
     card_classify_count=0;         //为下一步分类做准备
     /*************************************/
     *Island_step=Island_Card_Classify_Pick;//转变模式
     return;
  }
  else
  {
    *Island_step=Island_Card_Correct_again;       //保持模式

    if(near_card_x != 0 && near_card_y >= 150 && near_card_x !=666 && near_card_y!=666)    //有正确的卡片坐标传入
    {
      if(allow_flag==OPEN)                        //只执行一次
      {
        island_correct_again_x=(float)near_card_x+10;
        island_correct_again_y=(float)near_card_y;       //再次微调的art4输入距离  原本测的时候，y偏移量有+12
        correct_art2_flag=CLOSE;                  //关闭art4修正中断
        allow_flag=CLOSE;
      }
    }
      CSI_dis_island_correct(island_correct_again_x, island_correct_again_y);//art4坐标对正，准备x,y速度
      Turn_Angle_PD(turn_IsLand_angel);           //准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);  //麦轮控制，为target_speed赋值
      return;
  }
  return;
}       
 /*********************对环岛中心卡片分类******************/
else if(*Island_step==Island_Card_Classify_Pick)//环岛卡片的分类
{
   if(island_card_type!=0)              //卡片类型已记录
   {
     if(card_classify_count<5)
     {
      Island_card[card_classify_count].Card_Type=island_card_type;    //记录下该张卡片类型
      Island_card[card_classify_count].Card_PWM_Duty=card_classify_count;      //记录该张卡片在舵机转动中的类别
      classify_little_360(card_classify_count);
      arm_control(2);//捡卡片 
	    arm_control(4);//默认模式                               //卡片拾取完成
      island_card_type=0;                                     //清零，等待下一次的识别传入
      card_classify_count++;                                  //卡片数量加一
      *Island_step=Island_Card_Classify_Pick;                 //继续进去环岛的卡片分类和拾取状态
      return;
     }
     else
     {
        correct_art2_flag=CLOSE;         //关闭atr4中断
        turn_outside_island_angle=now_Island_angle-90;       //为下一个角度转向做准备
       if(turn_outside_island_angle>=360) 
        turn_outside_island_angle=turn_outside_island_angle-360;
	     if(turn_outside_island_angle<=-360) 
        turn_outside_island_angle=turn_outside_island_angle+360;
        *Island_step=Car_Island_Turn_Outside;                //模式转变
        return;
     }
   }
   else
   {
     correct_art2_flag=OPEN;            //art4中断常开
     if(card_type!=0)
     {
       island_card_type=card_type;      //记录这张卡片的类型
       card_type=0;                     //处理完后及时清零
       correct_art2_flag=CLOSE;         //关闭
     }
     *Island_step=Island_Card_Classify_Pick;   //环岛的卡片分类
     return;
   }
   return;
}
 /*********************朝环岛外侧转向******************/
 else if(*Island_step==Car_Island_Turn_Outside)
 {
  if(fabsf(turn_outside_island_angle-Angle_Z)<=4)       //转向完成
  {
		car_stop();
		system_delay_ms(500);
    Island_back_flag = OPEN;                            //打开环岛后退里程计
    Angle_Island_back = 0;                              //清零角度
    Island_back_x = 0;
    Island_back_y = 0;                                  //清零里程计数
    right_lie_island_upline_position=0;                 //使用前先清零
    *Island_step=Car_Go_Ahead_Outside;                  //转变模式
    return;
  }
  else
  {
    Turn_Angle_PD(turn_outside_island_angle);           //准备Vz转速，向环岛外转向
    Vx=0;
    Vy=0;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Island_Turn_Outside;               //保持模式
    return;
  }
 }
 /***************************向环岛外侧直行**************************/
 else if(*Island_step==Car_Go_Ahead_Outside)
 {
  if(right_lie_island_upline_position>95 && right_lie_island_upline_position<118)               //最右列行坐标下降到70行
  {
    car_stop();
    system_delay_ms(500);
    right_lie_island_upline_position=0;                 //清零变量
    now_distance_x=0;
    now_distance_y=0;                                   //下一步要用的坐标清零
    NVIC_SetPriority(LPUART1_IRQn, 0);	//降低art1的中断优先级,为下一步art4再矫正做准备
    *Island_step=Car_Go_Find_Upline;                    //切换模式
    return;
  }
  else
  {
    right_lie_island_upline_position=Top_Top_Line_Search_Island(115,5,0);//持续从第100行往上扫上边线
    Vy=5;
    Vx=0;
    Turn_Angle_PD(turn_outside_island_angle);           //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Go_Ahead_Outside;
    return;
  }
  return;
 }

 /*******************对外侧扫描停车(停车扫描)********************/
 else if(*Island_step==Car_Go_Find_Upline)
 {
  if(now_distance_x > -500 && now_distance_x < 500 && now_distance_y> 350  && now_distance_y< 800)//设定一个识别区间(先暂时这样，后续可能会使用到世界坐标)，该判断条件更优先
  {
    car_stop();
    system_delay_ms(500);
    /**************再次打开环岛专用里程计(为下一步区域校正做准备)*************/
     Island_classify_flag=OPEN;                   //打开环岛里程计模式
     Angle_Island = 0;                            //第二次用来对环岛分类区域卡片做定位
     Island_x=0;
     Island_y=0;
     island_class_step=1;                         //校准步数初始化为1                                 
     delta_island_class_x=0;                           
     delta_island_class_y=0;                      //为下一步art1做分类区域定位做准备
     classify_correct_finish=NOT_FINISH;          //清除调整完毕的标志位，防止直接跳状态
     /***********************************************************************/
     record_island_zone_x=(float)now_distance_x;
     record_island_zone_y=(float)now_distance_y; //记录当前捕捉到的区域坐标，为下一步对准做准备,先不清零，要实时观察
    *Island_step=Car_Go_Island_Zone;             //去环岛的分类区域
    return;
  }
  else
  {
    car_stop();
    Vz=0;
    // Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Go_Find_Upline;
    return;
  }
  return;
 }
  /*******************对正入环前卡片的分类区域********************/
 else if(*Island_step==Car_Go_Island_Zone)
 {
  if(classify_correct_finish == FINISH)//对准完成
  {
    car_stop();
    system_delay_ms(500);
    //关闭环岛里程计，清空变量//
     Island_classify_flag=CLOSE;                 //关闭环岛里程计模式
     Angle_Island = 0;                           //清零角度
     Island_x=0;                                 //清零
     Island_y=0;     
    /************************/
    now_distance_x=0;
    now_distance_y=0;
    record_island_zone_x=0;
    record_island_zone_y=0;
    classify_correct_finish=NOT_FINISH;
    island_class_step=1;                                //校准步数初始化为1  
    delta_island_class_x=0;                           
    delta_island_class_y=0;                             //上一步校准所用的变量
    /************************/
    record_abc_flag = NOT_FINISH;                       //清空记录完成的标志位
    record_abc_card_type = 0;                           //清空记录过的标志位
    card_abc=0;                                         //先清零再使用
    classify_art2_flag=OPEN;                            //打开发送字母的art4中断
    correct_art2_flag=CLOSE;                            //关闭art4修正中断
    *Island_step=Island_Zone_Classify;                  //识别环岛的区域类型
    return;
  }
  else
  {
    find_island_classify(record_island_zone_x,record_island_zone_y);//环岛区域对正
    *Island_step=Car_Go_Island_Zone;
    return;
  }
  return;
 }
 /**********************环岛区域类型识别与卡片放置************************/
 else if(*Island_step==Island_Zone_Classify)
 {
  if(record_abc_flag == FINISH)                            //识别与记录完成
  {
    car_stop();                                            //清零速度
    system_delay_ms(500);
    record_abc_flag = NOT_FINISH;
    record_abc_card_type=0;
    card_abc=0;
    near_card_x=0;
    near_card_y=0;
    *Island_step=Step_Back_Island_Center;                  //后退到环岛中心(出发校正的那个点)
    return;
  }
  else
  {
    *Island_step=Island_Zone_Classify;                  //识别环岛的区域类型
    NVIC_SetPriority(LPUART1_IRQn, 2);                  //降低art1中断优先级
    classify_art2_flag=OPEN;                            //打开发送字母的art4中断
    if(card_abc!=0 && near_card_x!=666 && near_card_y!=666)
    {
      switch_abc_to_righttype(card_abc);//转化card_abc
      classify_art2_flag=CLOSE;                            //打开发送字母的art4中断
      for(uint8 i=0; i<5; i++)
     {
      if(record_abc_card_type==Island_card[i].Card_Type)
		  {
        island_found = READY;
        arm_control(5);
		    classify_little_360(Island_card[i].Card_PWM_Duty);//转到对应的舵机角度
        system_delay_ms(1000);
        arm_control(3);                                    //捡出卡片
        arm_control(4);                                    //归位
        record_abc_flag = FINISH;
        break;
		  }
     }
      if(!island_found)
     {
     // 如果没有找到匹配的Card_Type  
      record_abc_flag = NOT_FINISH;
      Island_Zone_count++;  
      *Island_step = Step_Back_Island_Center;
     return;  
     }
    }
    return;
  }
  return;
 }
//  /*******************后退到合适的位置*******************/
 else if(*Island_step==Step_Back_Island_Center)
 {
  if(back_flag==FINISH)         //回到出发前的位置
  {
		car_stop();
    Island_back_flag = CLOSE;   //关闭环岛后退得里程计
    Island_back_x = 0;
    Island_back_y = 0;          //清零所用变量
    
    back_flag=NOT_FINISH;
    right_lie_island_upline_position=0;
    turn_inside_island_zone_angle=turn_outside_island_angle-90;
    if(turn_inside_island_zone_angle>=360) 
    turn_inside_island_zone_angle=turn_inside_island_zone_angle-360;
	  if(turn_inside_island_zone_angle<=-360) 
    turn_inside_island_zone_angle=turn_inside_island_zone_angle+360;
    *Island_step=Car_Turn_Again_And_Again;
    return;
  }
  else
  {
    island_found = NOT_READY;                          //清零，未位后续环岛保底做准备
    Vy=Distance_pid(&distance_pid[0], 0, (int)Island_back_y);//回退至刚转向完成的位置
    Vx=0;
    Turn_Angle_PD(turn_outside_island_angle);           //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    // right_lie_island_upline_position=Top_Top_Line_Search_Island(119,50, 2);//持续从119行往上到40行找下边线数组最右列行坐标
    if(Island_back_y>-2 && Island_back_y<2)             //倒退至合理区间
    {
      back_flag=FINISH;
    }
    *Island_step=Step_Back_Island_Center;
    return;
  }
  return;
 }
//  /*************************转向环岛右侧区域的卡片**********************************/
else if(*Island_step==Car_Turn_Again_And_Again)
{
  if(fabsf(turn_inside_island_zone_angle-Angle_Z)<=4)       //转向成功
  {
    car_stop();
    system_delay_ms(500);
    right_lie_island_upline_position=0;
    shabi_saoxian_step=0;
    *Island_step=Car_Go_Island_Right_Zone;
    return;
  }
  else
  {
    Turn_Angle_PD(turn_inside_island_zone_angle);           //准备Vz转速，锁住车头
    Vx=0;
    Vy=0;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);            //麦轮控制，为target_speed赋值
    *Island_step=Car_Turn_Again_And_Again;
    return;
  }
  return;
}
/******************向向环岛右侧前进**********************************/
else if(*Island_step==Car_Go_Island_Right_Zone)
{
  if(ahead_flag==FINISH)                                                     //到达目标行
  {
    car_stop();
    // system_delay_ms(500);
    Angle_Island_back = 0;                                                   //环岛判断角度清零
    ahead_flag=NOT_FINISH;
    // right_lie_island_upline_position=0;                                   //清零行坐标
    *Island_step=Car_Go_Find_Upline_Inside_Island;                                      //巡上边线
    return;
  }
  else
  {
    Turn_Angle_PD(Angle_Z);                                    //准备Vz转速，锁住车头
    Vx=0;
    Vy=5;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);            //麦轮控制，为target_speed赋值
    if(shabi_saoxian_step==0)
    {
      right_lie_island_upline_position = Top_Top_Line_Search_Island(119,50, 0);//从119行开始往40行扫下边线，取最右列的行坐标
      if(right_lie_island_upline_position>=115)//扫第一条线，直至消失
    {
      shabi_saoxian_step=1;
    }
    }
    else if(shabi_saoxian_step==1)
    {
      second_right_lie_island_upline_position = Top_Top_Line_Search_Island(110, 70, 0);//扫上半屏幕
      if(second_right_lie_island_upline_position>80 && second_right_lie_island_upline_position<110)//防止丢线时误判
     {
       ahead_flag=FINISH;
     }
    }
    *Island_step=Car_Go_Island_Right_Zone;
    return;
  }
  return;
}
// /*****************************************环岛内巡上边线*****************************/
else if(*Island_step==Car_Go_Find_Upline_Inside_Island)
{
    if(now_distance_x > -200 && now_distance_x < 200 && now_distance_y> 100  && now_distance_y< 750)//设定一个识别区间(先暂时这样，后续可能会使用到世界坐标)，该判断条件更优先x区间限制只扫右平面 y区间屏蔽放下的卡片
  {
    record_island_catch_angle = Angle_Island_back;                    //记录下本次识别到的角度值
    delta_record_island_angle = record_island_catch_angle - last_record_island_catch_angle;//对比上一捕获到卡片时的角度
    last_record_island_catch_angle = record_island_catch_angle;
    if(fabsf(delta_record_island_angle)>=25 || Island_Zone_count==1)   //当卡片还是第一张的时候直接识别
    {
    car_stop();
    system_delay_ms(500);
    /**************再次打开环岛专用里程计(为下一步区域校正做准备)*************/
     Island_classify_flag=OPEN;                   //打开环岛里程计模式
     Angle_Island = 0;                            //第二次用来对环岛分类区域卡片做定位
     Island_x=0;
     Island_y=0;
     island_class_step=1;                         //校准步数初始化为1                                 
     delta_island_class_x=0;                           
     delta_island_class_y=0;                      //为下一步art1做分类区域定位做准备
     classify_correct_finish=NOT_FINISH;          //清除调整完毕的标志位，防止直接跳状态
     /***********************************************************************/
     record_island_zone_x=(float)now_distance_x;
     record_island_zone_y=(float)now_distance_y;        //记录当前捕捉到的区域坐标，为下一步对准做准备
     now_distance_x = 0;                                //清零
     now_distance_y = 0;                                //清零
    *Island_step=Car_Go_Island_Zone_Inside;             //去环岛的分类区域
    return;
    return;
    }
    else if(fabsf(delta_record_island_angle)<=25 && Island_Zone_count!=1)
    {
      now_distance_x = 0;                                //清零，重新检测
      now_distance_y = 0;                                //清零
      *Island_step=Car_Go_Find_Upline_Inside_Island;
      return;
    }
    return;
  }
  else
  {
    car_run_upline_right(island_target_upline);
    *Island_step=Car_Go_Find_Upline_Inside_Island;
    return;
  }
  return;
}
/**************************环岛内区域对准***************************/
else if(*Island_step==Car_Go_Island_Zone_Inside)
 {
  if(classify_correct_finish == FINISH)//对准完成
  {
    car_stop();
    system_delay_ms(500);
    //关闭环岛里程计，清空变量//
     Island_classify_flag=CLOSE;                  //关闭环岛里程计模式
     Angle_Island = 0;                           //清零角度
     Island_x=0;                                 //清零
     Island_y=0;     
    /************************/
    now_distance_x=0;
    now_distance_y=0;
    record_island_zone_x=0;
    record_island_zone_y=0;
    classify_correct_finish=NOT_FINISH;
    island_class_step=1;                                //校准步数初始化为1  
    delta_island_class_x=0;                           
    delta_island_class_y=0;                             //上一步校准所用的变量
    /************************/
    record_abc_flag = NOT_FINISH;                       //清空记录完成的标志位
    record_abc_card_type = 0;                           //清空记录过的标志位
    card_abc=0;                                         //先清零再使用
    classify_art2_flag=OPEN;                            //打开发送字母的art4中断
    correct_art2_flag=CLOSE;                            //关闭art4修正中断
    delay_place_flag=1;                                 //数秒5s
    *Island_step=Island_Zone_Classify_Inside;                  //识别环岛的区域类型
    return;
  }
  else
  {
    find_island_classify(record_island_zone_x,record_island_zone_y);//环岛区域对正
    *Island_step=Car_Go_Island_Zone_Inside;
    return;
  }
  return;
 }
//  /***********************环岛内区域识别***********************************/
 else if(*Island_step==Island_Zone_Classify_Inside)
 {
  if(record_abc_flag == FINISH)                            //识别与记录完成
  {

    Island_Zone_count++;
    record_abc_flag = NOT_FINISH;
    record_abc_card_type=0;
    card_abc=0;
    near_card_x=0;
    near_card_y=0;
    right_lie_island_upline_position=0;
    test_top_error=0;
    *Island_step=Step_Back_Island_Less;                     //后退一点
    return;
  }
  else
  {
    *Island_step=Island_Zone_Classify_Inside;               //识别环岛的区域类型
    NVIC_SetPriority(LPUART1_IRQn, 3);                      //降低art1中断优先级
    if(card_abc!=0 && near_card_x!=666 && near_card_y!=666)
    {
      switch_abc_to_righttype(card_abc);                    //转化card_abc
      classify_art2_flag=CLOSE;                            //打开发送字母的art4中断
      for (int i = 0; i < 5; i++) 
     {  
      if (record_abc_card_type == Island_card[i].Card_Type) 
      {  
        island_found = READY;  
        arm_control(5);  
        classify_little_360(Island_card[i].Card_PWM_Duty); // 转到对应的舵机角度  
        system_delay_ms(1000);  
        arm_control(3); // 捡出卡片  
        arm_control(4); // 归位  
        record_abc_flag = FINISH;  
        break; // 找到匹配项后退出循环  
      }  
     }  
      if (!island_found)//当没找到或者停留秒数超过10s时 
      {  
        // 如果没有找到匹配的Card_Type  
        record_abc_flag = NOT_FINISH;
        Island_Zone_count++;
        back_flag = NOT_FINISH;  
        *Island_step = Step_Back_Island_Less;
        return;  
      }
    }
   return;   
  }
  return;   
}
//  /****************************环岛内后退(后退至可寻迹行)******************************/
 if(*Island_step==Step_Back_Island_Less)
 {
  if(back_flag==FINISH)         //最右侧行坐标上升至100行
  {
    if(Island_Zone_count==5)                                                               //已经完成5张的分类
   {
      now_distance_x=0;
      now_distance_y=0;                                                                      //一定要清零，不然会看到假的坐标
      turn_out_Island_angle = now_Island_angle - 90;                                                  //准备出环角度
      if(turn_out_Island_angle>=360) 
        turn_out_Island_angle=turn_out_Island_angle-360;
	    if(turn_out_Island_angle<=-360) 
        turn_out_Island_angle=turn_out_Island_angle+360;
      *Island_step=Car_Turn_Island_Outside_Again;                                            //模式转变，准备出环
      return;
   }
    NVIC_SetPriority(LPUART1_IRQn, 0);                                                       //恢复art1中断优先级
    car_stop();                                                                              //清空速度
    now_distance_x=0;                                                                        //清空坐标，重新检测
    now_distance_y=0;
    back_flag=NOT_FINISH;
    right_lie_island_upline_position=0;
    test_top_error=0;
    *Island_step=Car_Go_Find_Upline_Inside_Island;                                            //环岛内巡上边线
    return;
  }
  else 
  {
    island_found = NOT_READY;                           //清零，未后续保底做准备
    allow_flag=OPEN;
    island_found = NOT_FINISH;//重置为未找到卡片
    Vy=-2;
    Vx=0;
    Vz=0;
    Turn_Angle_PD(Angle_Z);                             //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    Top_Line_Search();                 // 对上边线扫线，为上边线数组做准备
    test_top_error = Top_Line_Err_Right(island_target_upline)/25; // 对上边线扫线输出一个误差,作归一化处理
    if(test_top_error < 0 && test_top_error >-0.7)
    {
      back_flag=FINISH;
    }
    *Island_step=Step_Back_Island_Less;
    return;
  }
  return;
 }
 ////////////////////出环/////////////////////
 //*********************************向环岛外转向***************************/
 else if(*Island_step==Car_Turn_Island_Outside_Again)
 {
  if(fabsf(turn_out_Island_angle-Angle_Z)<=4)
  {
		car_stop();
		system_delay_ms(500);
    right_lie_island_upline_position=0;                 //使用前先清零
    *Island_step=Car_Go_Island_Outside_Again;           //转变模式
    return;
  }
  else
  {
    Turn_Angle_PD(turn_out_Island_angle);           //准备Vz转速，向环岛外转向
    Vx=0;
    Vy=0;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Turn_Island_Outside_Again;               //保持模式
    return;
  }
  return;
 }
 /***********************************向上寻找目标行*************************/
 else if(*Island_step==Car_Go_Island_Outside_Again)
 {
  if(right_lie_island_upline_position>95 && right_lie_island_upline_position<115)               //最右列行坐标下降到95行
  {
    car_stop();
    system_delay_ms(500);
    right_lie_island_upline_position=0;                 //清零变量
    *Island_step=Car_Turn_Out_Island;                    //切换模式
    return;
  }
  else
  {
    right_lie_island_upline_position=Top_Top_Line_Search_Island(115,5,2);//持续从第100行往上扫上边线
    Vy=5;
    Vx=0;
    Turn_Angle_PD(Angle_Z);           //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Go_Island_Outside_Again;
    return;
  }
 }
 else if (*Island_step==Car_Turn_Out_Island)
 {
  if(now_Island_angle - Angle_Z<= 4)
  {
    car_stop();
    Left_Island_Done=FINISH;
    *Island_step = Arrive_zeropoint;//步数回到第0步
    banmaxian_allow_flag = READY;   //重新打开斑马线
    Cross_allow_flag = READY;       //重新打开十字
    left_island_flag = 0;
    right_island_flag = 0;          //清零环岛标志位
    Left_Island_Finish = FINISH;    //左圆环完成标志位
    island_stop_flag=1;             //开始停止检测环岛
    normal_stop_flag=1;             //正常路边检测标志位
    return;
  }
  else
  {
    Vy=0;
    Vx=0;
    Turn_Angle_PD(now_Island_angle);                    //准备Vz转速，向赛道外转向
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Turn_Out_Island;
    return;
  }
  return;
 }
}// /**
// * @brief 右环岛多搬策略
// * @param Island_step为步数选择
// * @param 该函数是判断出为右环岛类型时才使用
// * @return 无
// */
extern int Island_State;

int Right_Island_mode=0;							//右环岛模式选择
uint8 Right_Island_allow_flag = READY;//右环岛允许处理标志
uint8 Right_Island_Done = NOT_FINISH;
uint8 allow_right_flag=OPEN;
double Right_Island_classify_zone_x,Right_Island_classify_zone_y;
double delta_find_Island_zero_angle; 
float now_right_Island_angle;//记录进入右圆环处理时的角度
float turn_right_IsLand_angel;//右圆环转向使用角度
float turn_outside_right_island_angle;//转向右圆环外侧的角度
float record_right_island_zone_x,record_right_island_zone_y;//记录右环岛外侧区域坐标
int record_right_abc_flag = NOT_FINISH;         //记录完成的标志位
float turn_inside_island_zone_angle;//回到中心后的转向角
int Right_Island_Zone_count=0;//识别出的右环岛区域数目
float test_right_top_error;   //右环岛上边线巡线时的误差
int right_island_target_upline=105;//右环岛上边线巡线的目标行
int right_island_found = NOT_READY;//右环岛环岛防置卡片区域的匹配标志位
float turn_out_right_Island_angle; //右圆环出环角度
/***************记录环岛的区域解算坐标和它的类型**************/
uint8 arrive_right_island_center_flag=NOT_READY;//到达右环岛处理点的位置
float right_island_card_center_x,right_island_card_center_y;//art4记录的右环岛卡片坐标，矫正使用
uint8 Find_Right_Island_Card_Position=NOT_READY;    //记录向右环岛中心卡片捕获的标志位,初始化为未完成
float right_island_correct_again_x,right_island_correct_again_y;//环岛卡片微调的输入坐标
int right_island_card_type=0;                       //记录识别的右环岛卡片类型
int right_lie_right_island_upline_position;//环岛扫上边线最右列的行坐标
uint8 right_shabi_saoxian_step=0;//傻逼扫线的步数
int second_right_lie_right_island_upline_position;//扫两次的，防止直接扫到内圆
float record_right_island_catch_angle,last_record_right_island_catch_angle;//环岛内角度记录
float delta_record_island_angle;                               //环岛内捕获卡片时的角度差值
uint8 Right_Island_Finish = NOT_FINISH;
float turn_out_crossing_angle;
//float turn_outside_island_angle=-90;					//测试用的，一定要记得删掉！！！！！！！！！！！！！！！！

 void Right_Island_pick_and_move(int *
	 Island_step)
 {
 /*********************去原点处的判断******************/
  if(*Island_step==Arrive_zeropoint)              //去环岛的原点处
 {
   if(arrive_right_island_center_flag==READY)        //到达与最右侧的区域的环岛区域
   {
     car_stop();
     system_delay_ms(500);
     arrive_right_island_center_flag=NOT_READY;
    //  Angle_Island = 0;                            //第一次用来对停车的位置做一个初步定位
    // //  Island_classify_flag=OPEN;                   //打开环岛里程计模式
    //  Island_x=0;
    //  Island_y=0;                                  //环岛里程计清零
      // now_distance_x=0;
      // now_distance_y=0;
      now_right_Island_angle = Angle_Z;      //记录下环岛转向前的角度
      Angle_Island_Zero = 0;           //专门用来圆环转向的角度
     turn_right_IsLand_angel = Angle_Z+90;  //向左转向90度
      if(turn_right_IsLand_angel>=360) turn_right_IsLand_angel=turn_right_IsLand_angel-360;
	    if(turn_right_IsLand_angel<=-360) turn_right_IsLand_angel=turn_right_IsLand_angel+360;
     *Island_step=Car_Island_turn;    //向圆环中心区域转向
     return;
   }
   else
   {
    /***********************/
    banmaxian_allow_flag = NOT_READY; //关闭斑马线标志
    Cross_allow_flag = NOT_READY;//关闭十字
    car_run();
    if(Island_State==3)
    {
      arrive_island_center_flag=READY;
    }
     *Island_step=Arrive_zeropoint;
     return;
   }
   return;
 }
 /*********************转向环岛中心区域******************/
  else if(*Island_step==Car_Island_turn)           //车头转向环岛外侧
 {
   if(abs(turn_right_IsLand_angel-Angle_Z)<=3)        //摄像头朝向环岛的内侧
   {
     car_stop();
     system_delay_ms(500);
    //  correct_art2_flag=OPEN;                        //打开art4识别类型
     Find_Right_Island_Card_Position=NOT_READY;           //进入前先清零调整完毕的坐标
     /********打开用来环岛修正的里程计(必须)*******/
     Island_classify_flag=OPEN;                  //打开环岛里程计模式
     Angle_Island = 0;                           //第一次用来对环岛中心卡片做定位
     Island_x=0;
     Island_y=0;                                 
     delta_island_x=0;                           
     delta_island_y=0;                           //为下一步art1定位做准备
     CSI_island_correct_flag =  NOT_FINISH;      //总钻风调整完毕标志清除
     /*********到时候有个总钻风识别坐标，可以动态调整，目前先用art4调坐标*********/
     right_island_card_center_x=0;
     right_island_card_center_y=0;
    //  now_distance_x=0;
    //  now_distance_y=0;
     *Island_step=Island_Card_Correct;              //转变模式 
     return;   
   }
   else
   {
     Turn_Angle_PD(turn_right_IsLand_angel);
     Vx=0;
     Vy=0;
     Car_Inverse_kinematics_solution(Vx, Vy, Vz);              //麦轮控制，为target_speed赋值
     *Island_step=Car_Island_turn;
     return;
   }
   return;
 }
/*********************对环岛卡片对正******************/
else if(*Island_step==Island_Card_Correct)  
 {
   if(CSI_island_correct_flag == FINISH)//定位完成
   {
     NVIC_SetPriority(LPUART1_IRQn, 2);	//降低art1的中断优先级,为下一步art4再矫正做准备
     car_stop();
     system_delay_ms(500);
     /**************清零变量与下一步的变量准备*************/
     CSI_island_correct_flag = NOT_FINISH;
     right_island_card_center_x=0;        //用完就清
     right_island_card_center_y=0;
     Island_classify_flag=CLOSE;                 //关闭环岛里程计模式
     Angle_Island = 0;                           //第二次用来对环岛中心卡片做定位
     Island_x=0;
     Island_y=0;                                 
     delta_island_x=0;                           
     delta_island_y=0;
     /****************************************/
     now_distance_x=0;
     now_distance_y=0;
     near_card_x=0;                 //准备用才清
     near_card_y=0;
     right_island_correct_again_x=0;
     right_island_correct_again_y=0;
     /**************为art4对准做准备*************/
     arrive_card_flag = OPEN;       //开启修正用的里程计
     Angle_arrive_card = 0;
     correct_x=0;
     correct_y=0;                   //清零修正里程
     delta_x=0;
     delta_y=0;                     //art修正用的变量
     CSI_island_center_correct_flag = NOT_FINISH; //art4完成修正的标志位
     /**************清零/重启标志位***************/
     allow_flag=OPEN;                             //重新打开allow_flag
     correct_art2_flag=OPEN;                      //打开art4中断
     Find_Right_Island_Card_Position=NOT_READY;         //清空记录标志位
    /****************************************/
    *Island_step=Island_Card_Correct_again;       //环岛的卡片再次对正
    return;
   }
   else
   {
    if(now_distance_x != 0 && now_distance_y> 100 && now_distance_y <600)      //目前先使用art1做矫正
    {
      if(allow_flag==OPEN)
      {
        if(now_distance_x > 0)
        {
           right_island_card_center_x=(float)now_distance_x;           
        }
        if(now_distance_x < 0)
        {
           right_island_card_center_x=-(float)now_distance_x;           
        }
			right_island_card_center_y=(float)now_distance_y;                                   //记录下环岛的卡片坐标，做矫正使用
      // correct_art2_flag=CLOSE;                                            //关闭atr4的中断
      allow_flag=CLOSE;
      }
    }
      CSI_correct_island_correct(right_island_card_center_x, right_island_card_center_y);//art1坐标对正，准备x,y速度,只做一个初步校正
      Turn_Angle_PD(turn_right_IsLand_angel);                                      //准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);                           //麦轮控制，为target_speed赋值
    *Island_step=Island_Card_Correct;//保持模式
    return;
   }
   return;
 }
 /**************对环岛中心卡片用art4再对准一遍中心****************/
else if(*Island_step==Island_Card_Correct_again)
{
  if(CSI_island_center_correct_flag == FINISH)                  //art4对准完成
  {
     allow_flag=OPEN;
     car_stop();                    //清零速度，防止乱动
     /************清零使用过的变量**********/
     arrive_card_flag = CLOSE;      //关闭修正用的里程计
     correct_x=0;
     correct_y=0;                   //清零修正里程
     delta_x=0;
     delta_y=0;                     //art修正用的变量
     CSI_island_center_correct_flag = NOT_FINISH; //清零矫正完成的标志位
     right_island_card_type=0;
     card_type=0;                   //这个变量虽然没有用到但其实已经赋值，为下一步art4做准备
     card_classify_count=0;         //为下一步分类做准备
     /*************************************/
     *Island_step=Island_Card_Classify_Pick;//转变模式
     return;
  }
  else
  {
    *Island_step=Island_Card_Correct_again;       //保持模式

    if(near_card_x != 0 && near_card_y >= 150 && near_card_x !=666 && near_card_y!=666)    //有正确的卡片坐标传入
    {
      if(allow_flag==OPEN)                        //只执行一次
      {
        right_island_correct_again_x=(float)near_card_x;
        right_island_correct_again_y=(float)near_card_y+12;       //再次微调的art4输入距离
        correct_art2_flag=CLOSE;                  //关闭art4修正中断
        allow_flag=CLOSE;
      }
    }
      CSI_dis_island_correct(right_island_correct_again_x, right_island_correct_again_y);//art4坐标对正，准备x,y速度
      Turn_Angle_PD(turn_right_IsLand_angel);           //准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);  //麦轮控制，为target_speed赋值
      return;
  }
  return;
}       
 /*********************对环岛中心卡片分类******************/
else if(*Island_step==Island_Card_Classify_Pick)//环岛卡片的分类
{
   if(right_island_card_type!=0)              //卡片类型已记录
   {
     if(card_classify_count<5)
     {
      Island_card[card_classify_count].Card_Type=right_island_card_type;    //记录下该张卡片类型
      Island_card[card_classify_count].Card_PWM_Duty=card_classify_count;      //记录该张卡片在舵机转动中的类别
      classify_little_360(card_classify_count);
      arm_control(2);//捡卡片 
	    arm_control(4);//默认模式                               //卡片拾取完成
      right_island_card_type=0;                                     //清零，等待下一次的识别传入
      card_classify_count++;                                  //卡片数量加一
      *Island_step=Island_Card_Classify_Pick;                 //继续进去环岛的卡片分类和拾取状态
      return;
     }
     else
     {
        correct_art2_flag=CLOSE;         //关闭atr4中断
        turn_outside_right_island_angle=now_right_Island_angle-90;       //为下一个角度转向做准备
       if(turn_outside_right_island_angle>=360) 
        turn_outside_right_island_angle=turn_outside_right_island_angle-360;
	     if(turn_outside_right_island_angle<=-360) 
        turn_outside_right_island_angle=turn_outside_right_island_angle+360;
        *Island_step=Car_Island_Turn_Outside;                //模式转变
        return;
     }
   }
   else
   {
     correct_art2_flag=OPEN;            //art4中断常开
     if(card_type!=0)
     {
       right_island_card_type=card_type;      //记录这张卡片的类型
       card_type=0;                     //处理完后及时清零
       correct_art2_flag=CLOSE;         //关闭
     }
     *Island_step=Island_Card_Classify_Pick;   //环岛的卡片分类
     return;
   }
   return;
}
 /*********************朝环岛外侧转向******************/
 else if(*Island_step==Car_Island_Turn_Outside)
 {
  if(fabsf(turn_outside_right_island_angle-Angle_Z)<=4)
  {
		car_stop();
		system_delay_ms(500);
    right_lie_right_island_upline_position=0;                 //使用前先清零
    *Island_step=Car_Go_Ahead_Outside;                  //转变模式
    return;
  }
  else
  {
    Turn_Angle_PD(turn_outside_right_island_angle);           //准备Vz转速，向环岛外转向
    Vx=0;
    Vy=0;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Island_Turn_Outside;               //保持模式
    return;
  }
 }
 /***************************向环岛外侧直行**************************/
 else if(*Island_step==Car_Go_Ahead_Outside)
 {
  if(right_lie_right_island_upline_position>95 && right_lie_right_island_upline_position<118)               //最右列行坐标下降到70行
  {
    car_stop();
    system_delay_ms(500);
    right_lie_right_island_upline_position=0;                 //清零变量
    now_distance_x=0;
    now_distance_y=0;                                   //下一步要用的坐标清零
    NVIC_SetPriority(LPUART1_IRQn, 0);	//降低art1的中断优先级,为下一步art4再矫正做准备
    *Island_step=Car_Go_Find_Upline;                    //切换模式
    return;
  }
  else
  {
    right_lie_right_island_upline_position=Top_Top_Line_Search_Island(115,5,0);//持续从第100行往上扫上边线
    Vy=5;
    Vx=0;
    Turn_Angle_PD(turn_outside_right_island_angle);           //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Go_Ahead_Outside;
    return;
  }
  return;
 }

 /*******************对外侧扫描停车(巡上边线绕行)********************/
 else if(*Island_step==Car_Go_Find_Upline)
 {
  if(now_distance_x > -500 && now_distance_x < 500 && now_distance_y> 350  && now_distance_y< 800)//设定一个识别区间(先暂时这样，后续可能会使用到世界坐标)，该判断条件更优先
  {
    car_stop();
    system_delay_ms(500);
    /**************再次打开环岛专用里程计(为下一步区域校正做准备)*************/
     Island_classify_flag=OPEN;                   //打开环岛里程计模式
     Angle_Island = 0;                            //第二次用来对环岛分类区域卡片做定位
     Island_x=0;
     Island_y=0;
     island_class_step=1;                         //校准步数初始化为1                                 
     delta_island_class_x=0;                           
     delta_island_class_y=0;                      //为下一步art1做分类区域定位做准备
     classify_correct_finish=NOT_FINISH;          //清除调整完毕的标志位，防止直接跳状态
     /***********************************************************************/
     record_right_island_zone_x=(float)now_distance_x;
     record_right_island_zone_y=(float)now_distance_y;        //记录当前捕捉到的区域坐标，为下一步对准做准备,先不清零，要实时观察
    *Island_step=Car_Go_Island_Zone;             //去环岛的分类区域
    return;
  }
  else
  {
    car_stop();
    Vz=0;
    // Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Go_Find_Upline;
    return;
  }
  return;
 }
  /*******************去卡片的分类区域********************/
 else if(*Island_step==Car_Go_Island_Zone)
 {
  if(classify_correct_finish == FINISH)//对准完成
  {
    car_stop();
    system_delay_ms(500);
    //关闭环岛里程计，清空变量//
     Island_classify_flag=CLOSE;                 //关闭环岛里程计模式
     Angle_Island = 0;                           //清零角度
     Island_x=0;                                 //清零
     Island_y=0;     
     Right_Island_Zone_count=1;                        //此时为环岛外的卡片，置为1
    /************************/
    now_distance_x=0;
    now_distance_y=0;
    record_right_island_zone_x=0;
    record_right_island_zone_y=0;
    classify_correct_finish=NOT_FINISH;
    island_class_step=1;                                //校准步数初始化为1  
    delta_island_class_x=0;                           
    delta_island_class_y=0;                             //上一步校准所用的变量
    /************************/
    record_right_abc_flag = NOT_FINISH;                       //清空记录完成的标志位
    record_abc_card_type = 0;                           //清空记录过的标志位
    card_abc=0;                                         //先清零再使用
    classify_art2_flag=OPEN;                            //打开发送字母的art4中断
    correct_art2_flag=CLOSE;                            //关闭art4修正中断
    *Island_step=Island_Zone_Classify;                  //识别环岛的区域类型
    return;
  }
  else
  {
    find_island_classify(record_right_island_zone_x,record_right_island_zone_y);//环岛区域对正
    *Island_step=Car_Go_Island_Zone;
    return;
  }
  return;
 }
 /**********************环岛区域类型识别与卡片放置************************/
 else if(*Island_step==Island_Zone_Classify)
 {
  if(record_right_abc_flag == FINISH)                            //识别与记录完成
  {
    car_stop();                                            //清零速度
    system_delay_ms(500);
    record_right_abc_flag = NOT_FINISH;
    record_abc_card_type=0;
    card_abc=0;
    near_card_x=0;
    near_card_y=0;
    *Island_step=Step_Back_Island_Center;                  //后退到环岛中心(出发校正的那个点)
    return;
  }
  else
  {
    *Island_step=Island_Zone_Classify;                  //识别环岛的区域类型
    NVIC_SetPriority(LPUART1_IRQn, 2);                  //降低art1中断优先级
    classify_art2_flag=OPEN;                            //打开发送字母的art4中断
    if(card_abc!=0 && near_card_x!=666 && near_card_y!=666)
    {
      switch_abc_to_righttype(card_abc);//转化card_abc
      classify_art2_flag=CLOSE;                            //打开发送字母的art4中断
      for(uint8 i=0; i<5; i++)
     {
      if(record_abc_card_type==Island_card[i].Card_Type)
		  {
        right_island_found = READY;
        arm_control(5);
		    classify_little_360(Island_card[i].Card_PWM_Duty);//转到对应的舵机角度
        system_delay_ms(1000);
        arm_control(3);                                    //捡出卡片
        arm_control(4);                                    //归位
        record_right_abc_flag = FINISH;
        break;
		  }
     }
      if(!right_island_found)
     {
     // 如果没有找到匹配的Card_Type  
      record_right_abc_flag = NOT_FINISH;
      Right_Island_Zone_count++;  
      *Island_step = Step_Back_Island_Center;
     return;  
     }
    }
    return;
  }
  return;
 }
//  /*******************后退到合适的位置*******************/
 else if(*Island_step==Step_Back_Island_Center)
 {
  if(back_flag==FINISH)         //最右侧行坐标上升至40行
  {
		car_stop();
    back_flag=NOT_FINISH;
    right_lie_right_island_upline_position=0;
    turn_inside_island_zone_angle=turn_outside_right_island_angle-90;
    if(turn_inside_island_zone_angle>=360) 
    turn_inside_island_zone_angle=turn_inside_island_zone_angle-360;
	  if(turn_inside_island_zone_angle<=-360) 
    turn_inside_island_zone_angle=turn_inside_island_zone_angle+360;
    *Island_step=Car_Turn_Again_And_Again;
    return;
  }
  else
  {
    right_island_found = NOT_READY;                          //清零，未=位后续环岛保底做准备
    Vy=-5;
    Vx=0;
    Turn_Angle_PD(turn_outside_right_island_angle);           //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    right_lie_right_island_upline_position=Top_Top_Line_Search_Island(119,30, 2);//持续从110行往上到20行找下边线数组中间行坐标
    if(right_lie_right_island_upline_position<60  && right_lie_right_island_upline_position>50)
    {
      back_flag=FINISH;
    }
    *Island_step=Step_Back_Island_Center;
    return;
  }
  return;
 }
//  /*************************转向环岛右侧区域的卡片**********************************/
else if(*Island_step==Car_Turn_Again_And_Again)
{
  if(fabsf(turn_inside_island_zone_angle-Angle_Z)<=4)       //转向成功
  {
    car_stop();
    system_delay_ms(500);
    right_lie_right_island_upline_position=0;
    right_shabi_saoxian_step=0;
    *Island_step=Car_Go_Island_Right_Zone;
    return;
  }
  else
  {
    Turn_Angle_PD(turn_inside_island_zone_angle);           //准备Vz转速，锁住车头
    Vx=0;
    Vy=0;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);            //麦轮控制，为target_speed赋值
    *Island_step=Car_Turn_Again_And_Again;
    return;
  }
  return;
}
/******************向向环岛右侧前进**********************************/
else if(*Island_step==Car_Go_Island_Right_Zone)
{
  if(ahead_flag==FINISH)        //到达目标行
  {
    car_stop();
    // system_delay_ms(500);
    Angle_Island_back = 0;                                                   //环岛判断角度清零
    ahead_flag=NOT_FINISH;
    // right_lie_island_upline_position=0;                                   //清零行坐标
    *Island_step=Car_Go_Find_Upline_Inside_Island;                                      //巡上边线
    return;
  }
  else
  {
    Turn_Angle_PD(Angle_Z);                                    //准备Vz转速，锁住车头
    Vx=0;
    Vy=5;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);            //麦轮控制，为target_speed赋值
    if(right_shabi_saoxian_step==0)
    {
      right_lie_right_island_upline_position = Top_Top_Line_Search_Island(119,50, 0);//从119行开始往40行扫下边线，取最右列的行坐标
      if(right_lie_right_island_upline_position>=115)//扫第一条线，直至消失
    {
      right_shabi_saoxian_step=1;
    }
    }
    else if(right_shabi_saoxian_step==1)
    {
      right_lie_right_island_upline_position = Top_Top_Line_Search_Island(110, 70, 0);//扫上半屏幕
      if(second_right_lie_right_island_upline_position>80 && second_right_lie_right_island_upline_position<110)//防止丢线时误判
     {
       ahead_flag=FINISH;
     }
    }
    *Island_step=Car_Go_Island_Right_Zone;
    return;
  }
  return;
}
// /*****************************************环岛内巡上边线*****************************/
else if(*Island_step==Car_Go_Find_Upline_Inside_Island)
{
    if(now_distance_x > -500 && now_distance_x < 200 && now_distance_y> 100  && now_distance_y< 750)//设定一个识别区间(先暂时这样，后续可能会使用到世界坐标)，该判断条件更优先x区间限制只扫右平面 y区间屏蔽放下的卡片
  {
    record_right_island_catch_angle = Angle_Island_back;                    //记录下本次识别到的角度值
    delta_record_island_angle = record_right_island_catch_angle - last_record_right_island_catch_angle;//对比右环岛内上一捕获到卡片时的角度
    last_record_island_catch_angle = record_right_island_catch_angle;
    if(fabsf(delta_record_island_angle)>=25 || Right_Island_Zone_count==1)   //当卡片还是第一张的时候直接识别
    {
    car_stop();
    system_delay_ms(500);
    /**************再次打开环岛专用里程计(为下一步区域校正做准备)*************/
     Island_classify_flag=OPEN;                   //打开环岛里程计模式
     Angle_Island = 0;                            //第二次用来对环岛分类区域卡片做定位
     Island_x=0;
     Island_y=0;
     island_class_step=1;                         //校准步数初始化为1                                 
     delta_island_class_x=0;                           
     delta_island_class_y=0;                      //为下一步art1做分类区域定位做准备
     classify_correct_finish=NOT_FINISH;          //清除调整完毕的标志位，防止直接跳状态
     /***********************************************************************/
     record_right_island_zone_x=(float)now_distance_x;
     record_right_island_zone_y=(float)now_distance_y;        //记录当前捕捉到的区域坐标，为下一步对准做准备
     now_distance_x = 0;                                //清零
     now_distance_y = 0;                                //清零
    *Island_step=Car_Go_Island_Zone_Inside;             //去环岛的分类区域
    return;
    }
    else if(fabsf(delta_record_island_angle)<=25 && Right_Island_Zone_count!=1)
    {
      now_distance_x = 0;                                //清零，重新检测
      now_distance_y = 0;                                //清零
      *Island_step=Car_Go_Find_Upline_Inside_Island;
      return;
    }
    return;
  }
  else
  {
    car_run_upline_right(right_island_target_upline);
    *Island_step=Car_Go_Find_Upline_Inside_Island;
    return;
  }
  return;
}
/**************************环岛内区域对准***************************/
else if(*Island_step==Car_Go_Island_Zone_Inside)
 {
  if(classify_correct_finish == FINISH)//对准完成
  {
    car_stop();
    system_delay_ms(500);
    //关闭环岛里程计，清空变量//
     Island_classify_flag=CLOSE;                  //关闭环岛里程计模式
     Angle_Island = 0;                           //清零角度
     Island_x=0;                                 //清零
     Island_y=0;     
    /************************/
    now_distance_x=0;
    now_distance_y=0;
    record_right_island_zone_x=0;
    record_right_island_zone_y=0;
    classify_correct_finish=NOT_FINISH;
    island_class_step=1;                                //校准步数初始化为1  
    delta_island_class_x=0;                           
    delta_island_class_y=0;                             //上一步校准所用的变量
    /************************/
    record_right_abc_flag = NOT_FINISH;                       //清空记录完成的标志位
    record_abc_card_type = 0;                           //清空记录过的标志位
    card_abc=0;                                         //先清零再使用
    classify_art2_flag=OPEN;                            //打开发送字母的art4中断
    correct_art2_flag=CLOSE;                            //关闭art4修正中断
    delay_place_flag=1;                                 //数秒5s
    *Island_step=Island_Zone_Classify_Inside;                  //识别环岛的区域类型
    return;
  }
  else
  {
    find_island_classify(record_right_island_zone_x,record_right_island_zone_y);//环岛区域对正
    *Island_step=Car_Go_Island_Zone_Inside;
    return;
  }
  return;
 }
//  /***********************环岛内区域识别***********************************/
 else if(*Island_step==Island_Zone_Classify_Inside)
 {
  if(record_right_abc_flag == FINISH)                            //识别与记录完成
  {

    Right_Island_Zone_count++;
    record_right_abc_flag = NOT_FINISH;
    record_abc_card_type=0;
    card_abc=0;
    near_card_x=0;
    near_card_y=0;
    right_lie_right_island_upline_position=0;
    test_right_top_error=0;
    *Island_step=Step_Back_Island_Less;                     //后退一点
    return;
  }
  else
  {
    *Island_step=Island_Zone_Classify_Inside;               //识别环岛的区域类型
    NVIC_SetPriority(LPUART1_IRQn, 3);                      //降低art1中断优先级
    if(card_abc!=0 && near_card_x!=666 && near_card_y!=666)
    {
      switch_abc_to_righttype(card_abc);                    //转化card_abc
      classify_art2_flag=CLOSE;                            //打开发送字母的art4中断
      for (int i = 0; i < 5; i++) 
     {  
      if (record_abc_card_type == Island_card[i].Card_Type) 
      {  
        right_island_found = READY;  
        arm_control(5);  
        classify_little_360(Island_card[i].Card_PWM_Duty); // 转到对应的舵机角度  
        system_delay_ms(1000);  
        arm_control(3); // 捡出卡片  
        arm_control(4); // 归位  
        record_right_abc_flag = FINISH;  
        break; // 找到匹配项后退出循环  
      }  
     }  
      if (!right_island_found)//当没找到或者停留秒数超过10s时 
      {  
        // 如果没有找到匹配的Card_Type  
        record_right_abc_flag = NOT_FINISH;
        Right_Island_Zone_count++;
        back_flag = NOT_FINISH;  
        *Island_step = Step_Back_Island_Less;
        return;  
      }
    }
   return;   
  }
  return;   
}
//  /****************************环岛内后退(后退至可寻迹行)******************************/
 if(*Island_step==Step_Back_Island_Less)
 {
  if(back_flag==FINISH)         //最右侧行坐标上升至100行
  {
    if(Right_Island_Zone_count==5)                                                               //已经完成5张的分类
   {
      now_distance_x=0;
      now_distance_y=0;                                                                      //一定要清零，不然会看到假的坐标
      turn_out_right_Island_angle = now_right_Island_angle - 90;                                                  //准备出环角度
      if(turn_out_right_Island_angle>=360) 
        turn_out_right_Island_angle=turn_out_right_Island_angle-360;
	    if(turn_out_right_Island_angle<=-360) 
        turn_out_right_Island_angle=turn_out_right_Island_angle+360;
      *Island_step=Car_Turn_Island_Outside_Again;                                            //模式转变，准备出环
      return;
   }
    NVIC_SetPriority(LPUART1_IRQn, 0);                                                       //恢复art1中断优先级
    car_stop();                                                                              //清空速度
    now_distance_x=0;                                                                        //清空坐标，重新检测
    now_distance_y=0;
    back_flag=NOT_FINISH;
    right_lie_right_island_upline_position=0;
    test_right_top_error=0;
    *Island_step=Car_Go_Find_Upline_Inside_Island;                                            //环岛内巡上边线
    return;
  }
  else 
  {
    right_island_found = NOT_READY;                           //清零，未后续保底做准备
    allow_flag=OPEN;
    Vy=-2;
    Vx=0;
    Vz=0;
    Turn_Angle_PD(Angle_Z);                             //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    Top_Line_Search();                 // 对上边线扫线，为上边线数组做准备
    test_right_top_error = Top_Line_Err_Right(right_island_target_upline)/25; // 对上边线扫线输出一个误差,作归一化处理
    if(test_right_top_error < 0 && test_right_top_error >-0.7)
    {
      back_flag=FINISH;
    }
    *Island_step=Step_Back_Island_Less;
    return;
  }
  return;
 }
 ////////////////////出环/////////////////////
 //*********************************向环岛外转向***************************/
 else if(*Island_step==Car_Turn_Island_Outside_Again)
 {
  if(fabsf(turn_out_right_Island_angle-Angle_Z)<=4)
  {
		car_stop();
		system_delay_ms(500);
    right_lie_right_island_upline_position=0;                 //使用前先清零
    *Island_step=Car_Go_Island_Outside_Again;           //转变模式
    return;
  }
  else
  {
    Turn_Angle_PD(turn_out_right_Island_angle);           //准备Vz转速，向环岛外转向
    Vx=0;
    Vy=0;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Turn_Island_Outside_Again;               //保持模式
    return;
  }
  return;
 }
 /***********************************向上寻找目标行*************************/
 else if(*Island_step==Car_Go_Island_Outside_Again)
 {
  if(right_lie_right_island_upline_position>95 && right_lie_right_island_upline_position<115)               //最右列行坐标下降到95行
  {
    car_stop();
    system_delay_ms(500);
    right_lie_right_island_upline_position=0;                 //清零变量
    *Island_step=Car_Turn_Out_Island;                    //切换模式
    return;
  }
  else
  {
    right_lie_right_island_upline_position=Top_Top_Line_Search_Island(115,5,2);//持续从第100行往上扫上边线
    Vy=5;
    Vx=0;
    Turn_Angle_PD(turn_outside_right_island_angle);           //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Go_Island_Outside_Again;
    return;
  }
 }
 else if (*Island_step==Car_Turn_Out_Island)
 {
  if(now_right_Island_angle - Angle_Z<= 4)
  {
    car_stop();
    Left_Island_Done=FINISH;
    *Island_step = Arrive_zeropoint;//步数回到第0步
    banmaxian_allow_flag = READY;   //重新打开斑马线
    Cross_allow_flag = READY;       //重新打开十字
    left_island_flag = 0;
    right_island_flag = 0;          //清零环岛标志位
    Right_Island_Finish = FINISH;    //左圆环完成标志位
    island_stop_flag=1;//开始停止检测环岛
    normal_stop_flag=1;//正常路边检测标志位
    return;
  }
  else
  {
    Vy=0;
    Vx=0;
    Turn_Angle_PD(now_right_Island_angle);                    //准备Vz转速，向赛道外转向
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Island_step=Car_Turn_Out_Island;
    return;
  }
  return;
 }
}
// /**
// * @brief 左十字多搬策略
// * @param Cross_step为十字步数选择
// * @param 该函数是进入十字状态后才使用才使用
// * @return 无
// */
int Crossing_mode=0;
uint8 Cross_allow_flag = READY;
uint8 arrive_crossing_center_flag=NOT_FINISH;
uint8 allow_crossing_flag = OPEN;//用于十字
float turn_crossing_angle;//转向十字中心的角度
float crossing_card_center_x,crossing_card_center_y;//记录下卡片的art1转入坐标
float crossing_correct_again_x,crossing_correct_again_y;//记录下art4校正的坐标
uint8 crossing_card_type=0;                             //记录的十字卡片类型
uint8 crossing_card_classify_count=0;                   //十字卡片的数量
float turn_outside_crossing_angle=0;                    //测试使用，记得要删！！！！！！！！！，初始为0°
int left_lie_island_upline_position;                    //最左列行坐标
float record_crossing_zone_x,record_crossing_zone_y;    //记录完成十字卡片的区域坐标
int crossing_target_upline = 100;                             //十字巡上边线的目标行
uint8 record_cross_abc_flag=NOT_FINISH;                 //记录十字外字母的标志位
uint8 Crossing_Zone_count=0;                            //十字的区域记数
float test_cross_top_error;                             //十字内的上边线归一化误差
float record_catch_angle,last_record_catch_angle;
float delta_record_angle;
int catch_count = 0;                                        //捕获次数
int found = 0;                                               //匹配成功标志位
int catch_crossing_zone = NOT_READY;
uint8 Left_Crossing_Finish = NOT_FINISH;


void Left_Crossing_pick_and_move(int *Cross_step)
{
  /*****************************进入十字***************************/
  if (*Cross_step == Car_Crossing_Enter)
  {
    if (arrive_crossing_center_flag == READY) // 十字准备左转弯
    {
      system_delay_ms(600);
      car_stop();
      // system_delay_ms(500);
      arrive_crossing_center_flag=NOT_READY;
      banmaxian_allow_flag = NOT_READY;
      turn_crossing_angle = Angle_Z + 90;  
      if(turn_crossing_angle>=360) 
      turn_crossing_angle=turn_crossing_angle-360;
	    if(turn_crossing_angle<=-360) 
      turn_crossing_angle=turn_crossing_angle+360;
      *Cross_step=Car_Turn_Inside;
      return;
    }
    else
    {
      banmaxian_allow_flag = NOT_READY;       //关闭斑马线
      Island_allow_flag = NOT_READY;          //关闭环岛
      car_run();
      if (Cross_State == 6)
      {
        arrive_crossing_center_flag = READY;
      }
      *Cross_step = Car_Crossing_Enter; // 保持状态
      return;
    }
    return;
  }
  /*****************************向十字中心转向***************************/
  else if (*Cross_step == Car_Turn_Inside)
  {
    if (fabsf(Angle_Z - turn_crossing_angle) <= 3)
    {
      car_stop();
      system_delay_ms(500);
      //  correct_art2_flag=OPEN;                    //打开art4识别类型
      allow_crossing_flag = OPEN;
      /********打开用来十字修正的里程计(必须)*******/
      Crossing_classify_flag = OPEN; // 打开环岛里程计模式
      Angle_Crossing = 0;            // 第一次用来对环岛中心卡片做定位
      Crossing_x = 0;
      Crossing_y = 0;
      delta_crossing_x = 0;
      delta_crossing_y = 0;                   // 为下一步art1定位做准备
      CSI_crossing_correct_flag = NOT_FINISH; // 总钻风调整完毕标志清除
      correct_crossing_card_step = 1;         // art1调整步数
      /*********到时候有个总钻风识别坐标，可以动态调整，目前先用art4调坐标*********/
      crossing_card_center_x = 0;
      crossing_card_center_y = 0;
      //  now_distance_x=0;
      //  now_distance_y=0;
      *Cross_step = Car_Find_Art1; // 转变art1模式
      return;
    }
    else
    {
      Vy = 0;
      Vx = 0;
      Turn_Angle_PD(turn_crossing_angle);          // 准备Vz转速，向十字中心转向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
      *Cross_step = Car_Turn_Inside;
      return;
    }
    return;
  }
  /*****************************对中心卡片用art1粗对正***************************/
  else if (*Cross_step == Car_Find_Art1)
  {
    if (CSI_crossing_correct_flag == FINISH) // art1对准完毕
    {
     NVIC_SetPriority(LPUART1_IRQn, 2);	//降低art1的中断优先级,为下一步art4再矫正做准备
     car_stop();
     system_delay_ms(500);
     /**************清零变量与下一步的变量准备*************/
     crossing_card_center_x=0;        //用完就清
     crossing_card_center_y=0;
     CSI_crossing_correct_flag = NOT_FINISH;       //重置为未完成
     Crossing_classify_flag=CLOSE;                 //关闭环岛里程计模式
     Angle_Crossing = 0;                           //第二次用来对环岛中心卡片做定位
     Crossing_x=0;
     Crossing_y=0;                                 
     delta_crossing_x=0;                           
     delta_crossing_y=0;
     /****************************************/
     now_distance_x=0;
     now_distance_y=0;
     near_card_x=0;                 //准备用才清
     near_card_y=0;
     crossing_correct_again_x=0;
     crossing_correct_again_y=0;
     /**************为art4对准做准备*************/
     arrive_card_flag = OPEN;       //开启修正用的里程计
     Angle_arrive_card = 0;
     correct_x=0;
     correct_y=0;                   //清零修正里程
     delta_x=0;
     delta_y=0;                     //art修正用的变量
     CSI_correct_flag = NOT_FINISH; //art4完成修正的标志位
     /**************清零/重启标志位***************/
     allow_crossing_flag=OPEN;                             //重新打开allow_crossing_flag
     correct_art2_flag=OPEN;                        //打开art4修正中断
     crossing_card_classify_count=0;                //为下一步分类做准备
    //  Find_Crossing_Card_Position=NOT_READY;         //清空记录标志位
    /****************************************/
    *Cross_step=Car_Find_Art4;                     //十字的卡片用art再次对正
    return;
   }
   else
   {
    if(now_distance_x!=0 && now_distance_y> 200 && now_distance_y <700)      //目前先使用art1做矫正
    {
      if(allow_crossing_flag==OPEN)
      {
        if(now_distance_x>=0)
        {
          crossing_card_center_x=(float)now_distance_x;
        }
        else
        {
          crossing_card_center_x=-(float)now_distance_x;
        }           
			crossing_card_center_y=(float)now_distance_y;                          //记录下环岛的卡片坐标，做矫正使用
      // correct_art2_flag=CLOSE;                                            //关闭atr4的中断
      allow_crossing_flag=CLOSE;
      }
    }
      CSI_correct_crossing_correct(crossing_card_center_x, crossing_card_center_y);//art1坐标对正，准备x,y速度,只做一个初步校正
      Turn_Angle_PD(Angle_Z);                                                      //准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);                                 //麦轮控制，为target_speed赋值
    *Cross_step=Island_Card_Correct;//保持模式
    return;
   }
    return;
  }
/*****************************对中心卡片用art4准确对正***************************/
  else if(*Cross_step==Car_Find_Art4)   
{
  if(CSI_correct_flag == FINISH)                  //art4对准完成
  {
     allow_crossing_flag=OPEN;
     car_stop();                    //清零速度，防止乱动
     /************清零使用过的变量**********/
     arrive_card_flag = CLOSE;      //关闭修正用的里程计
     correct_x=0;
     correct_y=0;                   //清零修正里程
     delta_x=0;
     delta_y=0;                     //art修正用的变量
     crossing_correct_again_x=0;
     near_card_x=0;
     crossing_correct_again_y=0;    //再次微调的art4输入距离
     near_card_y=0;
     CSI_correct_flag = NOT_FINISH; //清零矫正完成的标志位
     /************************************/
     crossing_card_type=0;
     card_type=0;                   //这个变量虽然没有用到但其实已经赋值，为下一步art4做准备
     /*************************************/
     *Cross_step=Crossing_Card_Classify_Pick;//转变模式
     return;
  }
  else
  {
    *Cross_step=Car_Find_Art4;       //保持模式
    if(near_card_x != 0 && near_card_y > 0 && near_card_y <= 400)    //有正确的卡片坐标传入
    {
      if(allow_crossing_flag==OPEN)                        //只执行一次
      {
        crossing_correct_again_x=(float)near_card_x+20;
        crossing_correct_again_y=(float)near_card_y+20;     //再次微调的art4输入距离
        correct_art2_flag=CLOSE;                            //关闭art4修正中断
        allow_crossing_flag=CLOSE;
      }
    }
      CSI_dis_new_correct(crossing_correct_again_x, crossing_correct_again_y);//art4坐标对正，准备x,y速度
      Turn_Angle_PD(Angle_Z);           //准备Vz转速，作用是锁住车头方向
      Car_Inverse_kinematics_solution(Vx, Vy, Vz);  //麦轮控制，为target_speed赋值
      return;
  }
     return;
}
/*********************对十字中心卡片分类******************/
else if(*Cross_step==Crossing_Card_Classify_Pick)//十字卡片的分类
{
   if(crossing_card_type!=0)              //卡片类型已记录
   {
     if(crossing_card_classify_count<5)
     {
      cross_card[crossing_card_classify_count].Card_Type=crossing_card_type;    //记录下该张卡片类型
      cross_card[crossing_card_classify_count].Card_PWM_Duty=crossing_card_classify_count;      //记录该张卡片在舵机转动中的类别
      classify_little_360(crossing_card_classify_count);
      arm_control(2);//捡卡片 
	    arm_control(4);//默认模式                               //卡片拾取完成
      crossing_card_type=0;                                     //清零，等待下一次的识别传入
      crossing_card_classify_count++;                                  //卡片数量加一
      *Cross_step=Crossing_Card_Classify_Pick;                 //继续进去环岛的卡片分类和拾取状态
      return;
     }
     else
     {
        car_stop();
        correct_art2_flag = CLOSE;                   // 关闭atr4中断
        turn_outside_crossing_angle = Angle_Z - 180; // 为下一个角度转向做准备
        if (turn_outside_crossing_angle >= 360)
          turn_outside_crossing_angle = turn_outside_crossing_angle - 360;
        if (turn_outside_crossing_angle <= -360)
          turn_outside_crossing_angle = turn_outside_crossing_angle + 360;
        *Cross_step = Car_Crossing_Turn_Outside; // 模式转变
        return;
     }
   }
   else
   {
     correct_art2_flag=OPEN;            //art4中断常开
     if(card_type!=0)
     {
       crossing_card_type=card_type;      //记录这张卡片的类型
       card_type=0;                     //处理完后及时清零
       correct_art2_flag=CLOSE;         //关闭
     }
     *Cross_step=Crossing_Card_Classify_Pick;   //环岛的卡片分类
     return;
   }
    return;
}
/*********************向十字外区域转向*****************/
else if(*Cross_step==Car_Crossing_Turn_Outside)
{
    if(fabsf(turn_outside_crossing_angle-Angle_Z)<=4)
  {
		car_stop();
		system_delay_ms(500);
    left_lie_island_upline_position=0;                    //使用前先清零
    *Cross_step=Car_Go_Crossing_Outside;                  //转变模式
    return;
  }
  else
  {
    Turn_Angle_PD(turn_outside_crossing_angle);           //准备Vz转速，向环岛外转向
    Vx=0;
    Vy=0;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Cross_step=Car_Crossing_Turn_Outside;               //保持模式
    return;
  }
  return;
}
else if(*Cross_step==Car_Go_Crossing_Outside)
{
if(left_lie_island_upline_position>110 && left_lie_island_upline_position<118)               //最右列行坐标下降到70行
  {
    car_stop();
    system_delay_ms(500);
    left_lie_island_upline_position=0;                 //清零变量
    now_distance_x=0;
    now_distance_y=0;                                   //下一步要用的坐标清零
    Angle_Crossing_Panduan = 0;                         //用于判断是否为重复检测
    NVIC_SetPriority(LPUART1_IRQn, 3);	                //降低art1的中断优先级,为下一步art4再矫正做准备
    *Cross_step=Car_Go_Crossing_Upline;                 //切换模式
    return;
  }
  else
  {
    left_lie_island_upline_position=Top_Top_Line_Search_Crossing(115,5,0);//持续从第100行往上扫上边线
    Vy=5;
    Vx=0;
    Turn_Angle_PD(Angle_Z);                             //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Cross_step=Car_Go_Crossing_Outside;
       return;
  }
     return;
}
/**************************十字内巡上边线**************************/
else if(*Cross_step==Car_Go_Crossing_Upline)
{
    if(catch_crossing_zone == READY )//设定一个识别区间(先暂时这样，后续可能会使用到世界坐标)，该判断条件更优先x区间限制只扫右平面 y区间屏蔽放下的卡片
  {
    /**************再次打开环岛专用里程计(为下一步区域校正做准备)*************/
    record_catch_angle = Angle_Crossing_Panduan;    //记录下本次识别到的角度值
    delta_record_angle = record_catch_angle - last_record_catch_angle;//对比上一捕获到卡片时的角度
    last_record_catch_angle = record_catch_angle;
    if(fabsf(delta_record_angle)>20 || catch_count==0)       //首次分类后角度变化
    {
      car_stop();
      system_delay_ms(1000);
     catch_count++;
     Crossing_classify_flag=OPEN;                   //打开环岛里程计模式
     Angle_Crossing = 0;                            //第二次用来对环岛分类区域卡片做定位
     Crossing_x=0;
     Crossing_y=0;
     crossing_class_step=1;                         //校准步数初始化为1                                 
     delta_crossing_class_x=0;                           
     delta_crossing_class_y=0;                      //为下一步art1做分类区域定位做准备
     crossing_classify_correct_finish=NOT_FINISH;          //清除调整完毕的标志位，防止直接跳状态
     /***********************************************************************/
     record_crossing_zone_x=(float)now_distance_x;
     record_crossing_zone_y=(float)now_distance_y;        //记录当前捕捉到的区域坐标，为下一步对准做准
     record_cross_abc_flag = NOT_FINISH;
     record_abc_card_type=0;
     card_abc=0;
     near_card_x=0;
     near_card_y=0;
     now_distance_x = 0;
     now_distance_y = 0;
    *Cross_step=Car_Go_Crossing_Zone_Outside;             //去环岛的分类区域
       return;
    }
    else if(fabsf(delta_record_angle)<=20 && catch_count!=0)                                                 //角度变化过小
    {
      now_distance_x=0;
      now_distance_y=0;                                   //清零后再进入循环
      catch_crossing_zone = NOT_READY;                    //未捕获正确的十字区域
      *Cross_step=Car_Go_Crossing_Upline;                 //继续找上边线
         return;
    }
  }
  else
  {
    if(now_distance_x <200  && now_distance_x > -200 && now_distance_y> 200  && now_distance_y<800 )
    {
      catch_crossing_zone = READY;
    }
    else
    {
      catch_crossing_zone = NOT_READY;
    }
    car_run_upline_left(crossing_target_upline);
    *Cross_step=Car_Go_Crossing_Upline;
    return;
  }
     return;
}
/*************************对准十字外分类区域**********************************/
else if(*Cross_step==Car_Go_Crossing_Zone_Outside)
 {
  if(crossing_classify_correct_finish)//对准完成
  {
    car_stop();
    system_delay_ms(500);
    catch_crossing_zone = NOT_READY;                //上一步的变量清零
    //关闭环岛里程计，清空变量//
     Crossing_classify_flag=CLOSE;                  //关闭环岛里程计模式
     Angle_Crossing = 0;                            //第二次用来对环岛分类区域卡片做定位
     Crossing_x=0;
     Crossing_y=0;
    /************************/
    now_distance_x=0;
    now_distance_y=0;
    record_crossing_zone_x=0;
    record_crossing_zone_y=0;
    crossing_classify_correct_finish=NOT_FINISH;
    crossing_class_step=1;                              //校准步数初始化为1  
    /************************/
    found = NOT_READY;                                  //找到标志
    record_cross_abc_flag = NOT_FINISH;                 //清空记录完成的标志位
    record_abc_card_type = 0;                           //清空记录过的标志位
    card_abc=0;                                         //先清零再使用
    classify_art2_flag=OPEN;                            //打开发送字母的art4中断
    correct_art2_flag=CLOSE;                            //关闭art4修正中断
    *Cross_step=Car_Crossing_Zone_Classify;             //识别环岛的区域类型
       return;
  }
  else
  {
    find_crossing_classify(record_crossing_zone_x,record_crossing_zone_y);//环岛区域对正
    *Cross_step=Car_Go_Crossing_Zone_Outside;
       return;
  }
     return;
 }
 //***********************十字内区域识别***********************************/
 else if(*Cross_step==Car_Crossing_Zone_Classify)
 {
  if(record_cross_abc_flag == FINISH)                            //识别与记录完成
  {
    Crossing_Zone_count++;
    allow_crossing_flag = OPEN;                                 //重置allow_crossing_flag
    record_cross_abc_flag = NOT_FINISH;
    record_abc_card_type=0;
    card_abc=0;
    near_card_x=0;
    near_card_y=0;
    left_lie_island_upline_position=0;
    test_cross_top_error=0;
    back_flag = NOT_FINISH;
    *Cross_step=Step_Back_Crossing_Center;                     //后退一点
    return;
  }
  else 
  {
    *Cross_step=Car_Crossing_Zone_Classify;                  //识别环岛的区域类型
    classify_art2_flag=OPEN;                                //打开发送字母的art4中断
    NVIC_SetPriority(LPUART1_IRQn, 3);                      //降低art1中断优先级
    if(card_abc!=0 && near_card_x!=666 && near_card_y!=666)
    {
      classify_art2_flag=CLOSE;                            //打开发送字母的art4中断
      switch_abc_to_righttype(card_abc);                    //转化card_abc
    //   if(record_abc_card_type==cross_card[0].Card_Type)
		// {
    //  arm_control(5);
		//  classify_little_360(cross_card[0].Card_PWM_Duty);//转到对应的舵机角度
    //  system_delay_ms(1000);
    //  arm_control(3);                                    //捡出卡片
    //  arm_control(4);                                    //归位
    //  record_cross_abc_flag = FINISH;
    //  return;
		//  }
    //   else if(record_abc_card_type==cross_card[1].Card_Type)
		// {
    //  arm_control(5);
		//   classify_little_360(cross_card[1].Card_PWM_Duty);//转到对应的舵机角度
    //  system_delay_ms(1000);
    //  arm_control(3);                                    //捡出卡片
    //  arm_control(4);                                    //归位
    //  record_cross_abc_flag = FINISH;
    //  return;
		//  }
    //   else if(record_abc_card_type==cross_card[2].Card_Type)
		// {
    //  arm_control(5);
		//   classify_little_360(cross_card[2].Card_PWM_Duty);//转到对应的舵机角度
    //  system_delay_ms(1000);
    //  arm_control(3);                                    //捡出卡片
    //  arm_control(4);                                    //归位
    //  record_cross_abc_flag = FINISH;
    //  return;
		//  }
    //   else if(record_abc_card_type==cross_card[3].Card_Type)
		// {
    //  arm_control(5);
		//   classify_little_360(cross_card[3].Card_PWM_Duty);//转到对应的舵机角度
    //  system_delay_ms(1000);
    //  arm_control(3);                                    //捡出卡片
    //  arm_control(4);                                    //归位
    //  record_cross_abc_flag = FINISH;
    //  return;
		//  }
    //   else if(record_abc_card_type==cross_card[4].Card_Type)
		// {
    //  arm_control(5);
		//  classify_little_360(cross_card[4].Card_PWM_Duty);//转到对应的舵机角度
    //  system_delay_ms(1000);
    //  arm_control(3);                                    //捡出卡片
    //  arm_control(4);                                    //归位
    //  record_cross_abc_flag = FINISH;
    //  return;
		//  }
    for (int i = 0; i < 5; i++) 
    {  
      if (record_abc_card_type == cross_card[i].Card_Type) 
      {  
        found = READY;  
        arm_control(5);  
        classify_little_360(cross_card[i].Card_PWM_Duty); // 转到对应的舵机角度  
        system_delay_ms(1000);  
        arm_control(3); // 捡出卡片  
        arm_control(4); // 归位  
        record_cross_abc_flag = FINISH;  
        break; // 找到匹配项后退出循环  
     }  
    }  
  if (!found) 
  {  
    // 如果没有找到匹配的Card_Type  
    record_cross_abc_flag = NOT_FINISH;
    Crossing_Zone_count++;  
    *Cross_step = Step_Back_Crossing_Center;
    return;  
  }   
    }
     return;
  }
     return;
  }
//*******************************后退至可寻迹行********************//
else if(*Cross_step==Step_Back_Crossing_Center)
 {
  if(back_flag==FINISH)         //最右侧行坐标上升至100行
  {
     if(Crossing_Zone_count==5)                                                            //已经完成5张的分类
  {
    car_stop();
    now_distance_x=0;
    now_distance_y=0;
    turn_out_crossing_angle = Angle_Z + 90;                                               //转向出环
    if(turn_out_crossing_angle>=360) 
        turn_out_crossing_angle=turn_out_crossing_angle-360;
	     if(turn_out_crossing_angle<=-360) 
        turn_out_crossing_angle=turn_out_crossing_angle+360;
    *Cross_step=Car_Turn_Out_Crossing;                                                    //模式转变，准备出环
       return;
  }
    car_stop();                                                                          //清空速度
    back_flag=NOT_FINISH;
    left_lie_island_upline_position=0;
    now_distance_x=0;
    now_distance_y=0;
    test_cross_top_error=0;
    *Cross_step=Car_Go_Crossing_Upline;                                       //环岛内巡上边线
       return;
  }
  else
  {
    found = NOT_FINISH;                                 //重置为未找到卡片
    allow_crossing_flag = OPEN;                         //重置allow_crossing_flag
    Vy=-2;
    Vx=0;
    Vz=0;
    Turn_Angle_PD(Angle_Z);                             //准备Vz转速，锁住车头
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    Top_Line_Search();                 // 对上边线扫线，为上边线数组做准备
    test_cross_top_error = Top_Line_Err_Left(crossing_target_upline)/25; // 对上边线扫线输出一个误差,作归一化处理
    if(test_cross_top_error < 0 && test_cross_top_error >-0.7)
    {
      back_flag=FINISH;
    }
    *Cross_step=Step_Back_Crossing_Center;
       return;
  }
     return;
 }
 else if(*Cross_step==Car_Turn_Out_Crossing)
 {
    if(fabsf(turn_out_crossing_angle-Angle_Z)<=4)
  {
		car_stop();
		system_delay_ms(500);
    now_distance_x=0;
    now_distance_y=0;
    banmaxian_allow_flag = READY;                    //重新打开斑马线
    Island_allow_flag = READY;                       //重新打开环岛
    Left_Crossing_Finish = FINISH;                   //左十字完成标志
    left_lie_island_upline_position=0;               //使用前先清零
    *Cross_step=Car_Crossing_Enter;               //转到第一步
    stop_detect_flag=1;
    Cross_State=0;
    Cross_Handle_Flag=0;                          //清零进入条件
    return;
  }
  else
  {
    Turn_Angle_PD(turn_out_crossing_angle);           //准备Vz转速，向环岛外转向
    Vx=0;
    Vy=0;
    Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
    *Cross_step=Car_Turn_Out_Crossing;               //保持模式
       return;
  }
     return;
 }
}