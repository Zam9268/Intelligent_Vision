#ifndef __CONTROL_H
#define __CONTROL_H

#include "stdio.h"
#include "stdint.h"
#include "zf_common_headfile.h"

#define DIR_LF D14//电机gpio
#define DIR_LB D3//电机gpio
#define DIR_RF D12//电机gpio
#define DIR_RB D1//电机gpio

#define motor_LF PWM1_MODULE1_CHB_D15//左前pwm通道
#define motor_LB PWM2_MODULE3_CHA_D2//左后pwm通道
#define motor_RF PWM1_MODULE0_CHB_D13//右前pwm通道
#define motor_RB PWM1_MODULE3_CHA_D0//右后pwm通道

#define ENCODER_LF                       (QTIMER1_ENCODER2)//左前编码器通道
#define ENCODER_LF_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_LF_DIR                   (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_LB                       (QTIMER2_ENCODER1)//左后编码器通道
#define ENCODER_LB_LSB                   (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_LB_DIR                   (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_RF                       (QTIMER1_ENCODER1)//右前编码器通道
#define ENCODER_RF_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_RF_DIR                   (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_RB                       (QTIMER2_ENCODER2)//右后编码器通道
#define ENCODER_RB_LSB                   (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_RB_DIR                   (QTIMER2_ENCODER2_CH2_C25)

#define AMPLITUDE_MOTOR 3000 //pwm???
#define CONTROL_FREQUENCY  100//编码器读取周期(0.01s 10ms)
#define Turn_limiting  20//转向速度输出限幅
#define Car_go           	0 //寻迹
#define Car_find_card_y     1 //寻迹，用在第一种方式
#define Car_arrive_card  	1 //已到达卡片世界坐标附近
#define Car_turn         	2 //转向
#define Car_find_card_cor  	3 //总钻风微调标
#define Pick_up_card  		4 //机械臂拾取卡片
#define Car_turn_again  	5 //回正
#define Distance_output 10  //速度环输出限幅
#define CSI_CORRECT_DONE 1  //总钻风完成校正标志

//??pid??
typedef struct{
	float now_speed;	  //实际速度
	float target_speed;	//目标速度
	int target_pwm;		//目标Pwm
	uint8 xuhao;		//编码器序号
	float kp ;		        
	float ki ;		        
	float kd ;	          
	float error;          //当前误差
	float lastError;	    //上次误差
	float lastlastError;  //上上次误差
	float dError;         //本次误差与上次误差的偏差值
	float output;         //输出值
	float output_last;    //上次输出值
}pid_info;


extern float Car_H;//车长
extern float Car_W;//车宽
extern float Vx,Vy,Vz;
extern float ahead_speed;//直行速度
extern float correct_x_speed;//x轴上的修正速度
extern float correct_z_speed;//z轴上的修正速度
extern float correct_move_speed ;//x轴修正速度
extern float correct_turn_speed;//x轴修正速度
extern int encoder[4];//四个编码器读数
extern int encoder_test[4];
extern float encoder_sum[4];
extern float target_encoder_sum[4];
extern float loc_target[4];
extern float loc_last_target[4];
extern int Turn_Left_flag,Turn_Right_flag;
extern int loc_Finish_flag;
extern int Location_pid_flag;
extern float loc_err;
extern float abs_loc_err;
extern float bili_act_turn;
extern float loc_kp;
extern float loc_kd;
extern float Vx_1, Vx_2, Vy_1, Vy_2;//对里程的cos，sin分解
extern float Vx_world, Vy_world;//世界坐标上的x，y
extern float Car_dis_x, Car_dis_y;//x轴，y轴行走距离
extern int car_world_distance;//车辆在全局坐标上与原点的距离
extern float car_world_angle;//卡片世界坐标解算出的世界方位角
extern float Car_dis_x2, Car_dis_y2;
extern float Card_dis_car_x,Card_dis_car_y;
extern double delta_card_y, delta_card_x;
extern double delta_angle;
extern float Turn_Bias;
extern float dis_kp;//距离环kp
extern float dis_kd;//距离环kd
extern float dis_error;
extern float dis_change[4];//存放距离环输出结果
extern int CSI_correct_flag;
extern int card_y[10];
extern int card_x[10];
extern int Put_flag;
extern int target_type;
extern int car_mode;
extern float turn_angle;
extern int catch_card_flag;
extern int delta_x,delta_y; //总钻风识别的卡片中心坐标
extern int correct_x_flag,correct_y_flag;
extern int correct_step;

extern int pid_motor[4];

extern pid_info Pos_turn_pid[4];

extern pid_info Angle_turn_pid;

extern pid_info distance_pid[4];



void Motor_Init(void);
void Encoder_Init(void);
void Read_Encoder(void);
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz);
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz);
void car_run(void);
void PidInit(void);
void Pos_PidInit(void);
void Distance_PidInit(void);
void increment_pid(void);
float Location_pid(pid_info *pid, float Encoder, float Target);
void clear_encoder_sum(void);
void Set_Distence_m(float distance);
void Drive_Motor();
void turnloc_pid(void);
void motor_close_control(void);
void Turn_Angle_PD(float Tar_angle_Z);
void Encoder_odometer(void);
float PIDInfo_Limit(float Value, float MaxValue);
float Distance_pid(pid_info *pid, int target_diantance, int actual_distance);
void CSI_dis_new_correct(int cor_x, int cor_y);
void car_findcard(int *mode);
void car_findcard_new(int *mode);
#endif
