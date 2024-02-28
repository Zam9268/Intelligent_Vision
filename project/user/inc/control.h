//#ifndef __CONTROL_H
//#define __CONTROL_H

#include "stdio.h"
#include "stdint.h"
#include "zf_common_headfile.h"

#define DIR_LF D14//定义gpio口
#define DIR_LB D3
#define DIR_RF D12
#define DIR_RB D1

#define motor_LF PWM1_MODULE1_CHB_D15//定义pwm通道
#define motor_LB PWM2_MODULE3_CHA_D2
#define motor_RF PWM1_MODULE0_CHB_D13
#define motor_RB PWM1_MODULE3_CHA_D0

#define ENCODER_LF                       (QTIMER1_ENCODER2)//定义编码器通道
#define ENCODER_LF_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_LF_DIR                   (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_RF                       (QTIMER2_ENCODER1)
#define ENCODER_RF_LSB                   (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_RF_DIR                   (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_LB                       (QTIMER1_ENCODER1)
#define ENCODER_LB_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_LB_DIR                   (QTIMER1_ENCODER1_CH2_C1)


#define ENCODER_RB                       (QTIMER2_ENCODER2)
#define ENCODER_RB_LSB                   (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_RB_DIR                   (QTIMER2_ENCODER2_CH2_C25)

//定义pid结构体
typedef struct{
	float kp ;		        //比例系数
	float ki ;		        //积分系数
	float kd ;	          //微分系数
	float error;          //误差值
	float lastError;	    //上一个误差值
	float dError;         //上次误差和这次误差的变化值
	float output;         //输出值
	float output_last;    //上次的输出值
}pid_info;


extern float Velocity_KP;//增量式Kp
extern float Velocity_KI;//增量式Ki
extern int encoder[4];//存放编码器数值
extern float target_motor[4];//存放四个麦轮计算出来的速度
extern float pid_motor[4];//存放增量式pid闭环出的速度
extern float turn_angle;//过弯转向角度
extern int spin;//过弯时的平均中线
extern int translation;
extern pid_info LF_motor_pid;//电机pid
extern pid_info RF_motor_pid;
extern pid_info LB_motor_pid;
extern pid_info RB_motor_pid;



void Motor_Init(void);
void Encoder_Init(void);
void Read_Encoder(void);
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz);
void Move_Transfrom(double target_Vx, double target_Vy, double target_Vz);
void Incremental_PI(void);
void PidInit(pid_info * pid);
float increment_pid(float error,pid_info *pid);
void PID_cale();
void motor_close_control(void);
void motor_control(void);
void Speed_Control(float Vx_Speed, float Vy_Speed, float Vz_Speed);