#ifndef __CONTROL_H
#define __CONTROL_H

#include "stdio.h"
#include "stdint.h"
#include "zf_common_headfile.h"

#define DIR_LF D14//????gpio??
#define DIR_LB D3//????gpio??
#define DIR_RF D12//????gpio??
#define DIR_RB D1//????gpio??

#define motor_LF PWM1_MODULE1_CHB_D15//????pwm??
#define motor_LB PWM2_MODULE3_CHA_D2//????pwm??
#define motor_RF PWM1_MODULE0_CHB_D13//????pwm??
#define motor_RB PWM1_MODULE3_CHA_D0//????pwm??

#define ENCODER_LF                       (QTIMER1_ENCODER2)//???????
#define ENCODER_LF_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_LF_DIR                   (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_LB                       (QTIMER2_ENCODER1)//???????
#define ENCODER_LB_LSB                   (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_LB_DIR                   (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_RF                       (QTIMER1_ENCODER1)//???????
#define ENCODER_RF_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_RF_DIR                   (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_RB                       (QTIMER2_ENCODER2)//???????
#define ENCODER_RB_LSB                   (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_RB_DIR                   (QTIMER2_ENCODER2_CH2_C25)

#define AMPLITUDE_MOTOR 3000 //pwm???

//??pid??
typedef struct{
	float now_speed;	  //当前速度
	float target_speed;	//目标速度
	int target_pwm;		//目标Pwm
	uint8 xuhao;		//编码器序号
	float kp ;		        
	float ki ;		        
	float kd ;	          
	float error;          //当前误差
	float lastError;	    //上次误差
	float lastlastError;  //上上次误差
	float dError;         //本次误差与上次误差的差值
	float output;         //输出值
	float output_last;    //上次输出值
}pid_info;


extern float Velocity_KP;//增量式kp
extern float Velocity_KI;//增量式Ki
extern float Car_H;//车长
extern float Car_W;//车宽
extern int encoder[4];//编码器数据
extern float target_motor[4];//目标电机pwm
extern int pid_motor[4];//pid处理后的速度
extern float turn_angle;//转向角
extern int spin;//旋转方向
extern int translation;
extern pid_info LF_motor_pid;//四个轮子pid结构体
extern pid_info RF_motor_pid;
extern pid_info LB_motor_pid;
extern pid_info RB_motor_pid;
extern float PID_Bias[4], PID_Last_bias[4];


void Motor_Init(void);
void Encoder_Init(void);
void Read_Encoder(void);
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz);
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz);
void Incremental_PI(void);
void PidInit(void);
void increment_pid(void);
void PID_cale();
void motor_close_control(void);
void motor_control(void);
void Speed_Control(float Vx_Speed, float Vy_Speed, float Vz_Speed);
float PIDInfo_Limit(float Value, float MaxValue);

#endif
