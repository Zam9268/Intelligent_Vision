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

//??pid??
typedef struct{
	float now_speed;	  //实际速度
	float target_speed;	//目标速度
	int target_pwm;		//目标Pwm
	uint8 xuhao;		//编码器序�?
	float kp ;		        
	float ki ;		        
	float kd ;	          
	float error;          //当前误差
	float lastError;	    //上次误差
	float lastlastError;  //上上次误�?
	float dError;         //本次误差与上次误差的偏差�?
	float output;         //输出�?
	float output_last;    //上次输出�?
}pid_info;


extern float Car_H;//车长
extern float Car_W;//车宽
extern int encoder[4];//四个编码器读�?
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

extern int pid_motor[4];

extern pid_info Pos_turn_pid[4];//


void Motor_Init(void);
void Encoder_Init(void);
void Read_Encoder(void);
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz);
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz);
void PidInit(void);
void Pos_PidInit(void);
void increment_pid(void);
float Location_pid(pid_info *pid, float Encoder, float Target);
void clear_encoder_sum(void);
void Set_Distence_m(float distance);
void Drive_Motor();
void turnloc_pid(void);
void motor_close_control(void);
void motor_control(void);
void Turn_Angle_PD(float Tar_angle_Z);
void Encoder_odometer(void);
float PIDInfo_Limit(float Value, float MaxValue);

#endif
