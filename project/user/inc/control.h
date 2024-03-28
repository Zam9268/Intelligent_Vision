#ifndef __CONTROL_H
#define __CONTROL_H

#include "stdio.h"
#include "stdint.h"
#include "zf_common_headfile.h"

#define DIR_LF D14//×óÇ°gpio
#define DIR_LB D3//×óºógpio
#define DIR_RF D12//ÓÒÇ°gpio
#define DIR_RB D1//ÓÒºógpio

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

#define AMPLITUDE_MOTOR 3000 //pwmÏÞ·ù

//??pid??
typedef struct{
	float now_speed;	  //Êµ¼ÊËÙ¶È
	float target_speed;	//Ä¿±êËÙ¶È
	int target_pwm;		//Ä¿ï¿½ï¿½Pwm
	uint8 xuhao;		//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
	float kp ;		        
	float ki ;		        
	float kd ;	          
	float error;          //ï¿½ï¿½Ç°ï¿½ï¿½ï¿?
	float lastError;	    //ï¿½Ï´ï¿½ï¿½ï¿½ï¿?
	float lastlastError;  //ï¿½ï¿½ï¿½Ï´ï¿½ï¿½ï¿½ï¿?
	float dError;         //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï´ï¿½ï¿½ï¿½ï¿½Ä²ï¿½Ö?
	float output;         //ï¿½ï¿½ï¿½Ö?
	float output_last;    //ï¿½Ï´ï¿½ï¿½ï¿½ï¿½Ö?
}pid_info;


extern float Car_H;//ï¿½ï¿½ï¿½ï¿½
extern float Car_W;//ï¿½ï¿½ï¿½ï¿½
extern int encoder[4];//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
extern float encoder_sum[4];
extern float target_encoder_sum[4];
extern float loc_target[4];
extern int Turn_Left_flag,Turn_Right_flag;
extern int loc_Finish_flag;
extern float loc_err;//ÖÐÏßÎó²î
extern float abs_loc_err;//ÖÐÏßÎó²î¾ø¶ÔÖµ
extern int pid_motor[4];//pid´¦ÀíºóÊä³öµÄpwmÖµ
extern float turn_angle;//×ªï¿½ï¿½ï¿?
extern int spin;//ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½
extern int translation;
extern pid_info LF_motor_pid;//ï¿½Ä¸ï¿½ï¿½ï¿½ï¿½ï¿½pidï¿½á¹¹ï¿½ï¿½
extern pid_info RF_motor_pid;
extern pid_info LB_motor_pid;
extern pid_info RB_motor_pid;
extern pid_info Pos_turn_pid[4];//
extern float PID_Bias[4], PID_Last_bias[4];


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
void Speed_Control(float Vx_Speed, float Vy_Speed, float Vz_Speed);
float PIDInfo_Limit(float Value, float MaxValue);

#endif
