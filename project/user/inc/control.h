#ifndef __CONTROL_H
#define __CONTROL_H

#include "stdio.h"
#include "stdint.h"
#include "zf_common_headfile.h"

#define DIR_LF D14//��ǰgpio
#define DIR_LB D3//���gpio
#define DIR_RF D12//��ǰgpio
#define DIR_RB D1//�Һ�gpio

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

#define AMPLITUDE_MOTOR 3000 //pwm�޷�

//??pid??
typedef struct{
	float now_speed;	  //ʵ���ٶ�
	float target_speed;	//Ŀ���ٶ�
	int target_pwm;		//Ŀ��Pwm
	uint8 xuhao;		//���������?
	float kp ;		        
	float ki ;		        
	float kd ;	          
	float error;          //��ǰ���?
	float lastError;	    //�ϴ����?
	float lastlastError;  //���ϴ����?
	float dError;         //����������ϴ����Ĳ��?
	float output;         //����?
	float output_last;    //�ϴ�����?
}pid_info;


extern float Car_H;//����
extern float Car_W;//����
extern int encoder[4];//����������
extern float encoder_sum[4];
extern float target_encoder_sum[4];
extern float loc_target[4];
extern int Turn_Left_flag,Turn_Right_flag;
extern int loc_Finish_flag;
extern float loc_err;//�������
extern float abs_loc_err;//����������ֵ
extern int pid_motor[4];//pid����������pwmֵ
extern float turn_angle;//ת���?
extern int spin;//��ת����
extern int translation;
extern pid_info LF_motor_pid;//�ĸ�����pid�ṹ��
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
