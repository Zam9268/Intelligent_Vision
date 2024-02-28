//#ifndef __CONTROL_H
//#define __CONTROL_H

#include "stdio.h"
#include "stdint.h"
#include "zf_common_headfile.h"

#define DIR_LF D14//����gpio��
#define DIR_LB D3
#define DIR_RF D12
#define DIR_RB D1

#define motor_LF PWM1_MODULE1_CHB_D15//����pwmͨ��
#define motor_LB PWM2_MODULE3_CHA_D2
#define motor_RF PWM1_MODULE0_CHB_D13
#define motor_RB PWM1_MODULE3_CHA_D0

#define ENCODER_LF                       (QTIMER1_ENCODER2)//���������ͨ��
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

//����pid�ṹ��
typedef struct{
	float kp ;		        //����ϵ��
	float ki ;		        //����ϵ��
	float kd ;	          //΢��ϵ��
	float error;          //���ֵ
	float lastError;	    //��һ�����ֵ
	float dError;         //�ϴ�����������ı仯ֵ
	float output;         //���ֵ
	float output_last;    //�ϴε����ֵ
}pid_info;


extern float Velocity_KP;//����ʽKp
extern float Velocity_KI;//����ʽKi
extern int encoder[4];//��ű�������ֵ
extern float target_motor[4];//����ĸ����ּ���������ٶ�
extern float pid_motor[4];//�������ʽpid�ջ������ٶ�
extern float turn_angle;//����ת��Ƕ�
extern int spin;//����ʱ��ƽ������
extern int translation;
extern pid_info LF_motor_pid;//���pid
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