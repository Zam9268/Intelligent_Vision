#ifndef __CONTROL_H
#define __CONTROL_H

#include "stdio.h"
#include "stdint.h"
#include "zf_common_headfile.h"

#define OPEN   1
#define CLOSE  0

#define READY   1
#define NOT_READY  0

#define FINISH   1
#define NOT_FINISH  0

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

#define Find_banmaxian      0//找到斑马线
#define Find_upline         1//上边线寻迹
#define Catch_card          2//向卡片前进识别
#define Watch_card          3//看卡片上的数字类型
#define Putout_card         4//放置卡片
#define Go_back             5//后退至y处原本的位置
#define Turn_back           6//返回正常寻迹

// #define Catch_zeropoint    				0//找到原点处的元素标志
#define Arrive_zeropoint   					0//到达原点处
#define Car_Island_turn    					1//向卡片放置区域转向
#define Island_Card_Correct   				2//环岛卡片矫正
#define Island_Card_Correct_again			3//art4再校正
#define Island_Card_Classify_Pick  			4//环岛卡片分类
#define Car_Island_Turn_Outside				5//向环岛外侧转向
#define Car_Go_Ahead_Outside				6//向环岛外侧直进
#define Car_Go_Find_Upline                  7//对环岛进行绕行
#define Car_Go_Island_Zone					8//去环岛的卡片分类区域
#define Island_Zone_Classify				9//识别环岛的区域的类型
#define Step_Back_Island_Center             10//后退至环岛中心
//////////////////环岛内//////////////////////
#define Car_Turn_Again_And_Again			11//向右转向
#define Car_Go_Island_Right_Zone			12//向环岛的右区域前进
#define Car_Go_Find_Upline_Inside_Island    13//环岛内巡上边线
#define Car_Go_Island_Zone_Inside			14//环岛内向卡片区域
#define Island_Zone_Classify_Inside			15//环岛内卡片区域分类
#define Step_Back_Island_Less				16//环岛内后退
/////////////////出环/////////////////////////
#define Car_Turn_Island_Outside_Again		17//向环岛外转向
#define Car_Go_Island_Outside_Again			18//向环岛外前进
#define Car_Turn_Out_Island					19//出环

//******************十字状态*********************/
#define Car_Crossing_Enter					 0//进入十字
#define Car_Turn_Inside						 1//转向中心
#define Car_Find_Art1						 2//art1粗对准
#define Car_Find_Art4						 3//art4细对准
#define Crossing_Card_Classify_Pick			 4//十字的卡片分类和拾取
#define Car_Crossing_Turn_Outside			 5//转向十字外部区域
#define Car_Go_Crossing_Outside				 6//向左十字右边前进至目标行
#define Car_Go_Crossing_Upline				 7//十字内巡上边线
#define Car_Go_Crossing_Zone_Outside		 8//对准十字区域
#define Car_Crossing_Zone_Classify			 9//十字区域识别分类与放置
#define Step_Back_Crossing_Center			 10//往后退至循迹行
#define Car_Turn_Out_Crossing				 11//出十字


#define A_card   	8
#define B_card   	5
#define C_card   	4
#define D_card   	14
#define E_card   	6
#define F_card   	9
#define G_card   	10
#define H_card   	12
#define I_card		3
#define J_card		15
#define K_card		11
#define L_card		7
#define M_card		1
#define N_card		2
#define O_card		13

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
/***环岛卡片和十字卡片***/
typedef struct{
	uint8 Card_Type;    //卡片对应的类别
    int Card_PWM_Duty;//卡片对应的角度
	int Card_Put_Out_Finish;//卡片放置完成的标志位
}card;

extern uint8 find_card_allow;
extern float right_top_error, last_right_top_error; // 与目标行数的加权误差
extern float left_top_error, last_left_top_error; // 与目标行数的加权误差
extern int card_center_x;
extern int card_center_y;
extern int arrive_card_flag;
extern float Vx,Vy,Vz;
extern float ahead_speed;//直行速度
extern float correct_z_speed;//z轴上的修正速度
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
extern float correct_x,correct_y;
extern uint8 card_classify;
extern float Car_dis_x1, Car_dis_y1;
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
extern uint8 Find_card_allow;
extern int card_y[10];
extern int card_x[10];
extern int Put_flag;
extern int target_type;
extern int car_mode;
extern float turn_angle;
extern int catch_card_flag;
extern float delta_x,delta_y; //总钻风识别的卡片中心坐标
extern int correct_x_flag,correct_y_flag;
extern int correct_step;
extern int correct_art2_flag;
extern uint8 find_ramp;
extern float ramp_x,ramp_y;
extern uint8 ramp_step;
extern uint8 ramp_finish;
extern uint8 Traffic;        //交通工具类
extern uint8 Weapon;         //武器类
extern uint8 Supply;		 //物资类
extern int classify_mode;
extern int classify_type;
extern int num_card_x,num_card_y;
extern uint8 numcard_classify;
extern int delta_class_x,delta_class_y;
extern float delta_crossing_x,delta_crossing_y;
extern uint8 Find_num;		//识别完毕的标志位
extern int banmaxian_finish;
extern uint8 class_step;
extern int car_run_mode;
extern uint8 classify_correct_finish;
extern uint8 banmaxian_allow_flag;
extern int put_out_count;  //需要放出的卡片数目
extern uint8 put_out_card_flag;
extern int Traffic_count;     //拾取的交通工具卡片数
extern int Weapon_count;      //拾取的武器总卡片数
extern int Supply_count;      //拾取的物资的总卡片数
extern uint8 CSI_island_correct_flag;
extern float Island_x,Island_y;
extern float delta_island_x,delta_island_y;
extern float delta_island_class_x,delta_island_class_y;
extern int correct_island_card_step;
extern int correct_island_x_flag,correct_island_y_flag;
extern int Island_mode;
extern double Left_Island_classify_zone_x, Left_Island_classify_zone_y;
extern double delta_find_Island_zero_x,delta_find_Island_zero_y;
extern double delta_find_Island_zero_angle; 
extern uint8 arrive_island_center_flag;
extern float island_card_center_x,island_card_center_y;//art4记录的环岛卡片坐标，矫正使用
extern int right_lie_island_upline_position;//环岛扫上边线最右列的行坐标
extern float record_island_zone_x, record_island_zone_y;
extern int Island_Zone_count;
extern int island_class_step;
extern card Island_card[5];
extern uint8 back_flag;
extern uint8 ahead_flag;
extern int second_right_lie_island_upline_position;
extern float test_top_error;
extern int Crossing_mode;
extern float crossing_card_center_x,crossing_card_center_y;
extern float crossing_correct_again_x,crossing_correct_again_y;
extern int left_lie_island_upline_position;
extern float record_crossing_zone_x,record_crossing_zone_y;    //记录完成十字卡片的区域坐标
extern float delta_crossing_class_x,delta_crossing_class_y;
extern uint8 Left_Island_Finish;
extern uint8 Left_Crossing_Finish;
extern uint8 Cross_allow_flag;
extern uint8 Island_allow_flag;
extern int zebra_card_x,zebra_card_y;

extern int pid_motor[4];

extern pid_info Pos_turn_pid[4];

extern pid_info Angle_turn_pid;

extern pid_info distance_pid[4];

extern card cross_card[5];


void Motor_Init(void);
void Encoder_Init(void);
void Read_Encoder(void);
void Car_Inverse_kinematics_solution(float target_Vx, float target_Vy, float target_Vz);
void Move_Transfrom(float target_Vx, float target_Vy, float target_Vz);
void car_run(void);
void car_run_upline_right(int target_line);
void car_run_upline_left(int target_line);
void car_stop(void);
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
void CSI_dis_new_correct(float cor_x, float cor_y);
void CSI_correct_island_correct(float Island_center_card_x, float Island_center_card_y);
void ramp_cross(int Traverse_distance, int Straight_distance);
void find_classify(int Traverse_distance, int Straight_distance);
void car_findcard(int *mode);
void car_findcard_new(int *mode);
void Left_Island_pick_and_move(int *Island_step);
void card_final_classify(int *classify_step);
void Left_Crossing_pick_and_move(int *Cross_step);
#endif
