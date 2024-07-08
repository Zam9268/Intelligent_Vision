#ifndef _TAKE_H
#define _TAKE_H

#include "zf_common_headfile.h"

#define SERVO_MOTOR_PWM1                (PWM4_MODULE2_CHA_C30)                      // 前臂舵机
#define SERVO_MOTOR_PWM2                (PWM4_MODULE3_CHA_C31)                       // 后臂舵机
#define SERVO_MOTOR_PWM3                (PWM2_MODULE0_CHA_C6)                       //  360度舵机
#define SERVO_MOTOR_PWM4                (PWM2_MODULE1_CHA_C8)                       //侧面舵机

#define SERVO_MOTOR_FREQ                (50)                                       // 舵机频率
#define SERVO_MOTOR_DUTY(x)             ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))    //舵机角度转换成对应的pwm值
#define SERVO_MOTOR_DUTY_360(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/180.0))    //舵机角度转换成对应的pwm值

#define class_1_angle            100    //a类对应舵机角度173度
#define class_2_angle            66    //b类对应舵机角度113度
#define class_3_angle            30     //c类对应舵机角度53度

#define class_first_angle            18      //环岛第一张卡片分类区域 173
#define class_second_angle           52     //环岛第二张卡片区域
#define class_third_angle            88     //环岛第三张卡片区域
#define class_fouth_angle            124    //环岛第四张卡片区域
#define class_fifth_angle            160    //环岛第五张卡片区域


#define ARM_PICK_NOT_DONE        (0)
#define ARM_PICK_DONE            (1)//����ʰȡ��ɵı�־λ

#define ARM_STATE_OFF            (0)
#define ARM_STATE_ON             (1)

extern uint16 servo1_duty;
extern uint16 servo2_duty;
extern uint16 servo3_duty;

extern uint32 servo1_pwm;
extern uint32 servo2_pwm;
extern uint32 servo3_pwm;//���ռ�ձ�
//extern uint16 servo3_duty;
extern uint8 step;
extern int card_classify_count;
extern uint8 side_step; 
extern uint8 arm_pick_flag;
extern uint8 arm_state_flag;
extern uint8 one_pick;
extern uint8 arm_put_down;
extern int finish_count;
//extern uint8 key_speed;

void my_pwm_gpio(void);
void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count);
void side_servo_slow_ctrl(uint16 _servo3_angle,float _step_count);
void arm_control(uint8 mode);
void test_arm(void);
void classify_360(uint8 card_classify_type);
void classify_little_360(int card_little_classify_type);
//void classify_pick(uint8 type, uint8 on_off);
// void classify_pick(uint8 mode);
// void tri_servo(uint8 tri_mode);
// void Arm_Pick(void);
// void Key_Set(void);










#endif