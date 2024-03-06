#ifndef _TAKE_H
#define _TAKE_H

#include "zf_common_headfile.h"

#define SERVO_MOTOR_PWM1                (PWM4_MODULE2_CHA_C30)                      // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM2                (PWM4_MODULE3_CHA_C31)                       // 定义主板上舵机对应引脚 
#define SERVO_MOTOR_PWM3                (PWM2_MODULE0_CHA_C6)                       // 定义主板上舵机对应引脚 
#define SERVO_MOTOR_FREQ                (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-30
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))    //舵机角度设置为0 - 180，90度为中值


#define ARM_PICK_NOT_DONE        (0)
#define ARM_PICK_DONE            (1)//设置拾取完成的标志位

#define ARM_STATE_OFF            (0)
#define ARM_STATE_ON             (1)

extern uint16 servo1_duty;
extern uint16 servo2_duty;
extern uint16 servo3_duty;
extern uint8 arm_pick_flag;
extern uint8 arm_state_flag;
extern uint8 one_pick;
extern uint8 arm_put_down;
extern uint8 key_speed;

void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count);
void arm_control(uint8 mode);
void my_pwm_gpio(void);
//void classify_pick(uint8 type, uint8 on_off);
// void classify_pick(uint8 mode);
// void tri_servo(uint8 tri_mode);
// void Arm_Pick(void);
// void Key_Set(void);










#endif