#include "take.h"
#include "math.h"
#include "isr.h"  
#include "zf_common_headfile.h"

#define PIT_CH_TIME (PIT_CH2)

uint16 servo1_duty = 50;
uint16 servo2_duty = 50;
uint16 servo3_duty = 50;//初始电机

uint16 servo4_duty = 50;//侧面舵机

uint32 servo1_pwm = 0;
uint32 servo2_pwm = 0;
uint32 servo3_pwm = 0;//三个舵机的pwm值

uint32 servo4_pwm = 0;//侧面舵机的pwm值

uint8 step = 1;
uint8 side_step = 1;
uint8 arm_pick_flag = 0; //机械臂拾取标志
uint8 arm_state_flag = 0;//机械臂开启标志

uint8 one_pick = 0;
uint8 arm_put_down = 0;//机械臂放下标志位

int card_classify_count=0;

int finish_count = 0;//中断结束计数位
int once = 1;
int pick_count=0;
extern int count;
extern int arm_flag;


void my_pwm_gpio(void)
{
 pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(60));
 pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(70)); //初始化前臂度数 
 pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(137)); //  30 66 100 137 175
// pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY_360(145)); //15  87 145(初始化最优角度) 218(初始化最优角度) 290 
 pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(30));

 gpio_init(C11, GPO, 0, GPO_PUSH_PULL);//正面电磁铁
 gpio_init(C10, GPO, 0, GPO_PUSH_PULL);	//电磁铁                             

 gpio_init(B14, GPO, 0, GPO_PUSH_PULL); //
 gpio_init(B16, GPO, 0, GPO_PUSH_PULL); //
 gpio_init(B17, GPO, 0, GPO_PUSH_PULL); //

 gpio_init(C12, GPI, 1, GPI_PULL_UP); 
 gpio_init(C13, GPI, 1, GPI_PULL_UP); 
 gpio_init(C14, GPI, 1, GPI_PULL_UP); 
 gpio_init(C15, GPI, 1, GPI_PULL_UP); 

 gpio_init(C26, GPI, 1, GPI_PULL_UP); //
 gpio_init(C27, GPI, 1, GPI_PULL_UP); //
}
/**************************************************************************
函数功能：舵机连续控制
入口参数：_servo1_angle,_servo2_angle,_step_count（舵机1目标速度，舵机2目标速度，步数设置）
返回值：无
备注：_step_count越小，速度越快（一般这样设置：快速：10；中速：50；慢速：100）





调用示例：servo_slow_ctrl(148,110,10);
**************************************************************************/
void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count)
{
  float servo1_start = (float)servo1_duty, servo2_start = (float)servo2_duty;
  float servo1_step = (float)(_servo1_angle - servo1_duty) / _step_count, servo2_step = (float)(_servo2_angle - servo2_duty) / _step_count;//每一步需要执行的角度
  while (1)
  {
    system_delay_ms(5);//fabsf函数是用来计算单精度浮点数的绝对值的，计算双精度浮点数的绝对值，然后将结果转换成单精度浮点数返回
    if (fabsf(servo1_start - (float)_servo1_angle) >= servo1_step)//所求绝对值大于单步调整的舵机角度时，初始角度加单步的舵机角度
      servo1_start += servo1_step;
    else//所求绝对值小于单步调整的舵机角度
      servo1_start = _servo1_angle;//初始角度直接更新为_servo1_angle
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_start));

    if (fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)
      servo2_start += servo2_step;
    else
      servo2_start = _servo2_angle;
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start));

    if (fabsf(servo1_start - (float)_servo1_angle) < 1 && fabsf(servo2_start - (float)_servo2_angle) < 1)
    {
      servo1_duty = (uint16)_servo1_angle;
      servo2_duty = (uint16)_servo2_angle;
      return;
    }
  }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     侧面舵机连续控制函数
// 参数说明     _servo3_angle               舵机3的目标角度
// 返回参数     _step_count                 舵机连续控制间隔次数
// 使用示例     side_servo_slow_ctrl(90, 100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void side_servo_slow_ctrl(uint16 _servo4_angle,float _step_count)
{
 float servo4_start = (float)servo4_duty;//设置初始角度值
 float servo4_step = (float)(_servo4_angle - servo4_duty) / _step_count;//每一步需要执行的步数
 while (1)
 {
   system_delay_ms(5);
		//fabsf()函数求浮点数绝对值
   if (fabsf(servo4_start - (float)_servo4_angle) >= servo4_step)//执行角度比设定的单步角度要大
     servo4_start += servo4_step;
   else//角度比设定的单步角度要小
     servo4_start = _servo4_angle;//直接更新为目标角度
		
   servo4_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo4_start);//pwm值
   pwm_set_duty(SERVO_MOTOR_PWM4, (uint32)SERVO_MOTOR_DUTY((uint16)servo4_start));//SERVO_MOTOR_DUTY舵机角度转化成pwm值

   if (fabsf(servo4_start - (float)_servo4_angle) <= 1)//1为误差范围，不设置0的原因是浮点数存在程序上的误差
   {
     servo4_duty = (uint16)_servo4_angle;//更新角度
     return;
   }
 }
}
/**************************************************************************
函数功能：机械臂控制模式选择
入口参数：mode
返回值：无
备注：1. 拾取模式 2. 收纳模式 3. 归中模式 4. 360度舵机调参 5. 其他默认为归中
调用示例：arm_control(1);
**************************************************************************/
void arm_control(uint8 mode)
{
//  ips114_show_string( 0 , 40,   "SUCCESS");                          // 测试使用
 switch (mode)
 {

 case 1:                  //模式1单拾取模式
    gpio_set_level(C9, 1); //电磁铁给电
//	  ips114_show_string( 0 , 40,   "SUCCESS");                          //测试用
    servo_slow_ctrl(165, 148, 50);
	  ips114_show_string( 0 , 40,   "SUCCESS");                          //测试用
   break;

 case 2: //模式2收纳模式，减第一张卡片时的角度
   gpio_set_level(C11, 1);
   servo_slow_ctrl(175, 80, 50);
   system_delay_ms(300);
   servo_slow_ctrl(170, 151, 50);
   system_delay_ms(300);
   servo_slow_ctrl(165, 100, 50);
   system_delay_ms(300);
   servo_slow_ctrl(30, 100, 20); //?????
   system_delay_ms(300);
   servo_slow_ctrl(30, 50, 20); //?????
   system_delay_ms(300);
   gpio_set_level(C11, 0);
   break;

 case 3: //放出卡片
   gpio_set_level(C11, 1);
   servo_slow_ctrl(0, 90, 20);//收前臂
   system_delay_ms(300);
   servo_slow_ctrl(0, 45, 100); //动后臂    15 45
   system_delay_ms(300);
    servo_slow_ctrl(0, 70, 50); //动后臂    15 45
    system_delay_ms(300);
   servo_slow_ctrl(170, 70, 100); //取出卡片 25 75
   system_delay_ms(300);
   servo_slow_ctrl(170, 150, 100); //取出卡片 25 75
   system_delay_ms(300);
   break;

 case 4: //机械臂默认模式
   gpio_set_level(C11, 0);
   servo_slow_ctrl(60, 70, 100);//默认模式
   break;

   break;

 case 5: //侧面舵机关门
	 servo3_duty = 135;
   pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_duty));
  break;
  case 6: //模式6侧边舵机拾取
  gpio_set_level(C10, 0);//电磁铁断电
	side_servo_slow_ctrl(50, 10);//侧面舵机控制，默认角度
	system_delay_ms(1000);
	break;

 default:
   break; //退出
 }
}
/**************************************************************************
函数功能：360度舵机散落卡片分类
入口参数：mode
返回值：无
备注：1.A类  2.B类  3.C类
调用示例：classify_360(card_type);
**************************************************************************/
void classify_360(uint8 card_classify_type)
{
  switch(card_classify_type)
  {
  case 1:
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_1_angle));
    break;
  case 2:
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_2_angle));
    break;
  case 3:
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_3_angle));
    break;  
  }
}
/**************************************************************************
函数功能：360度舵机环岛/十字分类
入口参数：mode
返回值：无
备注：1.A类  2.B类  3.C类
调用示例：classify_360(card_type);
**************************************************************************/
void classify_little_360(int card_little_classify_type)
{
  switch(card_little_classify_type)
  {
  case 0:
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_first_angle));
    break;
  case 1:
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_second_angle));
    break;
  case 2:
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_third_angle));
    break;
  case 3:
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_fouth_angle));
    break;
  case 4:
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_fifth_angle));
    break;
  }
}
//*******************************舵机测试函数******************************//
void test_arm(void)
{
	if(pick_count<1)
	{
//		pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_A_angle));
//		system_delay_ms(1000);
//		pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_B_angle));
//		system_delay_ms(1000);
//		pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)class_C_angle));
//    system_delay_ms(1000);
		arm_control(2);//放卡片
	  arm_control(4);//默认模式
		pick_count++;
	}
}