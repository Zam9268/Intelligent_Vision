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

int finish_count = 0;//中断结束计数位
int once = 1;
extern int count;
extern int arm_flag;


void my_pwm_gpio(void)
{
 pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(50));
 pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(50)); //初始化前臂度数
 pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(131)); //云台舵机度数12  85  131
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

 case 1:                  //模式1拾取模式
    gpio_set_level(C9, 1); //电磁铁给电
//	  ips114_show_string( 0 , 40,   "SUCCESS");                          //测试用
    servo_slow_ctrl(165, 148, 50);
	  ips114_show_string( 0 , 40,   "SUCCESS");                          //测试用
   break;

 case 2: //模式2收纳模式
   gpio_set_level(C11, 1);
   servo_slow_ctrl(173, 50, 50);//下前臂
   system_delay_ms(1000);
   servo_slow_ctrl(173, 148, 50);
   system_delay_ms(1000);
   servo_slow_ctrl(55, 100, 50); //动后臂
   system_delay_ms(1000);
   servo_slow_ctrl(55, 45, 100); //收前臂
   system_delay_ms(1000);
   gpio_set_level(C11, 0);
   break;

 case 3: //归中模式(默认)
   gpio_set_level(C11, 0);
   servo_slow_ctrl(60, 50, 100);//默认模式
   break;

 case 4: //360度舵机调参
  //  gpio_set_level(C9, 1);
  //  servo_slow_ctrl(148, 110, 10);
  servo3_duty = 30;
  pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_duty));
  system_delay_ms(1000);
  servo3_duty = 90;
  pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_duty));
  system_delay_ms(1000);

   break;

 case 5: //侧面舵机关门
	 gpio_set_level(C10, 1);//侧面电磁铁上电
	 servo3_duty = 85;
   pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_duty));
   system_delay_ms(1000);
   side_servo_slow_ctrl(160, 100);//侧面舵机控制
   system_delay_ms(1000);
 	side_servo_slow_ctrl(30, 10);//侧面舵机控制，默认角度
	system_delay_ms(1000);
   break;
  case 6: //模式6侧边舵机拾取
  gpio_set_level(C10, 0);//电磁铁断电
	side_servo_slow_ctrl(50, 10);//侧面舵机控制，默认角度
	system_delay_ms(1000);
	break;
//    switch (side_step)
//  {
//    case 1:
//        side_servo_slow_ctrl(20, 30);//侧边舵机控制，吸门
//    if(arm_flag==1)//计数延时完成标志
//    {
//       mode = 6;
//       side_step = 2;
//       arm_flag = 0;//重置计数标志
//    }
//    case 2:
//    side_servo_slow_ctrl(141, 50); //放门
//    PIT_CH2_Int_Init(10);
//    if(arm_flag==1)//计数延时完成标志
//    {
//       mode = 6;
//       side_step = 3;
//       arm_flag = 0;//重置计数标志
//    }
//    case 3:
//    side_servo_slow_ctrl(22, 100); //门回归初始位置
//      if(arm_flag==1)//计数延时完成标志
//    {
//       arm_flag = 0;
//       side_step = 4;//更新步骤
//       mode = 0;//模式变为0
//    }
//    case 4:
//    gpio_set_level(C9, 0);//电磁铁断电
//    side_servo_slow_ctrl(50, 100); //回归初始状态
//    if(arm_flag == 1)//计数延时完成标志
//    {
//       arm_flag = 0;
//       side_step = 0;//全部完成，步骤置0
//       mode = 0;//模式重置
//    }
//    break;
//  }
 default:
   break; //退出
 }
}
//*******************************舵机测试函数******************************//
void test_arm(void)
{
	if(arm_pick_flag==ARM_PICK_NOT_DONE)
	{
		arm_control(2);//捡卡片
	  arm_control(3);//默认模式
		arm_pick_flag=ARM_PICK_DONE;
//    arm_control(6);//开门
	}
	if(arm_pick_flag==ARM_PICK_DONE)
	{
		arm_control(5);//开门
//		arm_control(6);//恢复默认
		arm_pick_flag=2;
	}
}