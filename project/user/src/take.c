#include "take.h"
#include "math.h"
#include "isr.h"  
#include "zf_common_headfile.h"

#define PIT_CH_TIME (PIT_CH2)

uint16 servo1_duty = 50;
uint16 servo2_duty = 50;
uint16 servo3_duty = 50;//初始电机

uint32 servo1_pwm = 0;
uint32 servo2_pwm = 0;
uint32 servo3_pwm = 0;//三个舵机的pwm值

uint8 step = 1;
uint8 side_step = 1;
uint8 arm_flag = 0;//计数完成标志
uint8 arm_pick_flag = ARM_PICK_DONE; //
uint8 arm_state_flag = ARM_STATE_OFF;

uint8 one_pick = 0;
uint8 arm_put_down = 0;//机械臂放下标志位


void PIT_CH2_Int_Init(uint32 ldval)
{
    pit_ms_init(PIT_CH2, ldval);//初始化周期为 ldval ms
    interrupt_global_enable(0);
}

void my_pwm_gpio(void)
{
  pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(35));
  pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(50)); //初始化前臂度数
  pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(79)); //云台舵机度数12  73  133

  gpio_init(C9, GPO, 0, GPO_PUSH_PULL);                                       //电磁铁                             

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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机连续控制函数
// 参数说明     _servo1_angle               舵机1的目标角度
// 参数说明     _servo2_angle               舵机2的目标角度
// 返回参数     _step_count                 舵机连续控制间隔次数
// 使用示例     servo_slow_ctrl(90, 90, 100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void servo_slow_ctrl(uint16 _servo3_angle, uint16 _servo2_angle, float _step_count)
{
  float servo3_start = (float)servo3_duty, servo2_start = (float)servo2_duty;//设置初始角度值
  float servo3_step = (float)(_servo3_angle - servo3_duty) / _step_count, servo2_step = (float)(_servo2_angle - servo2_duty) / _step_count;//每一步需要执行的步数
  while (1)
  {
    system_delay_ms(5);
		//fabsf()函数求浮点数绝对值
    if (fabsf(servo3_start - (float)_servo3_angle) >= servo3_step)//执行角度比设定的单步角度要大
      servo3_start += servo3_step;
    else//角度比设定的单步角度要小
      servo3_start = _servo3_angle;//直接更新为目标角度
		
    servo1_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo3_start);//方便debug查看pwm值
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_start));//SERVO_MOTOR_DUTY将角度转化成对应的pwm

    if (fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)
      servo2_start += servo2_step;
    else
      servo2_start = _servo2_angle;
    servo2_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start);
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start));

    if (fabsf(servo3_start - (float)_servo3_angle) <= 1 && fabsf(servo2_start - (float)_servo2_angle) <= 1)//1为误差范围，不设置0的原因是浮点数存在程序上的误差
    {
      servo3_duty = (uint16)_servo3_angle;
      servo2_duty = (uint16)_servo2_angle;//更新角度
      return;
    }
  }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     侧面舵机连续控制函数
// 参数说明     _servo3_angle               舵机3的目标角度
// 返回参数     _step_count                 舵机连续控制间隔次数
// 使用示例     servo_slow_ctrl(90, 90, 100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void side_servo_slow_ctrl(uint16 _servo3_angle,float _step_count)
{
  float servo3_start = (float)servo3_duty;//设置初始角度值
  float servo3_step = (float)(_servo3_angle - servo3_duty) / _step_count;//每一步需要执行的步数
  while (1)
  {
    system_delay_ms(5);
		//fabsf()函数求浮点数绝对值
    if (fabsf(servo3_start - (float)_servo3_angle) >= servo3_step)//执行角度比设定的单步角度要大
      servo3_start += servo3_step;
    else//角度比设定的单步角度要小
      servo3_start = _servo3_angle;//直接更新为目标角度
		
    servo3_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo3_start);//pwm值
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_start));//SERVO_MOTOR_DUTY舵机角度转化成pwm值

    if (fabsf(servo3_start - (float)_servo3_angle) <= 1)//1为误差范围，不设置0的原因是浮点数存在程序上的误差
    {
      servo3_duty = (uint16)_servo3_angle;//更新角度
      return;
    }
  }
}
/**************************************************************************
函数功能：机械臂控制模式选择
入口参数：mode
返回值：无
备注：1. 拾取模式 2. 收纳模式 3. 归中模式 4. 调试模式（按键） 5. 其他默认为归中
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
    servo_slow_ctrl(148, 141, 5);
	  ips114_show_string( 0 , 40,   "SUCCESS");                          //测试用
    break;

  case 2: //模式2收纳模式
    gpio_set_level(C9, 1);
    switch (step)
  {
    case 1:
        servo_slow_ctrl(148, 141, 10);
        PIT_CH2_Int_Init(10);
    if(arm_flag==1)//延时计数完成
    {
       mode = 2;
       step = 2;
       arm_flag = 0;//清零计数标志位
    }
    case 2:
    servo_slow_ctrl(22, 141, 50); // 58 110   58  34//动前臂
    PIT_CH2_Int_Init(10);
    if(arm_flag==1)//延时计数完成
    {
       mode = 2;
       step = 3;
       arm_flag = 0;
    }
    case 3:
    servo_slow_ctrl(22, 38, 100); //收后臂
    PIT_CH2_Int_Init(10);
      if(arm_flag==1)//延时计数完成
    {
       arm_flag = 0;
       step = 0;//清空step
       mode = 3;//跳至默认模式3
    }
    break;
  }
  case 3: //归中模式(默认)
    gpio_set_level(C9, 0);
    servo_slow_ctrl(50, 50, 100);//默认模式
	  step = 1;//将步骤重置为一，此时mode已经变更为3，不会再返回至mode2执行
    break;

  case 4: //调试模式（按键）
    gpio_set_level(C9, 1);
    servo_slow_ctrl(148, 110, 10);
    break;

  case 5: //调试模式（按键）
    if (!gpio_get_level(C14) && gpio_get_level(C27))
    {
      servo3_duty += 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_duty));
    }
    //锟斤拷锟铰诧拷C27锟斤拷锟斤拷锟斤拷C26锟斤拷应锟斤拷C30锟斤拷锟斤拷嵌燃锟叫?
    if (!gpio_get_level(C26) && gpio_get_level(C27))
    {
      servo3_duty -= 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo3_duty));
    }
    //锟斤拷锟较诧拷C27锟斤拷锟斤拷锟斤拷C14锟斤拷应锟斤拷C31锟斤拷锟斤拷嵌锟斤拷锟斤拷锟?
    if (!gpio_get_level(C14) && !gpio_get_level(C27))
    {
      servo2_duty += 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_duty));
    }
    //锟斤拷锟较诧拷C27锟斤拷锟斤拷锟斤拷C26锟斤拷应锟斤拷C31锟斤拷锟斤拷嵌燃锟叫?
    if (!gpio_get_level(C26) && !gpio_get_level(C27))
    {
      servo2_duty -= 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_duty));//直锟接革拷值锟桔诧拷
    }
    if (!gpio_get_level(D4))
      gpio_set_level(C9, 1); //锟竭碉拷平锟接达拷锟斤拷
    else
      gpio_set_level(C9, 0);
    break;
  case 6: //模式6侧边舵机拾取
  gpio_set_level(C9, 1);//电磁铁给电
    switch (side_step)
  {
    case 1:
        side_servo_slow_ctrl(20, 30);//侧边舵机控制，吸门
        PIT_CH2_Int_Init(10);
    if(arm_flag==1)//计数延时完成标志
    {
       mode = 6;
       side_step = 2;
       arm_flag = 0;//重置计数标志
    }
    case 2:
    side_servo_slow_ctrl(141, 50); //放门
    PIT_CH2_Int_Init(10);
    if(arm_flag==1)//计数延时完成标志
    {
       mode = 6;
       side_step = 3;
       arm_flag = 0;//重置计数标志
    }
    case 3:
    side_servo_slow_ctrl(22, 100); //门回归初始位置
    PIT_CH2_Int_Init(100);
      if(arm_flag==1)//计数延时完成标志
    {
       arm_flag = 0;
       side_step = 4;//更新步骤
       mode = 0;//模式变为0
    }
    case 4:
    gpio_set_level(C9, 0);//电磁铁断电
    side_servo_slow_ctrl(50, 100); //回归初始状态
    PIT_CH2_Int_Init(100);
    if(arm_flag==1)//计数延时完成标志
    {
       arm_flag = 0;
       side_step = 0;//全部完成，步骤置0
       mode = 0;//模式重置
    }
    break;
  }
  default:
    break; //退出
  }
}
//*******************************舵机测试函数******************************//
void text_arm(void)
{
	my_pwm_gpio();//初始化
	arm_control(2);//测试舵机模式2
//	ips114_show_string( 0 , 40,   "SUCCESS");                          // 测试通过
}