#include "take.h"
#include "math.h"
#include "isc.h"  
#include "zf_common_headfile.h"

#define PIT_CH_TIME (PIT_CH2)

uint16 servo1_duty = 50;
uint16 servo2_duty = 50;//舵机初始角度

uint32 servo1_pwm = 0;
uint32 servo2_pwm = 0;//舵机占空比

uint8 step = 1;
uint8 arm_pick_flag = ARM_PICK_DONE; //
uint8 arm_state_flag = ARM_STATE_OFF;

uint8 one_pick = 0;
uint8 arm_put_down = 0;//机械臂放下物品的标志位


void PIT_CH2_Int_Init(uint32 ldval)
{
    pit_ms_init(PIT_CH_TIME, ldval);//初始化CH2通道，中断周期为 ldval ms
    interrupt_global_enable(0);
}

void my_pwm_gpio(void)
{
  pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(35));
  pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(50)); //初始化和归中的位置相等
  pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(79)); //初始化云台舵机12  73  133

  gpio_init(C9, GPO, 0, GPO_PUSH_PULL);                                       //电磁铁                              //60为左边放图片，88右边放图片

  gpio_init(B14, GPO, 0, GPO_PUSH_PULL); //电磁锁
  gpio_init(B16, GPO, 0, GPO_PUSH_PULL); //电磁锁
  gpio_init(B17, GPO, 0, GPO_PUSH_PULL); //电磁锁

  gpio_init(C12, GPI, 1, GPI_PULL_UP); //按键
  gpio_init(C13, GPI, 1, GPI_PULL_UP); //按键
  gpio_init(C14, GPI, 1, GPI_PULL_UP); //按键
  gpio_init(C15, GPI, 1, GPI_PULL_UP); //按键

  gpio_init(C26, GPI, 1, GPI_PULL_UP); //拨码开关
  gpio_init(C27, GPI, 1, GPI_PULL_UP); //拨码开关
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
  float servo1_start = (float)servo1_duty, servo2_start = (float)servo2_duty;//?bug出现的地方？
  float servo1_step = (float)(_servo1_angle - servo1_duty) / _step_count, servo2_step = (float)(_servo2_angle - servo2_duty) / _step_count;//每一步需要执行的角度
  while (1)
  {
    system_delay_ms(5);
		//fabsf函数是用来计算单精度浮点数的绝对值的，计算双精度浮点数的绝对值，然后将结果转换成单精度浮点数返回
    if (fabsf(servo1_start - (float)_servo1_angle) >= servo1_step)//所求绝对值大于单步调整的舵机角度时，初始角度加单步的舵机角度
      servo1_start += servo1_step;
    else//所求绝对值小于单步调整的舵机角度
      servo1_start = _servo1_angle;//初始角度直接更新为_servo1_angle
		
    servo1_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo1_start);//查看占空比
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_start));//SERVO_MOTOR_DUTY将转动角度转化成舵机占空比(舵机脉宽)

    if (fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)
      servo2_start += servo2_step;
    else
      servo2_start = _servo2_angle;
    servo2_pwm = (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start);
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start));

    if (fabsf(servo1_start - (float)_servo1_angle) <= 1 && fabsf(servo2_start - (float)_servo2_angle) <= 1)//1为误差范围，存在浮点数误差，输入进来的角度与初始角度误差很小时
    {
      servo1_duty = (uint16)_servo1_angle;
      servo2_duty = (uint16)_servo2_angle;//
      return;
    }
  }
}

/**************************************************************************
函数功能：机械臂控制模式选择
入口参数：mode
返回值：无
备注：1. 拾取模式 2. 收纳模式 3. 归中模式 4. 调试模式（按键） 5. 其他默认为归中，暂时测试阶段
调用示例：arm_control(1);
**************************************************************************/
void arm_control(uint8 mode)
{
//  ips114_show_string( 0 , 40,   "SUCCESS");                          // 显示字符串
  switch (mode)
  {

  case 1:                  //模式1：拾取模式
    gpio_set_level(C9, 1); //电磁铁给高电平带电
//	  ips114_show_string( 0 , 40,   "SUCCESS");                          //电磁铁上电成功
    servo_slow_ctrl(148, 141, 5);
	  ips114_show_string( 0 , 40,   "SUCCESS");                          //测试舵机，显示成功，但是舵机不会动
    break;

  case 2: //模式2：收纳模式
    gpio_set_level(C9, 1);
    switch (step)
  {
    case 1:
        servo_slow_ctrl(148, 110, 10);
        PIT_CH2_Int_Init(10);
    if(arm_flag==1)//定时器计数完成标志
    {
       mode = 2;
       step = 2;
    }
    case 2:
    servo_slow_ctrl(22, 110, 50); // 58 110   58  34//这个速度不要乱改,先转前臂，再转后臂
    PIT_CH2_Int_Init(10);
    if(arm_flag==1)//定时器计数完成标志
    {
       mode = 2;
       step = 3;
    }
    case 3:
    servo_slow_ctrl(22, 38, 100); //三套参数的确定时间：2023年4月24日18:02:30
    PIT_CH2_Int_Init(10);
      if(arm_flag==1)//定时器计数完成标志
    {
       mode = 3;//不再进入回到模式2，进入默认模式3断电
    }
    break;
  }
  case 3: //模式3：归中模式
    gpio_set_level(C9, 0);
    servo_slow_ctrl(50, 50, 100);//默认模式
    break;

  case 4: //模式4：单搬保持模式
    gpio_set_level(C9, 1);
    servo_slow_ctrl(148, 110, 10);
    break;

  case 5: //模式5：调试模式
          //往下拨C27，按下C14,对应的C30舵机角度增加
    if (!gpio_get_level(C14) && gpio_get_level(C27))
    {
      servo1_duty += 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_duty));
    }
    //往下拨C27，按下C26对应的C30舵机角度减小
    if (!gpio_get_level(C26) && gpio_get_level(C27))
    {
      servo1_duty -= 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_duty));
    }
    //往上拨C27，按下C14对应的C31舵机角度增加
    if (!gpio_get_level(C14) && !gpio_get_level(C27))
    {
      servo2_duty += 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_duty));
    }
    //往上拨C27，按下C26对应的C31舵机角度减小
    if (!gpio_get_level(C26) && !gpio_get_level(C27))
    {
      servo2_duty -= 10;
      system_delay_ms(300);
      pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_duty));//直接赋值观察
    }
    if (!gpio_get_level(D4))
      gpio_set_level(C9, 1); //高电平加磁性
    else
      gpio_set_level(C9, 0);
    break;

  default:
    break; //外对应C30（servo1），内对应C31（servo2）
  }
}
//*******************************测试机械臂******************************//
void text_arm(void)
{
	my_pwm_gpio();//初始化各个引脚
	arm_control(1);//进入函数后没有反应，但是电磁铁上电成功
//	ips114_show_string( 0 , 40,   "SUCCESS");                          // 显示字符串
}