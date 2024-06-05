#include <math.h>
#include "imu660ra.h"
#include "zf_common_headfile.h"

#define dt 0.005;		  // 滤波时间，每5ms更新一次滤波
#define LED1                        (B9 )  
#define PIT_CH                         (PIT_CH0 )                                 // 使用的定时器通道，根据需要修改，需要与isr.c中的中断函数对应

float Angle_z, Angle_Z=0; // 目标角度
float acc_y, acc_x; // y轴、x轴加速度，用于静态平衡
float Gyro_z=0;
float fil_Gyro_z; // 滤波后的陀螺仪角速度
float Angle_z=0;
float Angle_world;//用于计算卡片全局坐标的车辆角度
float kal_angle=0;
float coe_Gyro_z=0.2;
float IMU660ra_FIFO[11];
int moto_flag=0;
int gyro_i=0;
int start_flag;

void my_imu660ra_init() // 初始化函数
{
	
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 为输出模式，默认高电平，推挽输出模式 
    while(1)
    {
        if(imu660ra_init())
        {
            printf("IMU660RA init error.\r\n");                                 // IMU660RA 初始化失败
        }
        else
        {
            break;
        }
        gpio_toggle_level(LED1);                                                // 翻转 LED 状态，用于指示初始化过程中的闪烁
    }
}

/*******************平均滤波函数，用于获取角度****************/
void IMU660ra_newValues()
{
	 float sum=0;
	 static float gyro[100],sum_gyro;
	 static int gyro_flag=0,Gyro_flag;
	 
	imu660ra_get_gyro();  		
		if(gyro_flag==0)
	 {		 
		  gyro[gyro_i]= imu660ra_gyro_z;
		  fil_Gyro_z=0.0;
		  gyro_i++;
		 if(gyro_i==99)//gyro数组存满100个数据
		 {
			 moto_flag=1;
			 for(gyro_i=0;gyro_i<100;gyro_i++)
			 {
				 sum_gyro+=gyro[gyro_i];//用于去零漂
			 }
			 gyro_flag=1;
                         start_flag=1;
		 }
	 } 
	 if(gyro_flag==1)
	 {
   Gyro_z = (float)(imu660ra_gyro_z-sum_gyro/100)/16.3835;
	  if(abs((int)Gyro_z)<3)//角速度小于3时，默认为静止
	  {
		  Gyro_z=0;
	  }
	  for(Gyro_flag=1;Gyro_flag<10;Gyro_flag++)
		{	
		  IMU660ra_FIFO[Gyro_flag-1]=IMU660ra_FIFO[Gyro_flag];//FIFO 移位
		}
	  IMU660ra_FIFO[9]=Gyro_z;
	  for(Gyro_flag=0;Gyro_flag<10;Gyro_flag++)
		{	            
			sum+=IMU660ra_FIFO[Gyro_flag];//当前10个数据求平均值
		}
	  fil_Gyro_z=sum/10;
	}
}		
/**************************************************************************
函数功能：根据加速度和陀螺仪获取角度
内部参数：无
返回  值：无
**************************************************************************/
void Get_angle()
{
    IMU660ra_newValues();
	 Angle_Z+=fil_Gyro_z*dt;
	 Angle_z+=fil_Gyro_z*dt;//用作拾取卡片时
	 Angle_world=-Angle_Z;//顺时针角度为正
	 if(Angle_Z>=360) Angle_Z=Angle_Z-360;
	 if(Angle_Z<=-360) Angle_Z=Angle_Z+360;
}
/****************************** BEFIN ********************************
**@Name       : Kalman_Filter_x
**@Brief      : 获取z轴角度的卡尔曼滤波  
**@Param Accel: 加速度计测得的角度
**		  Gyro: 陀螺仪测得的角速度
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2022-06-04
******************************** END *********************************/    
float Kalman_Filter_x(float Accel,float Gyro)		
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // 加速度计噪声方差
	float Q_gyro=0.003;	//0.003 陀螺仪噪声方差，需要根据实际情况调整
	float R_angle=0.5;		// 角度测量误差方差，需要根据实际情况调整
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //更新角度

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-更新方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-更新方差的微分
	PP[0][1] += Pdot[1] * dt;   // =更新方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-更新角度误差
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //更新方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //更新角度
	Q_bias	+= K_1 * Angle_err;	 //更新角速度偏差
	angle_dot   = Gyro - Q_bias;	 //角速度(角度导数)的微分=加速度计测得的角速度
	return angle;
}