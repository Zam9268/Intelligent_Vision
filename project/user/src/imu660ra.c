#include"math.h"
#include"imu660ra.h"
#include "zf_common_headfile.h"

#define dt 0.005;		  //滤波周期，每5ms进行一次滤波
#define LED1                        (B9 )  
#define PIT_CH                         (PIT_CH0 )                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用

float Angle_z,Angle_Z=90;//目标角度
float acc_y , acc_x;//y轴，x轴加速度，用于解算姿态角
float Gyro_z=0;
float fil_Gyro_z;//陀螺仪角速度
float Angle_z=0;
float kal_angle=0;
float coe_Gyro_z=0.2;
float IMU660ra_FIFO[11];
int moto_flag=0;
int gyro_i=0;
int start_flag;

void my_imu660ra_init()//放在主函数
{
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式 
    while(1)
    {
        if(imu660ra_init())
        {
            printf("\r\nIMU660RA init error.");                                 // IMU660RA 初始化失败
        }
        else
        {
            break;
        }
        gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
    pit_ms_init(PIT_CH, 5);
    interrupt_global_enable(0);
}

/*******************平均递推滤波，用来获取角度****************/
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
		 if(gyro_i==99)//gyro数组存储到达上限
		 {
			 moto_flag=1;
			 for(gyro_i=0;gyro_i<100;gyro_i++)
			 {
				 sum_gyro+=gyro[gyro_i];//
			 }
			 gyro_flag=1;
                         start_flag=1;
		 }
	 } 
	 if(gyro_flag==1)
	 {
   Gyro_z = (float)(imu660ra_gyro_z-sum_gyro/100)/16.3835;
	  if(abs((int)Gyro_z)<3)//角速度小于3时  默认为小车静止  
	  {
		  Gyro_z=0;
	  }
	  for(Gyro_flag=1;Gyro_flag<10;Gyro_flag++)
		{	
		  IMU660ra_FIFO[Gyro_flag-1]=IMU660ra_FIFO[Gyro_flag];//FIFO 操作
		}
	  IMU660ra_FIFO[9]=Gyro_z;
	  for(Gyro_flag=0;Gyro_flag<10;Gyro_flag++)
		{	            
			sum+=IMU660ra_FIFO[Gyro_flag];//求当前数组的合，再取平均值
		}
	  fil_Gyro_z=sum/10;
	}
}		
/**************************************************************************
函数功能：对角速度积分 得到角度
入口参数：无
返回  值：无
**************************************************************************/
void Get_angle()
{
    IMU660ra_newValues();
	 Angle_Z-=fil_Gyro_z*dt;
	 if(Angle_Z>=360) Angle_Z=Angle_Z-360;
	 if(Angle_Z<=-360) Angle_Z=Angle_Z+360;
}
/****************************** BEFIN ********************************
**@Name       : Kalman_Filter_x
**@Brief      : 获取z轴角度简易卡尔曼滤波  
**@Param Accel: 加速度算出的角度
**		  Gyro: 陀螺仪的角速度
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2022-06-04
******************************** END *********************************/    
float Kalman_Filter_x(float Accel,float Gyro)		
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // 过程噪声的协方差
	float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵，参数要调整
	float R_angle=0.5;		// 测量噪声的协方差 既测量偏差，参数要调整
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
	return angle;
}