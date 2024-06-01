#include"math.h"
#include"imu660ra.h"
#include "zf_common_headfile.h"

#define dt 0.005;		  //执行周期为10ms
#define LED1                        (B9 )  

float Angle_z,Angle_Z;//
float acc_y , acc_x;//x，y轴加速度
float Gyro_z=0;
float fil_Gyro_z;//陀螺仪角速度
float Angle_z=0;
float kal_angle=0;
float coe_Gyro_z=0.2;
float IMU660ra_FIFO[11];
int moto_flag=0;
int gyro_i=0;
int start_flag;

void my_imu660ra_init()//陀螺仪初始化
{
	
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             //LED1给高电平 
    while(1)
    {
        if(imu660ra_init())
        {
           ips114_show_string( 0 , 40,   "ERROR");                          // 测试通过，确实会进入判断条件来修改数值                                 // IMU660RA初始化失败
        }
        else
        {
            break;
        }
        gpio_toggle_level(LED1);                                                // led电平翻转
    }
}

/*******************平均递推滤波****************/
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
		 if(gyro_i==99)//gyro获取完毕
		 {
			 for(gyro_i=0;gyro_i<100;gyro_i++)
			 {
				sum_gyro+=gyro[gyro_i];//获取总零漂值
			 }
			gyro_flag=1;//完成获取零漂值，进入下一步
		 }
	 } 
	 if(gyro_flag==1)
	 {
      Gyro_z = (float)(imu660ra_gyro_z-sum_gyro/100)/16.3835;
	  if(abs((int)Gyro_z)<3)//
	  {
		  Gyro_z=0;
	  }
	  for(Gyro_flag=1;Gyro_flag<10;Gyro_flag++)
		{	
		  IMU660ra_FIFO[Gyro_flag-1]=IMU660ra_FIFO[Gyro_flag];//FIFO ����
		}
	  IMU660ra_FIFO[9]=Gyro_z;
	  for(Gyro_flag=0;Gyro_flag<10;Gyro_flag++)
		{	            
			sum+=IMU660ra_FIFO[Gyro_flag];//��ǰ����ĺϣ���ȡƽ��ֵ
		}
	  fil_Gyro_z=sum/10;
	}
}		
/**
 * @brief 获取当前角度值
 * @param 输入：Tar_angle_Z
 * @return 无
 * @attention 
 */
void Get_angle()
{
    IMU660ra_newValues();
	 Angle_Z-=fil_Gyro_z*dt;
	 if(Angle_Z>=360) Angle_Z=Angle_Z-360;
	 if(Angle_Z<=-360) Angle_Z=Angle_Z+360;
}
/****************************** BEFIN ********************************
**@Name       : Kalman_Filter_x
**@Brief      : 卡尔曼滤波 
**@Param Accel: 角度
**		  Gyro: 角速度
**@Return     : None
**@Author     : @mayuxin
******************************** END *********************************/    
float Kalman_Filter_x(float Accel,float Gyro)		
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // 
	float Q_gyro=0.003;	//0.003 
	float R_angle=0.5;		// 
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // 

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // 
	PP[0][1] += Pdot[1] * dt;   // 
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //
	Q_bias	+= K_1 * Angle_err;	 //
	angle_dot   = Gyro - Q_bias;	 //
	return angle;
}
void test_imu()
{
    imu660ra_get_acc();                                                         // 获取 IMU660RA 的加速度测量数值
    imu660ra_get_gyro();                                                        // 获取 IMU660RA 的角速度测量数值
}