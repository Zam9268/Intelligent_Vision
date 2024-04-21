extern float Angle_z,Angle_Z;//目标角度
extern float acc_y , acc_x;//y轴，x轴加速度，用于解算姿态角
extern float Gyro_z;
extern float fil_Gyro_z;//陀螺仪角速度
extern float Angle_z;
extern float kal_angle;
extern float coe_Gyro_z;
extern float IMU660ra_FIFO[11];
extern int moto_flag;
extern int gyro_i;
extern int start_flag;

void my_imu660ra_init();
void IMU660ra_newValues();
void Get_angle();
float Kalman_Filter_x(float Accel,float Gyro);