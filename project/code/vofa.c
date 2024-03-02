#include "vofa.h"
#include "usart.h"
#include "OLED.h"


//RawData数据协议 测试
void RawData_Test(void)		//  直接当串口助手使用 测试是否可行
{
    u1_SendByte(0x40);
    u1_SendByte(0x41);
    u1_SendByte(0x42);
    u1_SendByte(0x43);
    u1_SendByte(0x0d);
    u1_SendByte(0x0a);
}

//FireWater数据协议 测试
float a=5,b=10,c=20;
void FireWater_Test(void)
{
    a+=100;
    b+=50;
    c+=10;
    u1_printf("%.2f,%.2f,%.2f\n",a,b,c);
}


//使用justfloat 数据协议 所需

/*
要点提示:
1. float和unsigned long具有相同的数据结构长度
2. union据类型里的数据存放在相同的物理空间
*/
typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;


/*
将浮点数f转化为4个字节数据存放在byte[4]中
*/
void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}

/*
将4个字节数据byte[4]转化为浮点数存放在*f中
*/
void Byte_to_Float(float *f,unsigned char byte[])
{
    FloatLongType fl;
    fl.ldata=0;
    fl.ldata=byte[3];
    fl.ldata=(fl.ldata<<8)|byte[2];
    fl.ldata=(fl.ldata<<8)|byte[1];
    fl.ldata=(fl.ldata<<8)|byte[0];
    *f=fl.fdata;
}

void Test_FloatAndByte(void)	//浮点数和四字节数据转换示例
{
	float a=1.0;			//发送的数据 两个通道
	
	u8 byte[4]={0};			//float转化为4个字节数据
	
	Float_to_Byte(a,byte);
	//u1_printf("%f\r\n",a);
	u1_SendArray(byte,4);	//1转化为4字节数据 就是  0x00 0x00 0x80 0x3F
	a=2;	//改变a值
	Byte_to_Float(&a,byte);		//将byte数据转换为浮点型数据
	u1_SendByte(a);			//a = 1.0
}

/*  数据帧格式
typedef struct
{	
	u8 byte[4];		//float转化为4个字节数据
	u8 tail[4];	//帧尾
}Frame_TypeDef;
	byte 浮点数转换缓冲区	tail 帧尾
u8 byte[4]={0};		//float转化为4个字节数据
u8 tail[4]={0x00, 0x00, 0x80, 0x7f};	//帧尾
*/



void JustFloat_Test(void)	//justfloat 数据协议测试
{
    float a=1,b=2;	//发送的数据 两个通道
	
	u8 byte[4]={0};		//float转化为4个字节数据
	u8 tail[4]={0x00, 0x00, 0x80, 0x7f};	//帧尾
	
	//向上位机发送两个通道数据
	Float_to_Byte(a,byte);
	//u1_printf("%f\r\n",a);
	u1_SendArray(byte,4);	//1转化为4字节数据 就是  0x00 0x00 0x80 0x3F
	
	Float_to_Byte(b,byte);
	u1_SendArray(byte,4);	//2转换为4字节数据 就是  0x00 0x00 0x00 0x40 
	
	//发送帧尾
	u1_SendArray(tail,4);	//帧尾为 0x00 0x00 0x80 0x7f

}


















