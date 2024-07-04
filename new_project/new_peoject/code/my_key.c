#include "my_key.h"

struct key keys[4];//定义结构体

/*按键扫描函数*/
void key_scan(void)//最好隔20ms进入一次中断
{
    /*读取对应IO口的状态*/
    keys[0].key_now_state=gpio_get_level(C12);
    keys[1].key_now_state=gpio_get_level(C13);
    keys[2].key_now_state=gpio_get_level(C14);
    keys[3].key_now_state=gpio_get_level(C15);
    for(uint8 i=0;i<4;i++)
    {
        switch(keys[i].key_short_state)
        {
            case 0:
            {
                if(keys[i].key_now_state==0)
                {
                    keys[i].key_short_state=1;
                }
            }break;
            case 1:
            {
                if(keys[i].key_now_state==0)
                {
                    keys[i].key_short_state=2;
                }
                else
                {
                    keys[i].key_short_state=0;
                }
            }break;
            case 2:
            {
                if(keys[i].key_now_state==1)
                {
                    keys[i].single_flag=1;
                    keys[i].key_short_state=0;//状态归0
                }
            }break;
        }
    }
}

/*按键处理函数*/
void my_key_handle(void)
{
    if(keys[0].single_flag==1)
    {
        Edge_threshold+=100;//阈值连接增加100
        keys[0].single_flag=0;//清除标志位
    }
    else if(keys[1].single_flag==1)
    {
        Edge_threshold-=100;//阈值连接减少100
        keys[1].single_flag=0;
    }
}
