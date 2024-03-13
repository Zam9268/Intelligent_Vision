#include "image.h"

uint8 Image_Use[IMAGE_HEIGHT][IMAGE_WIDTH];


/*���������������*/
uint8 left_line[IMAGE_HEIGHT],right_line[IMAGE_HEIGHT];//���ұ�������
int center[IMAGE_HEIGHT];//��������
uint8 the_maxlen_position;//�����λ??
uint8 num;//������Ч����
uint8 Longest_White_Column_Left[2];//����������У����г��Ⱥ�����������
uint8 Longest_White_Column_Right[2];//���ҵ������??
uint8 Right_Lost_Flag[IMAGE_WIDTH];//�ұ߽綪�߱�??
uint8 Left_Lost_Flag[IMAGE_WIDTH];//��߽綪�߱�??
uint8 Left_Lost_Time=0;//����������
uint8 Right_Lost_Time=0;//�����ұ߽綪����
uint8 Both_Lost_Time=0;//���������߽�ͬʱ����??
uint8 Search_Stop_Line;//������ֹ??
uint8 Boundry_Start_Left,Boundry_Start_Right;//���ұ߽���ʼ??
uint8 Road_Wide[IMAGE_HEIGHT];//���������������
RoadType Road_Type;//������������
float Left_derivative[IMAGE_HEIGHT]={0.0};
float Right_derivative[IMAGE_HEIGHT]={0.0};

//��Ȩ�������飬�������ø�����������ǰհ
const uint8 Weight[IMAGE_HEIGHT]=
{
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // ͼ����Զ��00 ��??09 ��Ȩ??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // ͼ����Զ��10 ��??19 ��Ȩ??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // ͼ����Զ��20 ��??29 ��Ȩ??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,        // ͼ����Զ��30 ��??39 ��Ȩ??
    1, 1, 1, 1, 1, 1, 1, 3, 4, 5,        // ͼ����Զ��40 ��??49 ��Ȩ??
    6, 7, 9, 11, 13, 15, 17, 19, 20, 20, // ͼ����Զ��50 ��??59 ��Ȩ??
    19, 17, 15, 13, 11, 9, 7, 5, 3, 1,   // ͼ����Զ��60 ��??69 ��Ȩ??
};

/**
 * @brief ���鸳ֵ
 * @param ��
 * @return ��
 */
void Image_Change(void)
{
    for (int i = 0; i < IMAGE_HEIGHT; i++)
    {
        for (int j = 0; j < IMAGE_WIDTH; j++)
        {
            Image_Use[i][j] = mt9v03x_image[i][j];
        }
    }
}
volatile int White_Column[IMAGE_WIDTH];//ÿ�а��г���
/**
 * @brief ���ߴ��������õĴ�򷨵�����з�����sobel,canny������з�����Ҫ���䣩
 * @param H���������ȣ�
 * @return ??
 */
void Center_line_deal(uint8 start_column,uint8 end_column)
{
    for(uint8 i=0;i<IMAGE_HEIGHT-1;i++)
	{
		left_line[i]=0;
		right_line[i]=0;
	}
    int x=0,y=0;//��xΪ��,yΪ��
    uint8 middle=the_maxlen_position;//���������λ??
    uint8 x_num;
    /*Ѱ������У����Ż��� */
    for(uint8 j=start_column;j<end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-1;i>0;i--)
        {
            if(Image_Use[i][j]==BLACK_POINT)
            {
                break;
            }
            else
            {
                White_Column[j]++;
            }
        }
    }
    /*������Ѱ�����??*/
    Longest_White_Column_Left[0]=0;//���г�������
    for(uint8 i=start_column;i<end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//���ֵ��??
        {
            Longest_White_Column_Left[0]=White_Column[i];
            Longest_White_Column_Left[1]=i;
        }
    }
    /*���ҵ���Ѱ�����??*/
    Longest_White_Column_Right[0]=0;//���г�������
    for(uint8 i=end_column;i>start_column;i--)
    {
        if(White_Column[i]>Longest_White_Column_Right[0])//���ֵ��??
        {
            Longest_White_Column_Right[0]=White_Column[i];
            Longest_White_Column_Right[1]=i;//��еĳ��Ⱥ�����������??
        }
    }
    /*��ֹ�и�??*/
    Search_Stop_Line=Longest_White_Column_Left[0];//������ֹ�еĸ�??
    int right_border,left_border;//����߽��м����
    for(int i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        /*��Ѱ�ұ�??*/
        for(int j=Longest_White_Column_Left[1];j<=Longest_White_Column_Right[1];j++)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j+1]==BLACK_POINT&&Image_Use[i][j+2]==BLACK_POINT)
            {
                right_border=j;//��¼��ǰ�߽�??
                Right_Lost_Flag[i]=0;//û�ж��ߣ���??0
                break;
            }
            else if(j>=IMAGE_HEIGHT-1-2)//û���ҵ��ұ߽�ʱ���Ͱ����ұ߽縳ֵ���ұߣ�Ȼ���߱�־λ??1
            {
                right_border=j;
                Right_Lost_Flag[i]=1;
                break;
            }
        }
        for(uint8 j=Longest_White_Column_Left[1];j>=2;j--)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j-1]==BLACK_POINT&&Image_Use[i][j-2]==BLACK_POINT)
            {
                left_border=j;//��¼��ǰ�߽�??
                Left_Lost_Flag[i]=0;//û�ж��ߣ���??0
                break;
            }
            else if(j<=2)//û���ҵ���߽�ʱ���Ͱ�����߽縳ֵ����ߣ�Ȼ���߱�־λ??1
            {
                left_border=j;
                Left_Lost_Flag[i]=1;
                break;
            }
        }
        left_line[i]=left_border;//��ֵ����߽���??
        right_line[i]=right_border;//��ֵ���ұ߽���??
    }
}

/**
 * @brief ���ߴ�����Ե���������Ѳ�ߣ��Լ�д�ģ�������bug����Ҫȥ��һЩ��??
 * @param H���������ȣ�
 * @return ??
 */
void Center_line_deal_plus(uint8 start_column,uint8 end_column)
{
    /*�߽���������*/
    for(uint8 i=0;i<IMAGE_HEIGHT;i++)
    {
        left_line[i]=0;
        right_line[i]=0;
    }
    /*����м�??*/
    for(uint8 j=start_column;j<end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-1;i>0;i--)
        {
            if(Image_Use[i][j]==BLACK_POINT)
            {
                White_Column[j]++;
            }//��������ɫ��Ե��ʱ����˳���??
            else    break;
        }
    }
    /*������Ѱ�����??*/
    Longest_White_Column_Left[0]=0;//���г�������
    for(uint8 i=start_column;i<end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//���ֵ��??
        {
            Longest_White_Column_Left[0]=White_Column[i];//��Ӧ�İ��е����??
            Longest_White_Column_Left[1]=i;//��Ӧ����е�����
        }
    }
    /*�������鸳??*/
    int right_border,left_border;//����߽�������ֵ�м��??
    for(int i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        for(int j=Longest_White_Column_Left[1];j>=2;j--)//�����ҿ�ʼɨ??
        {
            if(Image_Use[i][j]==BLACK_POINT&&Image_Use[i][j-1]==WHITE_POINT&&Image_Use[i][j-2]==WHITE_POINT)
            {
                left_border=j;//��¼��ǰ�߽�??
                Left_Lost_Flag[i]=0;//û�ж��ߣ���??0
                break;
            }
            else if(j<=2)//�����������ʱ��û��������ɫ������㣬�ͻ�Ĭ���Ҳ�����??
            {
                left_border=j;//��¼��ǰ�߽����??
                Left_Lost_Flag[i]=1;//�����ˣ��ͻ�ֱ��??0
                break;
            }
        }
        for(int j=Longest_White_Column_Left[1];j<=IMAGE_WIDTH-3;j++)
        {
            if(Image_Use[i][j]==BLACK_POINT&&Image_Use[i][j+1]==WHITE_POINT&&Image_Use[i][j+2]==WHITE_POINT)
            {
                right_border=j;//��¼��ǰ�߽�??
                Right_Lost_Flag[i]=0;//û�ж��ߣ���??0
                break;
            }
            else if(j>=IMAGE_HEIGHT-1-2)//��������ҵ��ʱ��û��������ɫ������㣬�ͻ�Ĭ���Ҳ�����??
            {
                right_border=j;//��¼��ǰ�߽����??
                Right_Lost_Flag[i]=1;//�����ˣ��ͻ�ֱ��??0
                break;
            }
        }
        left_line[i]=left_border;//��ֵ����߽���??
        right_line[i]=right_border;//��ֵ���ұ߽���??
    }
}
/**
 * @brief ����������������ڻ������͸�ӣ�Ŀǰֻ����ȡԭʼ�ı�������??
 * @param ??
 * @return ??
 */
void Outer_Analyse(void)
{
    /*ʣ����������*/
    for(uint8 i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        if(Left_Lost_Flag[i]==1)    Left_Lost_Time++;
        if(Right_Lost_Flag[i]==1)   Right_Lost_Time++;
        if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==1)   Both_Lost_Time++;
        if(Boundry_Start_Left==0&&Left_Lost_Flag[i]==0)   Boundry_Start_Left=i;//��¼��һ??
        if(Boundry_Start_Right==0&&Right_Lost_Flag[i]==0) Boundry_Start_Right=i;//��¼��һ??
        Road_Wide[i]=right_line[i]-left_line[i];//�����������
    }
    /*���������ж�*/
    if(Left_Lost_Time<=15&&Right_Lost_Time<=15&&Both_Lost_Time<=15) Road_Type=STRAIGHT_ROAD;
    if(Left_Lost_Time<15&&Right_Lost_Time>=30&&Both_Lost_Time<15)   Road_Type=RIGHT_TURN;
    if(Right_Lost_Time<15&&Left_Lost_Time>=30&&Both_Lost_Time<15)   Road_Type=LEFT_TURN;
    if(Right_Lost_Time>=30&&Left_Lost_Time>=30&&Both_Lost_Time>=30) Road_Type=CROSSING;

}

/**
 * @brief �����Լ��
 * @param height:��ֹ����
 * @return ��������������������ֵ
 */
uint8 Continuity_detect(uint8 *line)
{
    uint8 max_uncontinuity=0;//������ֵ
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]-line[i-1]>max_uncontinuity)
        {
            max_uncontinuity=line[i]-line[i-1];
        }
    }
    return max_uncontinuity;//��������������������ֵ
}

void Derivative_Change(void)
{
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        Left_derivative[i]=(left_line[i]-left_line[i-1])/2;
        Right_derivative[i]=(right_line[i]-right_line[i-1])/2;//������Եĵ���ֵ
    }
}

float Derivative_detect_max(uint8 *line)
{
    float max_derivative=0.00;
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]>max_derivative)
        {
            max_derivative=line[i];
        }
    }
    return max_derivative;
}

float Derivative_detect_min(uint8 *line)
{
    float min_derivative=0.00;
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]<min_derivative)
        {
            min_derivative=line[i];
        }
    }
    return min_derivative;
}
/**
 * @brief ������
 * @param height:��ֹ����
 * @return ���
 */
float Err_Handle(uint8 height)
{
    /*����д����ʹ��ǰհ����Զ��������
    float err=0.00;//ֵΪ��������ƫ��ֵΪ��������ƫ
    int sum_err[IMAGE_HEIGHT]={0};
    int sum_hight=0;
    for(uint8 i=IMAGE_HEIGHT-1;i>=height;i--)
    {
        sum_err[i]=(right_line[i]+left_line[i])/2-94;
        sum_hight+=i;
    }
    for(uint8 i=IMAGE_HEIGHT-1;i>=height;i--)
    {
        err+=sum_err[i]/sum_hight;//��ֵ��??
    }
    */
    
    float err=0.00;
    int weight_count=0;//Ȩ����??
    for(int i=IMAGE_HEIGHT-1;i>IMAGE_HEIGHT/2;i--)
    {
        err+=(IMAGE_WIDTH/2-((left_line[i]+right_line[i])>>1))*Weight[i];
        weight_count+=Weight[i];//Ȩ����ֵ��??
    }
    err=err/weight_count;//Ȩ�ؾ�??
    return err;
}

/**
 * @brief ���Ժ������Ѳ��ԵĶ������
 * @param ??
 * @return ??
 */
//void test(void)
//{
//   Image_Change();
//	ips114_show_char(188,120,'Q');
//	ips114_displayimage03x(*Image_Use,188,120);
//}
void test(void)
{
    Image_Change();
	uint8 *output_address;//�����ַָ��
    
    output_address=Scharr_Edge(*mt9v03x_image);
    memcpy(Image_Use,output_address,IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(uint8));
	uint8 threshold=Camera_GetOSTU((uint8 *)mt9v03x_image);

    Simple_Binaryzation(*Image_Use,threshold);
    Center_line_deal(10,178);//����Ѱ����
//`	
//    for(uint8 i=0;i<=IMAGE_HEIGHT-1;i++)
//    {
//        ips114_draw_point((left_line[i]+right_line[i])/2,i,RGB565_BLUE);
//        
//    }
    // ips114_show_uint(188,120,Longest_White_Column_Left[1],3);      
	ips114_displayimage03x(*Image_Use,188,120);
}
