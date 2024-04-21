#include "mymath.h"

/**
 * @brief 比较元素大小，作为qsort的参数
 * 
 * @param a 
 * @param b 
 * @return int 
 */
int compare(const void *a, const void *b)
{
    return (*(uint8 *)a - *(uint8 *)b);
}

/**
 * @brief 交换数组元素
 *
 * @param arr
 * @param index_i
 * @param index_j
 */
void swap(uint8 arr[], int index_i, int index_j)
{
    //将数组相应位置的两个数相交换
    uint8 k = arr[index_i];
    arr[index_i] = arr[index_j];
    arr[index_j] = k;
}

/**
 * @brief Glenn W. Rowe划分算法
 *
 * @param arr
 * @param low
 * @param high
 * @return uint8
 */
uint8 partition_Rowe(uint8 arr[], int low, int high)
{
    //根据一个基准数，将数组分为基准数左边小于基准数，基准数右边大于或等于基准数的两部分
    //返回值是基准数在数组中的下标
    //这里选取数组元素的第0位作为基准数
    // low为最低下标，high为最高下标
    uint8 pivot = arr[low]; //选取基准数
    uint8 low_index = low;
    for (int i = low + 1; i <= high; i++)
    {
        if (arr[i] < pivot)
        {
            //在序列中找到一个比pivot小的，就递增low_index
            low_index++;
            if (i != low_index) swap(arr, i, low_index); //如果i和low_index相等，则在i之前都不存在需要交换的比pivot大的数
        }
    }
    // low_index的位置就是pivot应处在的位置，low_index指向的总是比pivot小的数
    arr[low] = arr[low_index];
    arr[low_index] = pivot;
    return low_index;
}

/**
 * @brief 快速分类算法
 *
 * @param arr
 * @param low
 * @param high
 */
void quick_sort(uint8 arr[], int low, int high)
{
    if (high > low) //如果需要排序的序列的元素个数大于1
    {
        uint8 pivot_pos = partition_Rowe(arr, low, high);
        quick_sort(arr, low, pivot_pos - 1);  //左序列
        quick_sort(arr, pivot_pos + 1, high); //右序列
    }
}

/**
 * @brief 反正切函数的简化写法
 *
 * @param y
 * @param x
 * @return uint8
 */
uint8 Atan2(float y, float x)
{
    float tanNum;
    uint8 direction; //像素点的方向，值为Gy/Gx
    tanNum = y / x;
    // 0.41421356对应的是22.5°，2.41421356对应的是67.5°
    // 可以尝试0.57735026对应30°，1.73205080对应的是60°
    if (tanNum > -0.41421356 && tanNum < 0.41421356)    direction = 0; //水平方向
    else if (tanNum >= 0.41421356 && tanNum < 2.41421356)   direction = 1; //左下、右上
    else if (tanNum <= -0.41421356 && tanNum > -2.41421356)     direction = 2; //左上、右下
                    //  if ( tanNum> -0.57735026 && tanNum< 0.57735026 )  direction=0;//水平方向
                    //  else if( tanNum>= 0.57735026 && tanNum< 1.73205080)  direction=1;//左下、右上
                    //  else if( tanNum<= -0.57735026 && tanNum> -1.73205080)  direction=2;//左上、右下
    else    direction = 3; //竖直方向
    return direction;
}

/**
 * @brief 卡马尔开平方算法
 *
 * @param x
 * @return float
 */
float InvSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int *)&x;             // get bits for floating VALUE
    i = 0x5f3759df - (i >> 1);      // gives initial guess y0
    x = *(float *)&i;               // convert bits BACK to float
    x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
    return 1 / x;
}

