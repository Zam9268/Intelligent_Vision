#ifndef __EFFICIENT_H
#define __EFFICIENT_H

#include "stdio.h"
#include "math.h"
#include "zf_common_typedef.h"

extern int compare(const void *a, const void *b);
extern float InvSqrt(float x);
extern void swap(uint8 arr[], int index_i, int index_j);
extern void quick_sort(uint8 arr[], int low, int high);
extern uint8 partition_Rowe(uint8 arr[], int low, int high);
extern uint8 Atan2(float y, float x);

#endif
