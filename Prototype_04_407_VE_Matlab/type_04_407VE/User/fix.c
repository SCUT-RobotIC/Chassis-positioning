/*************************************************
Copyright (C), Li_Jiang
File name: fix.c
Author: Li_Jiang
Version:               
Date: 
Description:  
Others:      
Function List:
History:
<author>    <time>          <version>       <desc>
**************************************************/
#include "fix.h"

/* ============ 全局变量声明 ============= */

/* ============ 局部变量声明 ============= */

/* ============ 内部函数声明 ============= */
static void fix_chack(struct fix_handle_typedef *fix_handle);
static int fix_compar(const void *p1, const void *p2);
static int fix_sort(struct fix_handle_typedef *fix_handle);

/* ============ 函数定义 ================= */

/**
  * @name   fix_chack()
  * @brief  检查修正点集是否单调，是否存在重复点（防止插值计算中出现除零的情况）
  * @call   Internal
  * @param  fix_handle  要检查的修正句柄
  * @RetVal void
  */
static void fix_chack(struct fix_handle_typedef *fix_handle)
{
	for(int i = 1; i < fix_handle->num_of_fix_point; i++)
	{
		if(fix_handle->fix_points[i-1].raw == fix_handle->fix_points[i].raw)    // 当前元素与上一个元素相同
		{
			int j, k;
			
			// 找到有多少重复元素
			for(j = 1; i + j < fix_handle->num_of_fix_point; j++)   
			{
				if(fix_handle->fix_points[i + j].raw != fix_handle->fix_points[i].raw) break;
			}
			
			// 依次前移，将重复值覆盖
			for(k = i; k + j < fix_handle->num_of_fix_point; k++)
			{
				fix_handle->fix_points[k] = fix_handle->fix_points[k + j];
			}
			
			fix_handle->num_of_fix_point = fix_handle->num_of_fix_point - j;    // 修改修正点个数
		}
	}
}/* fix_chack() */


// 实现一个对修正点集进行排序的功能
/**
  * @name   fix_compar()
  * @brief  用于在排序时进行比较
  * @call   Internal
  * @param  p1 第一个修正点
  * @param  p2 第二个修正点
  * @RetVal p1 - p2 的差值，可以从差值得到两者的大小关系
  */
static int fix_compar(const void *p1, const void *p2)
{
	return (((struct fix_point_s *)p1)->raw - ((struct fix_point_s *)p2)->raw);
}/* fix_compar() */



/**
  * @name   fix_sort
  * @brief  对修正点进行排序（升序）
  * @call   Internal
  * @param  fix_handle  要执行排序的修正句柄结构体
  * @RetVal 0
  */
static int fix_sort(struct fix_handle_typedef *fix_handle)
{
	qsort(fix_handle->fix_points, fix_handle->num_of_fix_point, sizeof(struct fix_point_s), fix_compar);
	
	return 0;
}/* fix_sort() */



/**
  * @name   fix_find_interval()
  * @brief  由于插值修正需要找到待修正数据所处的位置，故使用二分查找进行搜索，提高查找效率。使用指针的方式传回找到的区间端点
  * @call   Internal
  * @param  fix_handle  要操作的修正句柄
  * @param  raw         要寻找区间的原始值
  * @RetVal 运行情况    0代表运行正常，-1代表异常
  */
static int fix_find_interval(struct fix_handle_typedef *fix_handle, float raw)
{
	if(!fix_handle)
		return -1;
		
	// 特殊情况
	if(raw < fix_handle->fix_points[0].raw) // raw比最小参考点小
	{
		// 使用第一个和第二个参考点作为区间端点
		fix_handle->fix_param.inter.p1 = &fix_handle->fix_points[0];
		fix_handle->fix_param.inter.p2 = &fix_handle->fix_points[1];
		
		return 0;
	}
	else if(raw > fix_handle->fix_points[fix_handle->num_of_fix_point - 1].raw) // raw比最大参考点大
	{
		// 使用倒数第一个和倒数第二个参考点作为区间端点
		fix_handle->fix_param.inter.p1 = &fix_handle->fix_points[fix_handle->num_of_fix_point - 2];
		fix_handle->fix_param.inter.p2 = &fix_handle->fix_points[fix_handle->num_of_fix_point - 1];
		
		return 0;
	}
	else    // 正常情况
	{
		// 使用二分法进行查找
		int left, right, mid;
		left = 0;
		right = fix_handle->num_of_fix_point - 1;

		fix_handle->fix_param.inter.p1 = &fix_handle->fix_points[left];
		fix_handle->fix_param.inter.p2 = &fix_handle->fix_points[right];
		
		while((left - right) > 1 || (left - right) < -1)
		{
			mid = (left + right) >> 1;
			
			if(raw < fix_handle->fix_points[mid].raw)   // 在左半区
			{
				right = mid;
			}
			else    // 在右半区
			{
				left = mid;
			}
		}
		
		fix_handle->fix_param.inter.p1 = &fix_handle->fix_points[left];
		fix_handle->fix_param.inter.p2 = &fix_handle->fix_points[right];
	}
	
	return 0;
}/* fix_find_interval() */


/**
  * @name   fix_get_param()
  * @brief  获取修正的参数
  * @call   Internal
  * @param  fix_handle  要操作的修正句柄
  * @param  raw         要修正的原始值
  * @RetVal 运行情况    0代表运行正常，-1代表异常
  */
static int fix_get_param(struct fix_handle_typedef *fix_handle, float raw)
{
	if(!fix_handle)
		return -1;
		
	switch(fix_handle->fix_type)
	{
		case fix_type_interpolation:
			fix_find_interval(fix_handle, raw);
			fix_handle->fix_param.inter.k  = (fix_handle->fix_param.inter.p2->real - fix_handle->fix_param.inter.p1->real) / (fix_handle->fix_param.inter.p2->raw - fix_handle->fix_param.inter.p1->raw);
			fix_handle->fix_param.inter.b  = (fix_handle->fix_param.inter.p1->real - fix_handle->fix_param.inter.p1->raw * fix_handle->fix_param.inter.k);
			break;
		case fix_type_polynomial:
			if(fix_handle->fix_param.polynomial.cal_flag)
			{
				// 计算出多项式拟合的系数
			}
			break;
		default:
			break;
	}

	return 0;
}/* fix_get_param() */




/**
  * @name   fix_handle_init()
  * @brief  对修正结构体进行初始化，为其申请修正点的存储空间以及规定修正点个数，设置修正方式
  * @call   External
  * @param  fix_handle          要操作的修正句柄
  * @param  num_of_fix_point    修正点个数
  * @param  fix_type            修正方式
  * @RetVal 运行情况    0代表运行正常，-1代表异常
  */
int fix_handle_init(struct fix_handle_typedef *fix_handle, unsigned int num_of_fix_point, enum fix_type_e fix_type)
{
	if(!fix_handle)
		return -1;
		
	fix_handle->num_of_fix_point = num_of_fix_point;
	fix_handle->fix_type = fix_type;
	fix_handle->fix_points = (struct fix_point_s *)malloc(sizeof(struct fix_point_s) * num_of_fix_point);

	if(NULL == fix_handle->fix_points)
	{
		fix_handle->num_of_fix_point = 0;
		fix_handle->fix_type = fix_type_null;
		return -1;
	}
		
	return 0;
}/* fix_handle_init() */

/**
  * @name   fix_load()
  * @brief  为修正结构体装填修正点
  * @call   External
  * @param  fix_handle          要操作的修正句柄
  * @param  flash               要装填的修正点集所在的地址
  * @param  data_chack          用来检查修正点集的函数指针
  * @RetVal void
  */
void fix_load(struct fix_handle_typedef *fix_handle, void *flash, void (*data_chack)(struct fix_handle_typedef *))
{
	memcpy(fix_handle->fix_points, flash, sizeof(struct fix_point_s) * fix_handle->num_of_fix_point);

	fix_sort(fix_handle);

	// 可以添加对修正点合理性的检查
	// 可能出现的异常情况：相同的raw对应了不同的real
	if(data_chack)
	{
		data_chack(fix_handle);
	}
}/* fix_load() */

/**
  * @name   fix()
  * @brief  对输入原始值进行修正
  * @call   External
  * @param  fix_handle          要操作的修正句柄
  * @param  raw         要修正的原始值
  * @RetVal 修正之后的值
  */
float fix(struct fix_handle_typedef *fix_handle, float raw)
{
	if(!fix_handle)
		return raw;
		
	float real = raw;
	float product = raw;
	
	fix_get_param(fix_handle, raw);
	
	switch(fix_handle->fix_type)
	{
		case fix_type_polynomial:   // 多项式修正
			
			real  = fix_handle->fix_param.polynomial.a0;            // 常数项
			real += fix_handle->fix_param.polynomial.a1 * product;  // 一次项
			product *= raw;
			real += fix_handle->fix_param.polynomial.a2 * product;  // 二次项
			
			break;
			
		case fix_type_interpolation:    // 插值修正
			real = fix_handle->fix_param.inter.k * raw + fix_handle->fix_param.inter.b;
			break;
			
		case fix_type_null:
		default:
			real = raw;
			break;
	}
	
	return real;
}/* fix() */



