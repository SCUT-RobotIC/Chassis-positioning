/*************************************************
Copyright (C), Li_Jiang
File name: fix.h
Author: Li_Jiang
Version:               
Date: 
Description:  
Others:      
Function List:
History:
<author>    <time>          <version>       <desc>
**************************************************/
#ifndef __fix_h
#define __fix_h

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/* =========== 宏定义 ================ */

struct fix_point_s
{
	float raw;
	float real;
};/* struct fix_point_s */

enum fix_type_e
{
	fix_type_null       		= 0,    // 不修正
	fix_type_interpolation		,       // 插值修正
	fix_type_polynomial         ,       // 多项式修正
};/* enum fix_type_e */

enum fix_err_e
{
	fix_err_OK                	= 0,    // 正常
	fix_err_CRC                 ,       // 修正点集CRC校验异常
	
};/* enum fix_err_e */

union fix_param_u
{
	/* k = (p2->real - p1->real) / (p1->raw - p1->raw)
	 * b = (p1->real - p1->raw * k)
	 * real = k * raw + b
	 */
	struct inter_s
	{
		struct fix_point_s *p1;
		struct fix_point_s *p2;
		
		float k;
		float b;
	}inter;
	
	/* a2,a1,a0 由参考点集得出，不需要每次修正时计算
	 * real = a2 * raw^2 + a1 * raw + a0 */
	struct polynomial_s
	{
		float a2;
		float a1;
		float a0;
		int   cal_flag;     // 当此标志非零时执行多项式系数的拟合，之后置零
	}polynomial;
};/* union fix_param_u */

struct fix_handle_typedef
{
	struct fix_point_s 	*fix_points;            // 修正点集指针
	unsigned int        num_of_fix_point;       // 修正点数量
	enum fix_type_e     fix_type;               // 修正方式
	union fix_param_u   fix_param;              // 修正系数
	enum fix_err_e      error_code;             // 异常代码
	
};/* struct fix_handle_typedef */

/* =========== 全局变量声明 ========== */

/* =========== 函数声明 ============== */

int fix_handle_init(struct fix_handle_typedef *fix_handle, unsigned int num_of_fix_point, enum fix_type_e fix_type);

void fix_load(struct fix_handle_typedef *fix_handle, void *flash, void (*data_chack)(struct fix_handle_typedef *));

float fix(struct fix_handle_typedef *fix_handle, float raw);

#endif /* __fix_h */

