/*************************************************
Copyright (C),  Li_Jiang
File name: MA600.c
Author: Li_Jiang
Version: 1.0.2 
Date: 2024年6月10日
Description:  
		* 若要使用此驱动库，需要将对应的SPI外设初始化为16bit模式！！！
		
		此驱动库为每个MA600芯片定义一个句柄（hMA600_TypeDef）
		在句柄中记录磁编所在的SPI外设句柄以及片选引脚信息
		同时在句柄中记录最新一次的单圈角度值、多圈圈数*以及旋转速度*（标记*的项目需要通过对磁编寄存器的设置来开启）
		
		每个句柄需要通过调用 MA600_HandleInit() 函数进行初始化，此后在使用时将会根据句柄内的内容来与芯片进行通信
		
		最基础的函数便是 MA600_Get_Angle() 通过调用此函数可以从磁编中读出最新的角度值，用时为16个SPI时钟周期，并且读回的角度值会同时存在对应的句柄中
		若要实现对旋转圈数的读取或是对当前速度的读取，可以通过调用MA600_Get_Speed_rpm() 或 MA600_Get_MultiTurn() 函数，前提是正确设置的芯片的MA600_Reg_MTSP_PRT_PRTS_APRT_FTA_FTM 寄存器
		
		进一步使用此驱动，可以通过调用 MA600_Read_Reg() 函数来读出指定寄存器的内容以及使用 MA600_Write_Reg() 函数来向指定寄存器写入数据
		两者结合便可以实现根据需要对磁编进行配置以更加符合实际使用需求。
		
		MA600磁编码器支持对寄存器内容实现非易失性存储，实现方式为在对寄存器的值执行完修改之后调用 MA600_Store_Single() 函数将寄存器的值存储到存储单元中，这时的值将会在下一次上电时自动加载
		若要手动将当前所有寄存器的值恢复为存储单元中的值，则可以调用 MA600_Restore_All() 函数实现
		
		
		
Others:      
Function List:
History:
<author>    <time>          <version>       <desc>
Li_Jiang    2023_12_23      v1.0.0          初版
Li_Jiang    2024_6_10       v1.0.2          侧位安装并验证BCT修正
Li_Jiang    2025_1_8        v1.0.3          添加旋转方向，零位的设置、读取
**************************************************/
#include "MA600.h"

/* ============ 全局变量声明 ============= */
hMA600_TypeDef TestMA600[2] = {0};

/* ============ 局部变量声明 ============= */

/* ============ 内部函数声明 ============= */
// 说明：这里使用定时器5来实现延时功能，移植时根据实际情况实现us和ms的延时
// 主频72MHz PSC：73 ARR：0xFFFF 增长方向：Up
// 相当于每1us CNT值加一

static void MA600_Delay_ns(uint32_t ns);
static void MA600_Delay_us(uint16_t us);
static void MA600_Delay_ms(uint16_t ms);

/* ============ 函数定义 ================= */
/* ===== 函数内部 ===== */
/**
  * @name   MA600_Delay_ns
  * @brief  MA600实现纳秒延时
  * @call   Internal
  * @param  ns  要延迟的时长（ns）
  * @RetVal NULL
  */
static void MA600_Delay_ns(uint32_t ns)
{
    ns *= 100;
    for( ; ns > 0; ns--);
}/* MA600_Delay_ns() */

/**
  * @name   MA600_Delay_us
  * @brief  MA600实现微秒延时
  * @call   Internal
  * @param  us  要延迟的时长（us）
  * @RetVal NULL
  */
static void MA600_Delay_us(uint16_t us)
{
//    UNUSED(us);
    
    uint16_t start = TIM6->CNT;
    uint16_t end;
    
    if(0xFFFF - start < us)
    {
        end = us - (0xFFFF - start);
        
        while(TIM6->CNT < end || TIM6->CNT < start);
    }
    else
    {
        end = start + us;
        
        while(TIM6->CNT < end);
    }
}/* MA600_Delay_us() */

/**
  * @name   MA600_Delay_ms
  * @brief  MA600实现毫秒延时
  * @call   Internal
  * @param  ms  要延迟的时长（ms）
  * @RetVal NULL
  */
static void MA600_Delay_ms(uint16_t ms)
{
    for( ; ms > 0; ms--)
    {
        MA600_Delay_us(1000);
    }
}/* MA600_Delay_ms() */




/* ===== 可调用函数 ===== */
/**
  * @name   MA600_HandleInit()
  * @brief  MA600 句柄初始化
  * @call   External
  * @param  hMA600          要初始化的句柄的指针
  * @param  hspi            要初始化的磁编所在的SPI总线的句柄指针
  * @param  spi_CS_GPIOx    磁编对应的片选引脚的端口
  * @param  spi_CS_Pin      磁编对应的片选引脚的引脚
  * @RetVal Return value
  */

void MA600_HandleInit(hMA600_TypeDef *hMA600, SPI_HandleTypeDef *hspi, GPIO_TypeDef *spi_CS_GPIOx, uint32_t spi_CS_Pin)
{
    hMA600->hspi            = hspi;
    hMA600->spi_CS_GPIOx    = spi_CS_GPIOx;
    hMA600->spi_CS_Pin      = spi_CS_Pin;
    
    hMA600->Angle           = 0;
		hMA600->total_angle = 0;
		hMA600->cirle = 0;
		hMA600->Angle_last = hMA600->Angle;
		hMA600->delta_dis = 0;
	
	
}/* MA600_HandleInit() */

/**
  * @name   MA600_dataUpdate()
  * @brief  更新角度与数值
  * @call   Internal or External
  * @param  hMA600          MA600句柄指针
  * @RetVal ...           ...
	* tips:new function
	*/

void MA600_dataUpdate(hMA600_TypeDef *hMA600)
{
	int diff=hMA600->Angle-hMA600->Angle_last;
	if (diff > 60000) {
		hMA600->cirle--;
		hMA600->total_angle = hMA600->Angle + hMA600->cirle * 65535;
		hMA600->delta_dis = diff - 65535;

	} else if (diff < -60000) {
		hMA600->cirle++;
		hMA600->total_angle = hMA600->Angle + hMA600->cirle * 65535;
		hMA600->delta_dis = diff + 65535;
		
	}else if (diff >=0) {
		hMA600->total_angle += diff;
		hMA600->delta_dis = diff;
	
	} else if (diff < 0) {
		hMA600->total_angle += diff;
		hMA600->delta_dis = diff;
	}	
	hMA600->Angle_last = hMA600->Angle;

}


/**
  * @name   MA600_Get_Angle()
  * @brief  读取当前角度
  * @call   Internal or External
  * @param  hMA600          MA600句柄指针
  * @RetVal Angle           当前角度
  */
uint16_t MA600_Get_Angle(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    uint16_t TxData = 0, RxData;
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    // 时钟线发送16个脉冲，读回16位角度数据
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    
    hMA600->Angle = RxData;
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return hMA600->Angle;
}/* MA600_Get_Angle() */

/**
  * @name   MA600_Read_Reg()
  * @brief  读取指定寄存器的值
  * @call   Internal or External
  * @param  hMA600          MA600句柄指针
  *         RegAddr         要读取的寄存器地址
  * @RetVal RegData         寄存器的值
  */
uint8_t MA600_Read_Reg(hMA600_TypeDef *hMA600, uint8_t RegAddr)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    uint16_t TxData = 0xD200, RxData = 0;
    int8_t   RegData = 0;
    
    TxData |= (RegAddr & 0xFF);
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    hMA600->Angle = RxData;
    TxData = 0;
    
    // 两次传输之间需要间隔t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    
    RegData = RxData & 0xFF;
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return RegData;
}/* MA600_Read_Reg() */

/**
  * @name   MA600_Write_Reg
  * @brief  向指定寄存器写入数据
  * @call   Internal or External
  * @param  hMA600          MA600句柄指针
  *         RegAddr         要写入的寄存器地址
  *         Value           要写入的值
  * @RetVal RegData         写入后寄存器内的数据
  */
uint8_t MA600_Write_Reg(hMA600_TypeDef *hMA600, uint8_t RegAddr, uint8_t Value)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    uint16_t TxData = 0xEA54, RxData = 0;
    int8_t   RegData = 0;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    hMA600->Angle = RxData;
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // 两次传输之间需要间隔t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = (RegAddr << 8) | (Value & 0xFF);
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // 两次传输之间需要间隔t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_Receive(hMA600->hspi, (uint8_t *)&RxData, 1, 0xFF);
    
    RegData = RxData & 0xFF;
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return RegData;
}/* MA600_Write_Reg() */

/**
  * @name   MA600_Store_Single()
  * @brief  存储单个寄存器块值到NVM中
  * @call   Internal or External
  * @param  hMA600          MA600句柄指针
  *         BlockIndex      要存储的寄存器块的编号
  * @RetVal Angle           当前角度
  */
uint16_t MA600_Store_Single(hMA600_TypeDef *hMA600, uint8_t BlockIndex)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    if(0 != BlockIndex && 1 != BlockIndex)
    {
        return -1;
    }
    
    uint16_t TxData = 0xEA55, RxData = 0;
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    hMA600->Angle = RxData;
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // 两次传输之间需要间隔t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0xEA00 | (0 == BlockIndex ? 0x0000: 0x0001);
    HAL_SPI_Transmit(hMA600->hspi, (uint8_t *)&TxData, 1, 0xFF);
    
     // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // 两次传输之间需要间隔t_StoreRegBlock
    MA600_Delay_ms(MA600_t_StoreRegBlock);
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0x0000;
    HAL_SPI_Transmit(hMA600->hspi, (uint8_t *)&TxData, 1, 0xFF);
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return hMA600->Angle;
}/* MA600_Store_Single() */

/**
  * @name   MA600_Restore_All()
  * @brief  将NVM中的所有值恢复到寄存器中
  * @call   Internal or External
  * @param  hMA600          MA600句柄指针
  * @RetVal Angle           当前角度
  */
uint16_t MA600_Restore_All(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    uint16_t TxData = 0, RxData = 0;
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0xEA56;
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    hMA600->Angle = RxData;
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // 两次传输之间需要间隔t_RestoreAllRegBlocks
    MA600_Delay_us(MA600_t_RestoreAllRegBlocks);
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0x0000;
    HAL_SPI_Transmit(hMA600->hspi, (uint8_t *)&TxData, 1, 0xFF);
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    
    return hMA600->Angle;
}/* MA600_Restore_All() */

/**
  * @name   MA600_Clear_ErrFlags()
  * @brief  清楚所有错误标志
  * @call   Internal or External
  * @param  hMA600          MA600句柄指针
  * @RetVal Angle           当前角度
  */
uint16_t MA600_Clear_ErrFlags(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    uint16_t TxData = 0, RxData = 0;
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0xD700;
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)&TxData, (uint8_t *)&RxData, 1, 0xFF);
    hMA600->Angle = RxData;
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    // 两次传输之间需要间隔t_IdleCommand
    MA600_Delay_ns(MA600_t_IdleCommand);
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    TxData = 0x0000;
    HAL_SPI_Transmit(hMA600->hspi, (uint8_t *)&TxData, 1, 0xFF);
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    return hMA600->Angle;
}/* MA600_Clear_ErrFlags() */


/**
  * @name   MA600_SetMask()
  * @brief  设置读取角度时使用的遮罩
  * @call   External
  * @param  hMA600          MA600句柄指针
  * @param  Mask            遮罩值
  * @RetVal hMA600->Mask    当前有效的mask
  */
uint16_t MA600_SetMask(hMA600_TypeDef *hMA600, uint16_t Mask)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    hMA600->Mask = Mask;
    
    return hMA600->Mask;

}/* MA600_SetMask() */


/**
  * @name   MA600_SetValueBits()
  * @brief  设置读取角度时使用的遮罩(通过精度）
  * @call   External
  * @param  hMA600          MA600句柄指针
  * @param  bits            有效位精度
  * @RetVal hMA600->Mask    当前有效的mask
  */
uint16_t MA600_SetValueBits(hMA600_TypeDef *hMA600, uint16_t bits)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    hMA600->Mask = 0xFFFF << (16 - bits);
    
    return hMA600->Mask;
}/* MA600_SetValueBits() */


// 所有的寄存器设置都要遵循    读出->修改->写入->检查 的步骤


/**
  * @name   MA600_Read_PPT()
  * @brief  读取指定磁编的每周期脉冲数（编码器输出）
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @RetVal 指定磁编的每周期脉冲数
  */
uint16_t MA600_Read_PPT(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue[2] = {0};
    uint16_t PPTValue = 0;
    
    RegValue[0] = MA600_Read_Reg(hMA600, MA600_Reg_PPT_ILIP);
    RegValue[1] = MA600_Read_Reg(hMA600, MA600_Reg_PPT);
    
    PPTValue = (RegValue[1] << 3) | ((RegValue[0] >> 5) & 0x07) | ((RegValue[0] << 11) & 0x0800);
    PPTValue = PPTValue + 1;
    
    return PPTValue;
}/* MA600_Read_PPT() */

/**
  * @name   MA600_Set_PPT()
  * @brief  设置指定磁编的每周期脉冲数（编码器输出）
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @param  NewPPTValue 要设置的每周期脉冲数
  * @RetVal 指定磁编的每周期脉冲数
  */
uint16_t MA600_Set_PPT(hMA600_TypeDef *hMA600, uint16_t NewPPTValue)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue[2] = {0};
    uint8_t TxData[2];
    
    
    // 读出当前寄存器中的值
    RegValue[0] = MA600_Read_Reg(hMA600, MA600_Reg_PPT_ILIP);
    RegValue[1] = MA600_Read_Reg(hMA600, MA600_Reg_PPT);
    
    // 生成新的值
    NewPPTValue = NewPPTValue - 1;
    TxData[0] = RegValue[0] & MA600_Msk_ILIP;
    TxData[0] |= ((NewPPTValue >> 11U) & 0x01);
    TxData[0] |= ((NewPPTValue << 5U)  & 0xE0);
    TxData[1] = ((NewPPTValue >> 3U) & 0xFF);
    
    // 将新值写回到寄存器中
    RegValue[0] = MA600_Write_Reg(hMA600, MA600_Reg_PPT_ILIP, TxData[0]);
    RegValue[1] = MA600_Write_Reg(hMA600, MA600_Reg_PPT, TxData[1]);
    
    // 判断写入是否成功
    if(RegValue[0] == TxData[0] && RegValue[1] == TxData[1])
    {
        // 成功
        return NewPPTValue + 1;
    }
    else
    {
        return -1;
    }
}/* MA600_Set_PPT() */

/**
  * @name   MA600_Read_IOMatrix()
  * @brief  读取指定磁编的IO功能选项
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @RetVal 指定磁编的IO功能选项     @ IO Matrix Type
  */
uint8_t MA600_Read_IOMatrix(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t INFT_SEL_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_INTF_SEL_DAZ_CK100);
    
    INFT_SEL_Value = (RegValue & MA600_Msk_INTF_SEL) >> 5U;
    
    return INFT_SEL_Value;
}

/**
  * @name   MA600_Set_IOMatrix()
  * @brief  设置指定磁编的IO功能选项
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @param  Type        IO功能选项  @ IO Matrix Type
  * @RetVal 指定磁编的IO功能选项
  */
uint8_t MA600_Set_IOMatrix(hMA600_TypeDef *hMA600, uint8_t Type)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_INTF_SEL_DAZ_CK100);
    
    TxData = RegValue & ~MA600_Msk_INTF_SEL;
    
    TxData |= ((Type << 5U) & MA600_Msk_INTF_SEL);
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_INTF_SEL_DAZ_CK100, TxData);
    
    // 返回当前寄存器内的设置情况
    return (RegValue & MA600_Msk_INTF_SEL) >> 5U;
}/* MA600_Set_IOMatrix() */

/**
  * @name   MA600_Set_MTSP()
  * @brief  设置指定磁编的返回值类型（多圈圈数或旋转速度）
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @param  Type        返回值类型（多圈圈数或旋转速度）  @ MTSP Type
  * @RetVal 指定磁编的返回值类型      @ MTSP Type
  */
uint8_t MA600_Set_MTSP(hMA600_TypeDef *hMA600, uint8_t Type)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_MTSP_PRT_PRTS_APRT_FTA_FTM);
    
    TxData = RegValue & ~MA600_Msk_MTSP;
    Type &= 0x01U;
    TxData = TxData | (Type << 7U);
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_MTSP_PRT_PRTS_APRT_FTA_FTM, TxData);
    
    Type = (RegValue & MA600_Msk_MTSP) >> 7U;
    
    // 返回当前寄存器内的设置情况
    return Type;
}/* MA600_Set_MTSP() */

/**
  * @name   MA600_Get_Speed_rpm()
  * @brief  读取指定磁编测量的转速（rpm）
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @RetVal Omega_rpm   指定磁编测量的转速（rpm）
  */
float MA600_Get_Speed_rpm(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    
    uint16_t TxData[2] = {0};
    uint16_t RxData[2] = {0};
    float    Omega_rpm = 0.0f;
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)TxData, (uint8_t *)RxData, 2, 0xFF);
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    hMA600->Angle = RxData[0];
    Omega_rpm     = (int16_t)RxData[1] * 5.722;
    hMA600->Speed = Omega_rpm;
    
    // 如果不加滤波，这里得到的速度信息在零速时会有漂移值，原因估计是安装间距不合适，从角度值上看有正负2或3的跳动
    // 高速情况还没有测试，目前测试架上的3508屁股没有电调，转不起来
    
    return Omega_rpm;
}/* MA600_Get_Speed_rpm() */


/**
  * @name   MA600_Get_MultiTurn()
  * @brief  读取指定磁编测量的多圈圈数
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @RetVal MultiTurn   指定磁编测量的多圈圈数
  */
int16_t MA600_Get_MultiTurn(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return 0;
    }
    
    uint16_t TxData[2] = {0};
    uint16_t RxData[2] = {0};
    int16_t  MultiTurn = 0.0f;
    
    // 片选使能
    hMA600->spi_CS_GPIOx->ODR &= ~hMA600->spi_CS_Pin;
    
    HAL_SPI_TransmitReceive(hMA600->hspi, (uint8_t *)TxData, (uint8_t *)RxData, 2, 0xFF);
    
    // 片选失能
    hMA600->spi_CS_GPIOx->ODR |= hMA600->spi_CS_Pin;
    
    hMA600->Angle       = RxData[0];
    MultiTurn           = (int16_t)RxData[1];
    hMA600->MultiTurn   = MultiTurn;
    
    return MultiTurn;
}/* MA600_Get_MultiTurn() */

/**
  * @name   MA600_Read_BCT()
  * @brief  读取指定磁编的BCT修正值
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @RetVal Omega_rpm   指定磁编的BCT修正值
  */
uint8_t MA600_Read_BCT(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t BCT_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_BCT);
    
    BCT_Value = RegValue;
    
    return BCT_Value;
}/* MA600_Read_BCT() */

/**
  * @name   MA600_Set_BCT()
  * @brief  设置指定磁编的BCT修正值
  * @call   External
  * @param  hMA600      要操作的磁编的句柄指针
  * @param  BCTValue    要设置的BCT值
  * @RetVal Omega_rpm   指定磁编的BCT修正值
  */
uint8_t MA600_Set_BCT(hMA600_TypeDef *hMA600, uint8_t BCTValue)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
//    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_BCT);
    
    TxData = BCTValue;
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_BCT, TxData);
    
    // 返回当前寄存器内的设置情况
    return RegValue;
}/* MA600_Set_BCT() */

/**
  * @name   MA600_Read_ETYETX()
  * @brief  读取指定磁编的通道衰减使能情况
  * @call   External
  * @param  hMA600  要操作的磁编的句柄指针
  * @RetVal 指定磁编的通道衰减使能情况
  */
uint8_t MA600_Read_ETYETX(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t ETYETX_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_ETY_ETX);
    
    ETYETX_Value = RegValue & (MA600_Msk_ETY | MA600_Msk_ETX);
    
    return ETYETX_Value;
}/* MA600_Read_ETYETX() */

/**
  * @name   MA600_Set_ETYETX()
  * @brief  设置指定磁编的通道衰减使能情况
  * @call   External
  * @param  hMA600  要操作的磁编的句柄指针
  * @param  ETY     Y轴传感器的衰减使能
  * @param  ETX     X轴传感器的衰减使能
  * @RetVal 指定磁编的通道衰减使能情况
  */
uint8_t MA600_Set_ETYETX(hMA600_TypeDef *hMA600, uint8_t ETY, uint8_t ETX)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_ETY_ETX);
    
    TxData = RegValue & ~(MA600_Msk_ETY | MA600_Msk_ETX);
    TxData |= (0 == ETY) ? 0: MA600_Msk_ETY;
    TxData |= (0 == ETX) ? 0: MA600_Msk_ETX;
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_ETY_ETX, TxData);
    
    // 返回当前寄存器内的设置情况
    return RegValue;
}/* MA600_Set_ETYETX() */

/**
  * @name   MA600_Read_FW()
  * @brief  读取指定磁编的滤波窗口值
  * @call   External
  * @param  hMA600  要操作的磁编的句柄指针
  * @RetVal 指定磁编的滤波窗口值
  */
uint8_t MA600_Read_FW(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t FW_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_FW);
    
    FW_Value = RegValue & MA600_Msk_FW;
    
    return FW_Value;
}/* MA600_Read_FW() */

/**
  * @name   MA600_Set_FW()
  * @brief  设置指定磁编的滤波窗口值
  * @call   External
  * @param  hMA600  要操作的磁编的句柄指针
  * @param  FWValue 要设置的滤波窗口值
  * @RetVal 指定磁编的滤波窗口值
  */
uint8_t MA600_Set_FW(hMA600_TypeDef *hMA600, uint8_t FWValue)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_FW);
    
    TxData = RegValue & ~MA600_Msk_FW;
    TxData |= FWValue;
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_FW, TxData);
    
    // 返回当前寄存器内的设置情况
    return RegValue;
}/* MA600_Set_FW() */


/**
  * @name   MA600_Read_RotationDirection()
  * @brief  读取指定磁编的旋转正方向(数值增长方向)
  * @call   External
  * @param  hMA600  要操作的磁编的句柄指针
  * @RetVal RD_Value    旋转方向(0 - CW | 1 - CCW)
  */
uint8_t MA600_Read_RotationDirection(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t RD_Value = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_RD);
    
    RD_Value = (RegValue & MA600_Msk_RD) >> 7U;
    
    return RD_Value;
}/* MA600_Read_RotationDirection() */

/**
  * @name   MA600_Set_RotationDirection()
  * @brief  设置指定磁编的旋转正方向(数值增长方向)
  * @call   External
  * @param  hMA600  要操作的磁编的句柄指针
  * @param  RD      要设置的旋转正方向[MA600_RD_CW || MA600_RD_CW]
  * @RetVal 指定磁编的旋转正方向  (0 - CW | 1 - CCW)
  */
uint8_t MA600_Set_RotationDirection(hMA600_TypeDef *hMA600, uint8_t RD)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_RD);
    
    TxData = RegValue & ~MA600_Msk_RD;
    TxData |= (RD << 7U);
    
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_RD, TxData);
    
    RegValue = (RegValue & MA600_Msk_RD) >> 7U;
    
    // 返回当前寄存器内的设置情况
    return RegValue;
}/* MA600_Set_RotationDirection() */

/**
  * @name   MA600_Read_Zero()
  * @brief  读取指定磁编的零点位置
  * @call   External
  * @param  hMA600  要操作的磁编的句柄指针
  * @RetVal ZeroValue   
  */
uint16_t MA600_Read_Zero(hMA600_TypeDef *hMA600)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t  RegValue = 0;
    uint16_t ZeroValue = 0;
    
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_Z_H);
    ZeroValue = RegValue;
    ZeroValue = ZeroValue << 8;
    RegValue = MA600_Read_Reg(hMA600, MA600_Reg_Z_L);
    ZeroValue |= RegValue;
    
    return ZeroValue;
}/* MA600_Read_Zero() */

/**
  * @name   MA600_Set_Zero()
  * @brief  设置指定磁编的零点位置
  * @call   External
  * @param  hMA600  要操作的磁编的句柄指针
  * @param  Zero    要设置的零点位置
  * @RetVal 指定磁编的零点位置
  */
uint16_t MA600_Set_Zero(hMA600_TypeDef *hMA600, uint16_t Zero)
{
    if(NULL == hMA600)
    {
        return -1;
    }
    uint8_t RegValue = 0;
    uint8_t TxData = 0;
    uint16_t ZeroValue = 0;
    
    // 高8位
    TxData = (Zero >> 8) & 0xFFU;
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_Z_H, TxData);
    ZeroValue = RegValue;
    ZeroValue = ZeroValue << 8;
    
    // 低8位
    TxData = Zero & 0xFFU;
    RegValue = MA600_Write_Reg(hMA600, MA600_Reg_Z_L, TxData);
    ZeroValue |= RegValue;
    
    // 返回当前寄存器内的设置情况
    return ZeroValue;
}/* MA600_Set_RotationDirection() */


