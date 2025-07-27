/*************************************************
Copyright (C), Li_Jiang
File name: MA600.h
Author: Li_Jiang
Version:               
Date: 
Description:  
Others:      
Function List:
History:
<author>    <time>          <version>       <desc>
**************************************************/
#ifndef __MA600_h
#define __MA600_h

#include "stdint.h"
#include "spi.h"

/* =========== 宏定义 ================ */

//#define MA600_SPI_CS(phMA600, state)    

// === REGISTER MAP ===
#define MA600_Reg_Z_L                           0x00
#define MA600_Reg_Z_H                           0x01
#define MA600_Reg_BCT                           0x02
#define MA600_Reg_ETY_ETX                       0x03
#define MA600_Reg_PPT_ILIP                      0x04
#define MA600_Reg_PPT                           0x05
#define MA600_Reg_NPP                           0x07
#define MA600_Reg_PWMM_PWMF                     0x08
#define MA600_Reg_RD                            0x09
#define MA600_Reg_DAISY_RWM                     0x0A
#define MA600_Reg_ODx_SPULLIN_TRISTATE          0x0B
#define MA600_Reg_HYS                           0x0C
#define MA600_Reg_FW                            0x0D
#define MA600_Reg_INTF_SEL_DAZ_CK100            0x0E
#define MA600_Reg_MTOFFSET_L                    0x12
#define MA600_Reg_MTOFFSET_H                    0x13
#define MA600_Reg_NVMB_ERRx                     0x1A
#define MA600_Reg_MTSP_PRT_PRTS_APRT_FTA_FTM    0x1C
#define MA600_Reg_SUFFIXID                      0x1E
#define MA600_Reg_PRODUCTID                     0x1F
#define MA600_Reg_CORR0                         0x20
#define MA600_Reg_CORR1                         0x21
#define MA600_Reg_CORR2                         0x22
#define MA600_Reg_CORR3                         0x23
#define MA600_Reg_CORR4                         0x24
#define MA600_Reg_CORR5                         0x25
#define MA600_Reg_CORR6                         0x26
#define MA600_Reg_CORR7                         0x27
#define MA600_Reg_CORR8                         0x28
#define MA600_Reg_CORR9                         0x29
#define MA600_Reg_CORR10                        0x2A
#define MA600_Reg_CORR11                        0x2B
#define MA600_Reg_CORR12                        0x2C
#define MA600_Reg_CORR13                        0x2D
#define MA600_Reg_CORR14                        0x2E
#define MA600_Reg_CORR15                        0x2F
#define MA600_Reg_CORR16                        0x30
#define MA600_Reg_CORR17                        0x31
#define MA600_Reg_CORR18                        0x32
#define MA600_Reg_CORR19                        0x33
#define MA600_Reg_CORR20                        0x34
#define MA600_Reg_CORR21                        0x35
#define MA600_Reg_CORR22                        0x36
#define MA600_Reg_CORR23                        0x37
#define MA600_Reg_CORR24                        0x38
#define MA600_Reg_CORR25                        0x39
#define MA600_Reg_CORR26                        0x3A
#define MA600_Reg_CORR27                        0x3B
#define MA600_Reg_CORR28                        0x3C
#define MA600_Reg_CORR29                        0x3D
#define MA600_Reg_CORR30                        0x3E
#define MA600_Reg_CORR31                        0x3F
#define MA600_Reg_UR10                          0x84

// === REGISTER Bit Msk ===
#define MA600_Msk_ETY                           (0x01U << 1U)
#define MA600_Msk_ETX                           (0x01U << 0U)
#define MA600_Msk_ILIP                          (0x0FU << 1U)
#define MA600_Msk_NPP                           (0x07U << 5U)
#define MA600_Msk_PWMM                          (0x01U << 7U)
#define MA600_Msk_PWMF                          (0x01U << 6U)
#define MA600_Msk_RD                            (0x01U << 7U)
#define MA600_Msk_DAISY                         (0x01U << 7U)
#define MA600_Msk_RWM                           (0x01U << 0U)
#define MA600_Msk_OD615                         (0x01U << 7U)
#define MA600_Msk_OD243                         (0x01U << 6U)
#define MA600_Msk_SPULLIN                       (0x01U << 5U)
#define MA600_Msk_TRISTATE                      (0x01U << 5U)
#define MA600_Msk_FW                            (0x0FU << 0U)
#define MA600_Msk_INTF_SEL                      (0x07U << 5U)
#define MA600_Msk_DAZ                           (0x01U << 2U)
#define MA600_Msk_CK100                         (0x01U << 0U)
#define MA600_Msk_NVMB                          (0x01U << 7U)
#define MA600_Msk_ERRCRC                        (0x01U << 2U)
#define MA600_Msk_ERRMEM                        (0x01U << 1U)
#define MA600_Msk_ERRPAR                        (0x01U << 0U)
#define MA600_Msk_MTSP                          (0x01U << 7U)
#define MA600_Msk_PRT                           (0x01U << 5U)
#define MA600_Msk_PRTS                          (0x01U << 4U)
#define MA600_Msk_APRT                          (0x01U << 3U)
#define MA600_Msk_FTA                           (0x03U << 1U)
#define MA600_Msk_FTM                           (0x01U << 0U)
#define MA600_Msk_UR10                          (0x01U << oU)

// ==== IO Matrix Type====
/* | Type   | IO6   | IO1   | IO5   | IO2   | IO4   | IO3   
 * | Type0  |  U    |  V    |  W    |  A    |  B    |  Z
 * | Type1  |  U    |  V    |  W    |  SSD  |  SSCK |  PWM
 * | Type2  |  SSCK |  SSD  |  PWM  |  A    |  B    |  Z
 * | Type3  |  U    |  V    |  W    | /V    | /U    | /W
 * | Type4  | /B    | /A    | /Z    |  A    |  B    |  Z
 * | Type5  |  U    |  V    |  W    |  -    |  -    |  -
 * | Type6  |  -    |  -    |  -    |  A    |  B    |  Z
 * | Type7  |  SSCK |  SSD  |  PWM  |  -    |  -    |  -
 */ 
#define MA600_IOMatrix_Type0                    (0x00U)
#define MA600_IOMatrix_Type1                    (0x01U)
#define MA600_IOMatrix_Type2                    (0x02U)
#define MA600_IOMatrix_Type3                    (0x03U)
#define MA600_IOMatrix_Type4                    (0x04U)
#define MA600_IOMatrix_Type5                    (0x05U)
#define MA600_IOMatrix_Type6                    (0x06U)
#define MA600_IOMatrix_Type7                    (0x07U)

// ==== MTSP Type ====
#define MA600_MASP_MultiTurn                    (0U)
#define MA600_MASP_Speed                        (1U)

// ==== RotationDirection ==== Determines the sensor positive direction
#define MA600_RD_CW                             (0U)
#define MA600_RD_CCW                            (1U)

/* ====== 几个重要的延时 ======
 * t_IdleCommand            120 ns
 * t_StoreRegBlock          600 ms
 * t_RestoreAllRegBlocks    240 us
 */
#define MA600_t_IdleCommand                     120
#define MA600_t_StoreRegBlock                   600
#define MA600_t_RestoreAllRegBlocks             240

// ==== 工具性宏定义 ====
#define MA600_fun_MaskedAngle(_raw, _mask)      ((uint16_t)((_raw) & (_mask)))

typedef struct
{
    // SPI 句柄
    SPI_HandleTypeDef   *hspi;
    // 未来可以有软件SPI的句柄
    
    // SPI片选引脚
    GPIO_TypeDef        *spi_CS_GPIOx;
    uint32_t            spi_CS_Pin;
    
    // 记录的一些信息
    uint16_t             Angle;
    uint16_t             Angle_last;	
		uint16_t    			 	 total_angle;
		uint16_t    				 cirle;
		uint16_t    				 delta_dis;
    uint16_t             Angle_Masked;
    uint16_t             Angle_Masked_last;
    int32_t              MultiTurn;
    float                Speed;
    
    // 和精度有关的
    uint16_t             Mask;    // 1有效，0无效 例如0xFF00则代表高八位有效，低八位无效
}hMA600_TypeDef;

/* =========== 全局变量声明 ========== */
extern hMA600_TypeDef TestMA600[2];

/* =========== 函数声明 ============== */

//new add
void MA600_dataUpdate(hMA600_TypeDef *hMA600);


//original
void MA600_HandleInit(hMA600_TypeDef *hMA600, SPI_HandleTypeDef *hspi, GPIO_TypeDef *spi_CS_GPIOx, uint32_t spi_CS_Pin);

uint16_t MA600_Get_Angle(hMA600_TypeDef *hMA600);
uint8_t  MA600_Read_Reg(hMA600_TypeDef *hMA600, uint8_t RegAddr);
uint8_t  MA600_Write_Reg(hMA600_TypeDef *hMA600, uint8_t RegAddr, uint8_t Value);
uint16_t MA600_Store_Single(hMA600_TypeDef *hMA600, uint8_t BlockIndex);
uint16_t MA600_Restore_All(hMA600_TypeDef *hMA600);
uint16_t MA600_Clear_ErrFlags(hMA600_TypeDef *hMA600);

uint16_t MA600_SetMask(hMA600_TypeDef *hMA600, uint16_t Mask);
uint16_t MA600_SetValueBits(hMA600_TypeDef *hMA600, uint16_t bits);

uint16_t MA600_Read_PPT(hMA600_TypeDef *hMA600);
uint16_t MA600_Set_PPT(hMA600_TypeDef *hMA600, uint16_t NewPPTValue);

uint8_t MA600_Read_IOMatrix(hMA600_TypeDef *hMA600);
uint8_t MA600_Set_IOMatrix(hMA600_TypeDef *hMA600, uint8_t Type);

uint8_t MA600_Set_MTSP(hMA600_TypeDef *hMA600, uint8_t Type);
float MA600_Get_Speed_rpm(hMA600_TypeDef *hMA600);
int16_t MA600_Get_MultiTurn(hMA600_TypeDef *hMA600);

uint8_t MA600_Read_BCT(hMA600_TypeDef *hMA600);
uint8_t MA600_Set_BCT(hMA600_TypeDef *hMA600, uint8_t BCTValue);

uint8_t MA600_Read_ETYETX(hMA600_TypeDef *hMA600);
uint8_t MA600_Set_ETYETX(hMA600_TypeDef *hMA600, uint8_t ETY, uint8_t ETX);

uint8_t MA600_Read_FW(hMA600_TypeDef *hMA600);
uint8_t MA600_Set_FW(hMA600_TypeDef *hMA600, uint8_t FWValue);

uint8_t MA600_Read_RotationDirection(hMA600_TypeDef *hMA600);
uint8_t MA600_Set_RotationDirection(hMA600_TypeDef *hMA600, uint8_t RD);

uint16_t MA600_Read_Zero(hMA600_TypeDef *hMA600);
uint16_t MA600_Set_Zero(hMA600_TypeDef *hMA600, uint16_t Zero);


#endif /* __MA600_h */

