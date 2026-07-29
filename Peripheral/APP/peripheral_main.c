/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2020/08/06
 * Description        : 外设从机应用主函数及任务系统初始化
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "HAL.h"
#include "gattprofile.h"
#include "peripheral.h"

uint8_t trigB=7;
extern uint32_t adcBuff[40];
extern signed short RoughCalib_Value; // ADC粗调偏差值
// /* 记录当前的Image */
// unsigned char CurrImageFlag = 0xff;
// /* 用于APP判断文件有效性 */
// const uint32_t Address = 0xFFFFFFFF;

// __attribute__((aligned(4))) uint32_t Image_Flag __attribute__((section(".ImageFlag"))) = (uint32_t)&Address;

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
 void app_uart_process(void);
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
uint8_t MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x6E, 0x48};
#endif

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   主循环
 *
 * @return  none
 */
__HIGH_CODE
__attribute__((noinline))
void Main_Circulation()
{
    while(1)
    {
        TMOS_SystemProcess();
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main(void)
{
    uint8_t i;
#if(defined(DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);
#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
#endif
#ifdef DEBUG
    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    GPIOA_SetBits(bRXD1);
    GPIOA_ModeCfg(bRXD1, GPIO_ModeIN_PU);
    UART1_DefInit();
    UART1_ByteTrigCfg(UART_7BYTE_TRIG);
    trigB = 7;
    UART1_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    PFIC_EnableIRQ(UART1_IRQn);

    //LED状态灯
    GPIOA_SetBits(GPIO_Pin_11);//11
    GPIOA_SetBits(GPIO_Pin_10);//12
    GPIOA_ModeCfg(GPIO_Pin_10|GPIO_Pin_11, GPIO_ModeOut_PP_5mA);

    //采集灯初始化
    GPIOB_SetBits(GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_17|GPIO_Pin_16);//led灯初始化1
    GPIOB_ModeCfg(GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_17|GPIO_Pin_16, GPIO_ModeOut_PP_5mA);

    GPIOB_SetBits(GPIO_Pin_0);
    GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeOut_PP_5mA);

    //采样通道初始化
    GPIOA_ModeCfg(GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_12|GPIO_Pin_15, GPIO_ModeIN_Floating);
    // 采样率最高8M
    ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);
    RoughCalib_Value = ADC_DataCalib_Rough(); // 用于计算ADC内部偏差，记录到全局变量 RoughCalib_Value中
    ADC_ExcutSingleConver();//时间足够时建议再次转换并丢弃首次ADC数据

#endif
    //PRINT("%s\n", VER_LIB);
    CH58x_BLEInit();
    HAL_Init();
    GAPRole_PeripheralInit();
    Peripheral_Init();
    Main_Circulation();
}

/******************************** endfile @ main ******************************/
