/********************************** (C) COPYRIGHT *******************************
 * File Name          : peripheral.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : 外设从机多连接应用程序，初始化广播连接参数，然后广播，连接主机后，
 *                      请求更新连接参数，通过自定义服务传输数据
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "devinfoservice.h"
#include "gattprofile.h"
#include "peripheral.h"
#include <stdio.h>
#include <string.h>
#include <CH58x_sys.h>
/*********************************************************************
 * MACROS
 */
extern uint8_t MacAddr[6];
uint16_t write_flag;
extern uint8 addrmac[6];
uint8 SN[16];
uint8 addrmac_str1[32];
void hex_to_asciistring(u8* str,u32 size,u8* str1);
uint8_t ble_data[8]={0x01};
signed short RoughCalib_Value = 0; // ADC粗调偏差值
float dif=0;
uint8_t LED_flag=0;
uint8_t charValue1[SIMPLEPROFILE_CHAR1_LEN];
uint8_t charValue2[SIMPLEPROFILE_CHAR2_LEN]={0};
uint8_t newValue[15]={0};

uint32_t adcBuff[40];
uint32_t sum[4];
uint8_t CBT_Flag=0;
uint8_t LED_shan=0;
extern uint8_t trigB;

//uint16_t Time_Flag=0x0258;//10分钟
uint8_t Redled_flag;
/********************修改默认检测时间*********/
uint8_t What_time_flag=0x0F;
uint16_t What_time=0x0384;
/*******************************************/
uint8_t Result_5min[3];
uint8_t Result_10min[3];
uint8_t Result_15min[3];

void AD_Run(void);//ad采集函数
void LED(uint8_t LED_flag);
volatile uint8_t adclen;
volatile uint8_t DMA_end = 0;

void bubbleSort( uint32_t data[] ,int n );
void print11(uint32_t data[] ,int n);
void Up_ble_data(void);
void uart_tx(uint8_t x);
uint8_t hex_2_char(uint8_t src);
uint8_t char_2_hex(uint8_t src);
/*********************************************************************/
#define friValue    1     //第一阈值，计算区域有效判定
#define secValue    0.85  //第二阈值，加样检测阈值
#define thrValue    0.75  //第三阈值，C线质控阈值
#define testNumber  450
#define t           2     //采样周期
uint16_t c_diff=0;
int startIndex=-1;
uint16_t T[515]= {0};
uint16_t T_1[515]= {0};
uint16_t T_2[515]= {0};
uint16_t B[515]= {0};
uint16_t C[515]= {0};

extern uint8_t trigB;

uint8_t C_DATA[950];

uint8_t uart_rx_flag = 0;
uint16_t user_rx_buffer_length_mask = 8-1;
uint16_t user_rx_buffer_write_index = 0;
uint8_t uart_rx_buffer[30];
uint8_t RxBuff[30];
uint16_t user_rx_buffer_read_index=0;
uint16_t user_tx_buffer_length_mask = 8-1;
uint16_t user_tx_buffer_write_index = 0;
uint16_t user_tx_buffer_read_index=0;
uint8_t uart_tx_buffer[50];
uint8_t Rx_ok;
uint8_t Rx_cnt;

uint16_t Time_Flag=0;//0x012C;
uint16_t Standby_flag=0;//
uint16_t Sampling_flag=0;

float B_led[3];
float C_led[3];
unsigned char byte_arr[32]={0};
unsigned char jeiguo[12]={0};
float Result_1=0;
float Result_2=0;
float Result=0;

void app_uart_process(void);
/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD              1000//1600

// How often to perform read rssi event
#define SBP_READ_RSSI_EVT_PERIOD             3200

// Parameter update delay
#define SBP_PARAM_UPDATE_DELAY               6400

// PHY update delay
#define SBP_PHY_UPDATE_DELAY                 2400

// What is the advertising interval when device is discoverable (units of 625us, 80=50ms)
#define DEFAULT_ADVERTISING_INTERVAL         160//80

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE            GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 6=7.5ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    6

// Maximum connection interval (units of 1.25ms, 100=125ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    100

// Slave latency to use parameter update
#define DEFAULT_DESIRED_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms, 100=1s)
#define DEFAULT_DESIRED_CONN_TIMEOUT         1000//100

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST        TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL        6
// Company Identifier: WCH
#define WCH_COMPANY_ID                       0x07D7

#define INVALID_CONNHANDLE                   0xFFFF

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8_t Peripheral_TaskID = INVALID_TASK_ID; // Task ID for internal task/event processing
uint8_t TaskID_test1;
// GAP - SCAN RSP data (max size = 31 bytes)
uint8_t scanRspData[] = {
    // complete name
    16, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'J', 'N', 'B', '9', '2', '_', 'b', 'l', 'e', '_', 'u', 'a', 'r', 't','T',
    // connection interval range
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] = {
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,                  // length of this data
    GAP_ADTYPE_16BIT_MORE, // some of the UUID's, but not all
    LO_UINT16(SIMPLEPROFILE_SERV_UUID),
    HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// GAP GATT Attributes
//static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "JNC84C2E4036EFF";
extern uint8_t attDeviceName[GAP_DEVICE_NAME_LEN];
//attDeviceName[0]=scanRspData[5];

// Connection item list
static peripheralConnItem_t peripheralConnList;

static uint16_t peripheralMTU = ATT_MTU_SIZE;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);
static void performPeriodicTask(void);
static void simpleProfileChangeCB(uint8_t paramID, uint8_t *pValue, uint16_t len);
static void peripheralParamUpdateCB(uint16_t connHandle, uint16_t connInterval,
                                    uint16_t connSlaveLatency, uint16_t connTimeout);
static void peripheralInitConnItem(peripheralConnItem_t *peripheralConnList);
static void peripheralRssiCB(uint16_t connHandle, int8_t rssi);
static void peripheralChar4Notify(uint8_t *pValue, uint16_t len);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t Peripheral_PeripheralCBs = {
    peripheralStateNotificationCB, // Profile State Change Callbacks
    peripheralRssiCB,              // When a valid RSSI is read from controller (not used by application)
    peripheralParamUpdateCB
};

// Broadcast Callbacks
static gapRolesBroadcasterCBs_t Broadcaster_BroadcasterCBs = {
    NULL, // Not used in peripheral role
    NULL  // Receive scan request callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t Peripheral_BondMgrCBs = {
    NULL, // Passcode callback (not used by application)
    NULL,  // Pairing / Bonding state Callback (not used by application)
    NULL  // oob callback
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t Peripheral_SimpleProfileCBs = {
    simpleProfileChangeCB // Characteristic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Peripheral_Init
 *
 * @brief   Initialization function for the Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Peripheral_Init()
{
    Peripheral_TaskID = TMOS_ProcessEventRegister(Peripheral_ProcessEvent);
    TaskID_test1= TMOS_ProcessEventRegister(Task1_ProcessEvent);
    // Setup the GAP Peripheral Role Profile
    {
        uint8_t  initial_advertising_enable = TRUE;
        uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desired_min_interval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desired_max_interval);
    }

    {
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        // Set advertising interval
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);

 //       GAP_SetParamValue();
        // Enable scan req notify
        GAP_SetParamValue(TGAP_ADV_SCAN_REQ_NOTIFY, ENABLE);
    }

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = 0; // passkey "000000"
        uint8_t  pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t  mitm = TRUE;
        uint8_t  bonding = TRUE;
        uint8_t  ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Setup the SimpleProfile Characteristic Values
    
    {
        //uint8_t charValue2[SIMPLEPROFILE_CHAR2_LEN] = {2};
        uint8_t charValue3[SIMPLEPROFILE_CHAR3_LEN] = {3};
        uint8_t charValue4[SIMPLEPROFILE_CHAR4_LEN] = {4};
        uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = {1, 2, 3, 4, 5};

        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, charValue3);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5);
    }

    // Init Connection Item
    peripheralInitConnItem(&peripheralConnList);

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&Peripheral_SimpleProfileCBs);

    // Register receive scan request callback
    GAPRole_BroadcasterSetCB(&Broadcaster_BroadcasterCBs);

    // Setup a delayed profile startup
    tmos_set_event(Peripheral_TaskID, SBP_START_DEVICE_EVT);

    tmos_start_reload_task( TaskID_test1, TASK1_EVENT1, 400);//开始检测事件

    tmos_start_reload_task( TaskID_test1, TASK1_EVENT2, 1600);//开始计时事件

    tmos_start_reload_task( TaskID_test1, TASK1_EVENT3, 400);//开始指示灯事件

}
uint16_t Task1_ProcessEvent(uint8_t task_id, uint16_t events)
{
        int startIndex=-1;
        uint16_t    i,a,b;
        uint16_t    temp=0;
      if(events & TASK1_EVENT1)
      {
          switch(LED_flag)
          {
              case 1:
              {
                  ADC_ChannelCfg(5);
                  AD_Run();
                  C[c_diff]=sum[0];
              }break;
              case 2:
              {
                  AD_Run();
                  T_2[c_diff]=sum[0];
              }break;
              case 3:
              {
                  ADC_ChannelCfg(8);
                  AD_Run();
                  B[c_diff]=sum[0];
              }break;
              case 4:
              {
                  AD_Run();
                  T_1[c_diff]=sum[0];
              }break;
          }
          LED_flag++;
          LED(LED_flag);
          if(LED_flag==5)
          {
              LED_flag=0;
              charValue1[14]=(C[c_diff]>> 8) & 0xFF; 
              charValue1[15]=(u8)C[c_diff];
              charValue1[16]=(T_2[c_diff]>> 8) & 0xFF; 
              charValue1[17]=(u8)T_2[c_diff];
              charValue1[18]=(B[c_diff]>> 8) & 0xFF; 
              charValue1[19]=(u8)B[c_diff];
              charValue1[20]=(T_1[c_diff]>> 8) & 0xFF; 
              charValue1[21]=(u8)T_1[c_diff];
              SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1);
              //UART1_SendString(charValue1,SIMPLEPROFILE_CHAR1_LEN);
              printf("C:%d,B:%d,T_1:%d,T_2:%d,c_diff:%d\r\n",C[c_diff],B[c_diff],T_1[c_diff],T_2[c_diff],c_diff);
              if((charValue1[0]==0))//未开始就进行预判判断，防止干扰
              {
                //   if(((C[c_diff]>4000)||(C[c_diff]<800)||(B[c_diff]>4000)||(B[c_diff]<800)||(T[c_diff]>4000)||(T[c_diff]<800))&&(c_diff<4))//刚开始的几组数据不能就进行判断，防止误判
                //   {
                //       Redled_flag++;
                //       if(Redled_flag==3)
                //       {
                //           tmos_stop_task(TaskID_test1,TASK1_EVENT3);
                //           GPIOA_ResetBits(GPIO_Pin_11);
                //           GPIOA_SetBits(GPIO_Pin_10);
                //           charValue1[0]=4;//设备故障
                //           SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1);
                //           tmos_stop_task(TaskID_test1,TASK1_EVENT1);
                //           //tmos_stop_task(TaskID_test1,TASK1_EVENT2);
                //           tmos_stop_task(TaskID_test1,TASK1_EVENT4);
                //       }
                //   }
                  tmos_set_event(TaskID_test1,TASK1_EVENT4);
              }
              else if((charValue1[0]==1)||(charValue1[0]==2))
              {
                  c_diff++;
              }
          }
          return (events ^ TASK1_EVENT1);
      }
      if(events & TASK1_EVENT2)//计时
      {
          Up_ble_data();
          // return unprocessed events
          return (events ^ TASK1_EVENT2);
      }
      if(events & TASK1_EVENT3)//指示灯
      {
          switch(charValue1[0])
          {
              case 0://检测未开始
              {
                  GPIOB_InverseBits(GPIO_Pin_2);
                  tmos_start_task(TaskID_test1, TASK1_EVENT3 , 1600);
              }break;
              case 1://正在检测
              {
                  GPIOB_InverseBits(GPIO_Pin_2);
                  tmos_start_task(TaskID_test1, TASK1_EVENT3 , 400);
              }break;
              case 2://完成
              {
                  GPIOB_ResetBits(GPIO_Pin_2);
                  tmos_stop_task(TaskID_test1,TASK1_EVENT3);
              }break;
          }
          if(LED_shan==0)
          {
                if (Time_Flag > 0x0258)//10分钟后常亮
                {
                        GPIOB_ResetBits(GPIO_Pin_2);
                        //tmos_stop_task(TaskID_test1,TASK1_EVENT3);
                }
            }
          // return unprocessed events
          return (events ^ TASK1_EVENT3);
      }
      if(events & TASK1_EVENT4)//加样检测
      {
          if(c_diff>1)
          {
              if(c_diff>=10)
            {
                dif = (float)T[c_diff]/T[c_diff-10];
            }
            else 
            {
                dif = (float)T[c_diff]/T[0];
            }
              if(dif <secValue)
              {
                  //startIndex = c_diff+1;
                  charValue1[0]=1;
                  if(c_diff>=10)//取加样前10个数据
                  {
                      for(i=0;i<10;i++)
                      {
                          C[i]=C[c_diff-10+i];
                          B[i]=B[c_diff-10+i];
                          T[i]=T[c_diff-10+i];
                      }
                      c_diff=9;
                  }
              }
          }
          c_diff++;
          if(c_diff>460)
          {
            for(i=0;i<10;i++)
            {
                C[i]=C[c_diff-460+i];
                B[i]=B[c_diff-460+i];
                T[i]=T[c_diff-460+i];
             }
            c_diff=10;
            }
         return (events ^ TASK1_EVENT4);
      }
      if(events & TASK1_FLASH)//读写数据
      {
          uint8_t  s;
          uint16_t z;
          uint16_t x;

          tmos_set_event(TaskID_test1,TASK1_UART1);
          return (events ^ TASK1_FLASH);
      }
      if(events & TASK1_UART1)//算法
      {

        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1);
          return (events ^ TASK1_UART1);
      }
      if(events & TASK1_TEST)//测试
      {
          switch(LED_flag)
          {
              case 1:
              {
                  ADC_ChannelCfg(5);
                  AD_Run();
                  C[c_diff]=sum[0];
              }break;
              case 2:
              {
                  AD_Run();
                  T_2[c_diff]=sum[0];
              }break;
              case 3:
              {
                  ADC_ChannelCfg(8);
                  AD_Run();
                  B[c_diff]=sum[0];
              }break;
              case 4:
              {
                  AD_Run();
                  T_1[c_diff]=sum[0];
              }break;
          }
          LED_flag++;
        //   if(newValue[4]==0x06)
        //   { 
        //   }
        //   else
        //    {LED(LED_flag);}
          tmos_start_task(TaskID_test1,TASK1_TEST,400);
          if(LED_flag==4)
          {
              tmos_stop_task(TaskID_test1,TASK1_TEST);
                charValue2[3]=0x09;
                charValue2[4]=0x06;
                charValue2[5] = 1;
                charValue2[6] = (C[c_diff]>> 8) & 0xFF; ; //数据域长度
                charValue2[7] = (u8)C[c_diff];

                charValue2[8] = (T_2[c_diff]>> 8) & 0xFF; ; //数据域长度
                charValue2[9] = (u8)T_2[c_diff];

                charValue2[10] = (B[c_diff]>> 8) & 0xFF; ; //数据域长度
                charValue2[11] = (u8)B[c_diff];

                charValue2[12] = (T_1[c_diff]>> 8) & 0xFF; ; //数据域长度
                charValue2[13] = (u8)T_1[c_diff];

                charValue2[14] = 0x5D;
                UART1_SendString(charValue2, SIMPLEPROFILE_CHAR2_LEN);
                SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2);
          }
          return (events ^ TASK1_TEST);
      }

      // Discard unknown events
    return 0;
      return 0;
}

void Up_ble_data(void)
{
    Standby_flag++;
    charValue1[11]=(Standby_flag>> 8) & 0xFF; 
    charValue1[12]=(u8)Standby_flag;
    switch (charValue1[0]) 
    {
        case 0://待机计时
        {
            if (Standby_flag==1800) 
            {
                GPIOA_ResetBits(GPIO_Pin_11);
                GPIOA_SetBits(GPIO_Pin_10);
                charValue1[0]=3;//进入待机
                tmos_stop_task(TaskID_test1,TASK1_EVENT1);
                tmos_stop_task(TaskID_test1,TASK1_EVENT2);//停止计时
                tmos_stop_task(TaskID_test1,TASK1_EVENT3);
                tmos_stop_task(TaskID_test1,TASK1_EVENT4);
                LED(4); //关灯
            }
        }break;
        case 1://正在检测
        {
            if(Time_Flag <= What_time)
            {
                if (Time_Flag==0)//开始计时的时间戳
                {
                charValue1[13]=(Standby_flag>> 8) & 0xFF; 
                charValue1[14]=(u8)Standby_flag;
                }
                if(Time_Flag<0x00ff)//把时间传出
                {
                    charValue1[2]=(u8)Time_Flag;
                    charValue1[1]=0;
                }
                else
                {
                    charValue1[2]=(u8)Time_Flag;
                    charValue1[1]=(u8)(Time_Flag>>8);
                }
                if ((Time_Flag==0x0384)||(Time_Flag==0x0258)||(Time_Flag==0x012C)||(Time_Flag==0x01E0)) //时间到5分钟、10分钟、15分钟时就开始最后一次检测
                {
                int arrtLength=sizeof(T)/sizeof(T[0]);
                    for(int i = 1 ; i < arrtLength ; i++)
                    {
                        float dif = (float)T[i]/T[0];
                        if(dif <secValue)
                        {
                            startIndex = i;
                            break;
                        }
                    }
                    if (startIndex < 0 || startIndex > 65)
                    {
                        startIndex = 65;
                    }

                    float T1 = (float)C[1]/T_1[1];
                    float T2 = (float)C[1]/T_2[1];
                    float B1 = (float)C[1]/B[1];

                    float Tend=0;

                    float Tend_1=0;
                    float Tend_2=0;
                    float Bend=0;
                    float Cend=0;
                        switch (Time_Flag)
                        {
                            case 900: //15分钟
                            {
                                for(int i = 0 ; i <10; i++)
                                {
                                    Tend_1 += T_1[startIndex+440+i]*T1/10;
                                    Tend_2 += T_2[startIndex+440+i]*T2/10;
                                }

                                for(int i = 0 ; i <10; i++)
                                {
                                    Bend += B[startIndex+440+i]*B1/10;
                                }

                                for(int i = 0 ; i <10; i++)
                                {
                                    Cend += C[startIndex+440+i]/10;
                                }
                                Result_1=((Bend-Tend_1)/(Bend-Cend));
                                Result_2=((Bend-Tend_2)/(Bend-Cend));
                                memcpy(&byte_arr[24], &Result_1, sizeof(Result_1));
                                memcpy(&byte_arr[28], &Result_2, sizeof(Result_2));
                                B_led[2]=Bend/B[0];
                                C_led[2]=Cend/C[0];
                            }break;
                            case 600: //10分钟
                            {
                                for(int i = 0 ; i <10; i++)
                                {
                                    Tend_1 += T_1[startIndex+290+i]*T1/10;
                                    Tend_2 += T_2[startIndex+290+i]*T2/10;
                                }

                                for(int i = 0 ; i <10; i++)
                                {
                                    Bend += B[startIndex+290+i]*B1/10;
                                }

                                for(int i = 0 ; i <10; i++)
                                {
                                    Cend += C[startIndex+290+i]/10;
                                }
                                Result_1=((Bend-Tend_1)/(Bend-Cend));
                                Result_2=((Bend-Tend_2)/(Bend-Cend));
                                memcpy(&byte_arr[16], &Result_1, sizeof(Result_1));
                                memcpy(&byte_arr[20], &Result_2, sizeof(Result_2));
                                B_led[1]=Bend/B[0];
                                C_led[1]=Cend/C[0];
                            }break;
                            case 300: //5分钟
                            {
                                memset(byte_arr, 0, sizeof(byte_arr));//清空上一轮结果
                                for(int i = 0 ; i <10; i++)
                                {
                                    Tend_1 += T_1[startIndex+140+i]*T1/10;
                                    Tend_2 += T_2[startIndex+140+i]*T2/10;
                                }

                                for(int i = 0 ; i <10; i++)
                                {
                                    Bend += B[startIndex+140+i]*B1/10;
                                }

                                for(int i = 0 ; i <10; i++)
                                {
                                    Cend += C[startIndex+140+i]/10;
                                }
                                Result_1=((Bend-Tend_1)/(Bend-Cend));
                                Result_2=((Bend-Tend_2)/(Bend-Cend));
                                memcpy(&byte_arr[0], &Result_1, sizeof(Result_1));
                                memcpy(&byte_arr[4], &Result_2, sizeof(Result_2));
                                B_led[0]=Bend/B[0];
                                C_led[0]=Cend/C[0];
                            }break;
                            case 480://8分钟
                            {
                                for(int i = 0 ; i <10; i++)
                                {
                                    Tend_1 += T_1[startIndex+230+i]*T1/10;
                                    Tend_2 += T_2[startIndex+230+i]*T2/10;
                                }

                                for(int i = 0 ; i <10; i++)
                                {
                                    Bend += B[startIndex+230+i]*B1/10;
                                }

                                for(int i = 0 ; i <10; i++)
                                {
                                    Cend += C[startIndex+230+i]/10;
                                }
                                Result_1=((Bend-Tend_1)/(Bend-Cend));
                                Result_2=((Bend-Tend_2)/(Bend-Cend));
                                memcpy(&byte_arr[8], &Result_1, sizeof(Result_1));
                                memcpy(&byte_arr[12], &Result_2, sizeof(Result_2));
                                B_led[0]=Bend/B[0];
                                C_led[0]=Cend/C[0];
                            }break;
                        }
                }
                Time_Flag++;
            }
            else
            {
                charValue1[0]=2;                            //检测完成
                // charValue1[1]=0;                            //时间清零
                // charValue1[2]=0;                          //时间清零
                charValue1[11]=(Standby_flag>> 8) & 0xFF; 
                charValue1[12]=(u8)Standby_flag;
                //tmos_stop_task(TaskID_test1,TASK1_EVENT1);//停止采集
                tmos_set_event(TaskID_test1,TASK1_FLASH);
                //tmos_stop_task(TaskID_test1,TASK1_EVENT2);//关闭计时
                //LED(4);                                     //关灯
            }
        }break;
        case 2:
        {
            if((Time_Flag-What_time) <= 120)
            {Time_Flag++;}
            else
            {
                tmos_stop_task(TaskID_test1,TASK1_EVENT1);//停止采集
                tmos_stop_task(TaskID_test1,TASK1_EVENT2);//关闭计时
                LED(4);                                   //关灯
            }
        }
    }
    //UART1_SendString(charValue1, sizeof(charValue1));
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1);
}
/*********************************************************************
 * @fn      peripheralInitConnItem
 *
 * @brief   Init Connection Item
 *
 * @param   peripheralConnList -
 *
 * @return  NULL
 */
static void peripheralInitConnItem(peripheralConnItem_t *peripheralConnList)
{
    peripheralConnList->connHandle = GAP_CONNHANDLE_INIT;
    peripheralConnList->connInterval = 0;
    peripheralConnList->connSlaveLatency = 0;
    peripheralConnList->connTimeout = 0;
}

/*********************************************************************
 * @fn      Peripheral_ProcessEvent
 *
 * @brief   Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Peripheral_ProcessEvent(uint8_t task_id, uint16_t events)
{
    //  VOID task_id; // TMOS required parameter that isn't used in this function

    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(Peripheral_TaskID)) != NULL)
        {
            Peripheral_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & SBP_START_DEVICE_EVT)
    {
        // Start the Device
        GAPRole_PeripheralStartDevice(Peripheral_TaskID, &Peripheral_BondMgrCBs, &Peripheral_PeripheralCBs);
        return (events ^ SBP_START_DEVICE_EVT);
    }

    if(events & SBP_PERIODIC_EVT)
    {
        // Restart timer
        if(SBP_PERIODIC_EVT_PERIOD)
        {
            tmos_start_task(Peripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);
        }
        // Perform periodic application task
        performPeriodicTask();
        return (events ^ SBP_PERIODIC_EVT);
    }

    if(events & SBP_PARAM_UPDATE_EVT)
    {
        // Send connect param update request
        // When the current connection parameters already meet the requirements for update, return 0x18(InvalidRange)
        GAPRole_PeripheralConnParamUpdateReq(peripheralConnList.connHandle,
                                             DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                             DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                             DEFAULT_DESIRED_SLAVE_LATENCY,
                                             DEFAULT_DESIRED_CONN_TIMEOUT,
                                             Peripheral_TaskID);

        return (events ^ SBP_PARAM_UPDATE_EVT);
    }

    if(events & SBP_PHY_UPDATE_EVT)
    {
        // start phy update
        PRINT("PHY Update %x...\n", GAPRole_UpdatePHY(peripheralConnList.connHandle, 0, 
                    GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, 0));

        return (events ^ SBP_PHY_UPDATE_EVT);
    }

    if(events & SBP_READ_RSSI_EVT)
    {
        GAPRole_ReadRssiCmd(peripheralConnList.connHandle);
        tmos_start_task(Peripheral_TaskID, SBP_READ_RSSI_EVT, SBP_READ_RSSI_EVT_PERIOD);
        return (events ^ SBP_READ_RSSI_EVT);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      Peripheral_ProcessGAPMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Peripheral_ProcessGAPMsg(gapRoleEvent_t *pEvent)
{
    switch(pEvent->gap.opcode)
    {
        case GAP_SCAN_REQUEST_EVENT:
        {
//            PRINT("Receive scan req from %x %x %x %x %x %x  ..\n", pEvent->scanReqEvt.scannerAddr[0],
//                  pEvent->scanReqEvt.scannerAddr[1], pEvent->scanReqEvt.scannerAddr[2], pEvent->scanReqEvt.scannerAddr[3],
//                  pEvent->scanReqEvt.scannerAddr[4], pEvent->scanReqEvt.scannerAddr[5]);
            break;
        }

        case GAP_PHY_UPDATE_EVENT:
        {
            PRINT("Phy update Rx:%x Tx:%x ..\n", pEvent->linkPhyUpdate.connRxPHYS, pEvent->linkPhyUpdate.connTxPHYS);
            break;
        }

        default:
            break;
    }
}

/*********************************************************************
 * @fn      Peripheral_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        case GAP_MSG_EVENT:
        {
            Peripheral_ProcessGAPMsg((gapRoleEvent_t *)pMsg);
            break;
        }

        case GATT_MSG_EVENT:
        {
            gattMsgEvent_t *pMsgEvent;

            pMsgEvent = (gattMsgEvent_t *)pMsg;
            if(pMsgEvent->method == ATT_MTU_UPDATED_EVENT)
            {
                peripheralMTU = pMsgEvent->msg.exchangeMTUReq.clientRxMTU;
                PRINT("mtu exchange: %d\n", pMsgEvent->msg.exchangeMTUReq.clientRxMTU);
            }
            break;
        }

        default:
            break;
    }
}

/*********************************************************************
 * @fn      Peripheral_LinkEstablished
 *
 * @brief   Process link established.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void Peripheral_LinkEstablished(gapRoleEvent_t *pEvent)
{
    gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;

    // See if already connected
    if(peripheralConnList.connHandle != GAP_CONNHANDLE_INIT)
    {
        GAPRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
        PRINT("Connection max...\n");
    }
    else
    {
        peripheralConnList.connHandle = event->connectionHandle;
        peripheralConnList.connInterval = event->connInterval;
        peripheralConnList.connSlaveLatency = event->connLatency;
        peripheralConnList.connTimeout = event->connTimeout;
        peripheralMTU = ATT_MTU_SIZE;
        // Set timer for periodic event
        tmos_start_task(Peripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);

        // Set timer for param update event
        tmos_start_task(Peripheral_TaskID, SBP_PARAM_UPDATE_EVT, SBP_PARAM_UPDATE_DELAY);

        // Start read rssi
        tmos_start_task(Peripheral_TaskID, SBP_READ_RSSI_EVT, SBP_READ_RSSI_EVT_PERIOD);

        PRINT("Conn %x - Int %x \n", event->connectionHandle, event->connInterval);
    }
}

/*********************************************************************
 * @fn      Peripheral_LinkTerminated
 *
 * @brief   Process link terminated.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void Peripheral_LinkTerminated(gapRoleEvent_t *pEvent)
{
    gapTerminateLinkEvent_t *event = (gapTerminateLinkEvent_t *)pEvent;

    if(event->connectionHandle == peripheralConnList.connHandle)
    {
        peripheralConnList.connHandle = GAP_CONNHANDLE_INIT;
        peripheralConnList.connInterval = 0;
        peripheralConnList.connSlaveLatency = 0;
        peripheralConnList.connTimeout = 0;
        tmos_stop_task(Peripheral_TaskID, SBP_PERIODIC_EVT);
        tmos_stop_task(Peripheral_TaskID, SBP_READ_RSSI_EVT);

        // Restart advertising
        {
            uint8_t advertising_enable = TRUE;
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);
        }
    }
    else
    {
        PRINT("ERR..\n");
    }
}

/*********************************************************************
 * @fn      peripheralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void peripheralRssiCB(uint16_t connHandle, int8_t rssi)
{
    //PRINT("RSSI -%d dB Conn  %x \n", -rssi, connHandle);
}

/*********************************************************************
 * @fn      peripheralParamUpdateCB
 *
 * @brief   Parameter update complete callback
 *
 * @param   connHandle - connect handle
 *          connInterval - connect interval
 *          connSlaveLatency - connect slave latency
 *          connTimeout - connect timeout
 *
 * @return  none
 */
static void peripheralParamUpdateCB(uint16_t connHandle, uint16_t connInterval,
                                    uint16_t connSlaveLatency, uint16_t connTimeout)
{
    if(connHandle == peripheralConnList.connHandle)
    {
        peripheralConnList.connInterval = connInterval;
        peripheralConnList.connSlaveLatency = connSlaveLatency;
        peripheralConnList.connTimeout = connTimeout;

        PRINT("Update %x - Int %x \n", connHandle, connInterval);
    }
    else
    {
        PRINT("ERR..\n");
    }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
            PRINT("Initialized..\n");
            break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                Peripheral_LinkTerminated(pEvent);
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
                PRINT("Advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Advertising..\n");
            }
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                Peripheral_LinkEstablished(pEvent);
                PRINT("Connected..\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Connected Advertising..\n");
            }
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Waiting for advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                Peripheral_LinkTerminated(pEvent);
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
            }
            else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                if(pEvent->gap.hdr.status != SUCCESS)
                {
                    PRINT("Waiting for advertising..\n");
                }
                else
                {
                    PRINT("Error..\n");
                }
            }
            else
            {
                PRINT("Error..%x\n", pEvent->gap.opcode);
            }
            break;

        case GAPROLE_ERROR:
            PRINT("Error..\n");
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          TMOS event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask(void)
{
    uint8_t notiData[SIMPLEPROFILE_CHAR4_LEN] = {0x88};
    peripheralChar4Notify(notiData, SIMPLEPROFILE_CHAR4_LEN);
}

/*********************************************************************
 * @fn      peripheralChar4Notify
 *
 * @brief   Prepare and send simpleProfileChar4 notification
 *
 * @param   pValue - data to notify
 *          len - length of data
 *
 * @return  none
 */
static void peripheralChar4Notify(uint8_t *pValue, uint16_t len)
{
    attHandleValueNoti_t noti;
    if(len > (peripheralMTU - 3))
    {
        PRINT("Too large noti\n");
        return;
    }
    noti.len = len;
    noti.pValue = GATT_bm_alloc(peripheralConnList.connHandle, ATT_HANDLE_VALUE_NOTI, noti.len, NULL, 0);
    if(noti.pValue)
    {
        tmos_memcpy(noti.pValue, pValue, noti.len);
        if(simpleProfile_Notify(peripheralConnList.connHandle, &noti) != SUCCESS)
        {
            GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        }
    }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *          pValue - pointer to data that was changed
 *          len - length of data
 *
 * @return  none
 */
static void simpleProfileChangeCB(uint8_t paramID, uint8_t *pValue, uint16_t len)
{
    switch(paramID)
    {
        case SIMPLEPROFILE_CHAR1:
        {
            uint8_t newValue[SIMPLEPROFILE_CHAR1_LEN];
            tmos_memcpy(newValue, pValue, len);
            PRINT("profile ChangeCB CHAR1.. \n");
            break;
        }

        case SIMPLEPROFILE_CHAR2:
        {
            uint8_t newValue[SIMPLEPROFILE_CHAR2_LEN];
            tmos_memcpy(newValue, pValue, len);
            UART1_SendString(newValue,len);
            if((newValue[0] == 0x5B)&&(newValue[1] == 0x0D)&&(newValue[2] == 0x0A))
            {
                charValue2[0] = 0x5B;
                charValue2[1] = 0x0D;
                charValue2[2] = 0x0A;
                charValue2[3] = 0x08; 
                charValue2[7] = 0x5D;
                switch (newValue[4])
                {
                case 0x01:

                    break;
                case 0x02:
                    charValue2[4] = 0x01; 
                    if (newValue[5] == 0x02)
                    {
                        //GPIOB_ResetBits(GPIO_Pin_20);
                        SYS_ResetExecute();
                    }
                    break;
                case 0x03:
                    GPIOB_ResetBits(GPIO_Pin_20);
                    break;
                case 0x04:
                    tmos_stop_task(TaskID_test1,TASK1_EVENT1);
                    tmos_stop_task(TaskID_test1,TASK1_EVENT2);
                    tmos_stop_task(TaskID_test1,TASK1_EVENT3);
                    GPIOB_SetBits(GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_17|GPIO_Pin_16);
                    LED_flag=0;
                    c_diff=0;
                    tmos_start_reload_task(TaskID_test1, TASK1_TEST, 400);
                    break;
                }
                SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2);
            }
            UART1_SendString(charValue2,SIMPLEPROFILE_CHAR2_LEN);
            break;
        }
        case SIMPLEPROFILE_CHAR3:
        {
            uint8_t newValue[SIMPLEPROFILE_CHAR3_LEN];
            tmos_memcpy(newValue, pValue, len);
            PRINT("profile ChangeCB CHAR3..\n");
            break;
        }

        default:
            // should not reach here!
            break;
    }
}

/*********************************************************************
*********************************************************************/
/*********************************************************************
 * @fn      ADC_IRQHandler
 *
 * @brief   ADC中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void ADC_IRQHandler(void) //adc中断服务程序
{
    if(ADC_GetITStatus())
    {
        ADC_ClearITFlag();
        if(adclen < 20)
        {
            adcBuff[adclen] = ADC_ReadConverValue();
            ADC_StartUp(); // 作用清除中断标志并开启新一轮采样
        }
        adclen++;
    }
}

void hex_to_asciistring(u8* str,u32 size,u8* str1)
{
    u8 deposit [2];
    u16 i=0;
    u8 j = 0;

    for(i=0;i<size;i++){

        deposit[1] = str[i] & 0x0F;
        deposit[0] = (str[i] &0xF0) >> 4;
        for(j = 0; j < 2; j++){
        switch(deposit[j]){
            case 0x00:
              str1[i*2+j]='0';
                break;
            case 0x01:
              str1[i*2+j]='1';
                break;
            case 0x02:
              str1[i*2+j]='2';
                break;
            case 0x03:
              str1[i*2+j]='3';
                break;
            case 0x04:
             str1[i*2+j]='4';
                break;
            case 0x05:
              str1[i*2+j]='5';
                break;
            case 0x06:
              str1[i*2+j]='6';
                break;
            case 0x07:
              str1[i*2+j]='7';
                break;
            case 0x08:
              str1[i*2+j]='8';
                break;
            case 0x09:
              str1[i*2+j]='9';
                break;
            case 0x0A:
              str1[i*2+j]='A';
                break;
            case 0x0B:
              str1[i*2+j]='B';
                break;
            case 0x0C:
              str1[i*2+j]='C';
                break;
            case 0x0D:
              str1[i*2+j]='D';
                break;
            case 0x0E:
              str1[i*2+j]='E';
                break;
            case 0x0F:
              str1[i*2+j]='F';
                break;
            default:
                return ;
        }
    }
    }
   return ;
}

void AD_Run(void)//ad采集函数
{
    u_int8_t i;
    for(i = 0; i < 21; i++)
    {
        adcBuff[i] = ADC_ExcutSingleConver();//+RoughCalib_Value; // 连续采样20次
    }
    bubbleSort(adcBuff,20);//调用冒泡排序函数
    sum[0]=adcBuff[10];
    // if(sum[0]<1000)
    // {
    //     PRINT("sum=%d\n,LED_flag=%d\n",sum[0],LED_flag);
    //     ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_1_2);
    //     ADC_ChannelCfg(10);
    //     for(i = 0; i < 21; i++)
    //     {
    //         adcBuff[i] = ADC_ExcutSingleConver();//+RoughCalib_Value; // 连续采样20次
    //     }
    //     bubbleSort(adcBuff,20);//调用冒泡排序函数
    //     sum[0]=adcBuff[10];
    //     PRINT("sum_1=%d\n,LED_flag=%d\n",sum[0],LED_flag);
    //     ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);
    // }
    for(i = 2; i < 18; i++)
    {
        sum[0]=sum[0]+adcBuff[i];
        if(i==17)
        {
            sum[0]=sum[0]/16;
        }
    }
}

void LED(uint8_t LED_flag)
{
        switch(LED_flag)
        {
            case 1:
            {
                GPIOB_ResetBits(GPIO_Pin_9);
                GPIOB_SetBits(GPIO_Pin_8|GPIO_Pin_17|GPIO_Pin_16);
            }break;
            case 2:
            {
                GPIOB_ResetBits(GPIO_Pin_8);
                GPIOB_SetBits(GPIO_Pin_9|GPIO_Pin_17|GPIO_Pin_16);
            }break;
            case 3:
            {
                GPIOB_ResetBits(GPIO_Pin_17);
                GPIOB_SetBits(GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_16);
            }break;
            case 4:
            {
                GPIOB_ResetBits(GPIO_Pin_16);
                GPIOB_SetBits(GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_17);
            }break;
            case 5:
            {
                GPIOB_SetBits(GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_17|GPIO_Pin_16);
            }break;
        }
}

void bubbleSort( uint32_t data[] ,int n )//data[]是传过来的数组，n是数组中那些数的个数
{
 /*----begin------*/
 for(int i=0;i<n-1;i++)//这里是外循环，有n个数就比较n-1次
 {
    for(int j=0;j<n-1-i;j++)//这是内循环，每轮比较n-1-（上一轮的除去的数的个数）
    {
      if(data[j]>data[j+1])//判断如果左边的数大于右边的数，就把大的数往右移
      {
        //这个是交换不用第三方变量的方式，当然也可以借用第三方变量来交换
        data[j]=data[j]+data[j+1];
        data[j+1]=data[j]-data[j+1];
        data[j]=data[j]-data[j+1];
      }
    }
 }
 //print11(data,n);
 /*-----end------*/
}

void print11(uint32_t data[] ,int n)
{
        for(int i=0;i<n;i++)
     printf("%d ",data[i]);
     printf("\n");//输出后换行
}

__INTERRUPT
__HIGH_CODE
void UART1_IRQHandler(void)
{
    volatile uint8_t i,a;
    switch(UART1_GetITFlag())
    {
        case UART_II_LINE_STAT: // 线路状态错误
        {
            UART1_GetLinSTA();
            break;
        }
        case UART_II_RECV_RDY: // 数据达到设置触发点
            for(i = 0; i != trigB-1; i++)
            {
                uart_rx_buffer[i] = UART1_RecvByte();
            }
            Rx_ok++;
            break;
        case UART_II_RECV_TOUT: // 接收超时，暂时一帧数据接收完
             i = UART1_RecvString(uart_rx_buffer);
             for(a = 0; a < i; a++)
             {
                 RxBuff[Rx_cnt]=uart_rx_buffer[a];
                 Rx_cnt++;
             }
             Rx_ok=0;
             //UART1_SendString(RxBuff,Rx_cnt);
            break;

        case UART_II_THR_EMPTY: // 发送缓存区空，可继续发送
            break;

        case UART_II_MODEM_CHG: // 只支持串口0
            break;

        default:
            break;
    }
    app_uart_process();
}

void app_uart_process(void)
{
    UINT32 irq_status;
    uint16_t i;
    uint8_t s;
    uint8_t a=0;
    if(Rx_ok != 0)
    {
        for(i=0;i != trigB-1; i++)
        {
            RxBuff[i+6*(Rx_ok-1)]= uart_rx_buffer[i];
            Rx_cnt++;
        }
    }
        if((RxBuff[0]==0x5B)&&(RxBuff[1]==0x0D)&&(RxBuff[2]==0x0A)&&(RxBuff[Rx_cnt-1]==0x5D))
        {
              switch(RxBuff[4])
              {
                  case 1:
                  {
                      if(RxBuff[5]==01)//读取MAC
                      {
                          uart_tx(01);
                      }
                      else if(RxBuff[5]==0x02)//写入MAC
                      {
                          uart_tx(02);
                      }
                  }break;
                  case 2://SN号
                  {
                      if(RxBuff[5]==01)       //读取SN
                      {
                          uart_tx(3);
                      }
                      else if(RxBuff[5]==0x02)//写入SN
                      {
                          uart_tx(4);
                      }
                  }break;
                  case 3://软件版本号
                  {
                      uart_tx(5);
                  }break;
                  case 4://检测数据
                  {
                      uart_tx(6);
                  }break;
                  case 5://检测结果
                  {
                      uart_tx(7);
                  }break;
                  case 6://重启
                  {
                    SYS_ResetExecute();
                  }break;
                  case 9://测试
                  {
                      uart_tx(9);
                  }break;
                  case 0xFF:
                  {
                      //uart_tx(9);
                  }break;
              }
              Rx_cnt=0;
        }
        if(Rx_ok==0)
        {
            Rx_cnt=0;
        }
      //  SYS_RecoverIrq(irq_status);
//    SYS_DisableAllIrq(&irq_status);//关闭所有中断
//    SYS_RecoverIrq(irq_status);
    //tx process
    if(R8_UART1_TFC < UART_FIFO_SIZE)
    {
        if((user_tx_buffer_write_index - user_tx_buffer_read_index )& user_tx_buffer_length_mask)
        {
            //把软件缓冲区的数据填到uart的硬件发送fifo里
            R8_UART1_THR = uart_tx_buffer[user_tx_buffer_read_index & user_tx_buffer_length_mask] ;
            user_tx_buffer_read_index += 1;
        }else
        {

        }
    }
}

void uart_tx(uint8_t x)
{
    uint16_t i=0;
    uint8_t s;
    uint8_t a=0;
    uart_tx_buffer[0]=0x5B;
    uart_tx_buffer[1]=0x0D;
    uart_tx_buffer[2]=0x0A;
    switch (x)
    {
        case 1://读取MAC
        {
            uart_tx_buffer[3]=0x0C;
            uart_tx_buffer[4]=0x01;
            uart_tx_buffer[5]=0x11;
            for(i=0;i<6;i++)
            {
            uart_tx_buffer[6+i*2]=MacAddr[i]>>4;
            uart_tx_buffer[7+i*2]=MacAddr[i]&0x0F;
            uart_tx_buffer[6+i*2]=hex_2_char(uart_tx_buffer[6+i*2]);
            uart_tx_buffer[7+i*2]=hex_2_char(uart_tx_buffer[7+i*2]);
            }
            uart_tx_buffer[18]=0x5D;
            UART1_SendString(uart_tx_buffer,19);
        }break;
        case 2://写入MAC
        {
            uart_tx_buffer[3]=0x01;
            uart_tx_buffer[4]=0x01;
            uart_tx_buffer[5]=0x12;
            for(i=0;i<12;i++)
            {
                RxBuff[i]=char_2_hex(RxBuff[i+6]);
                if(a==0)
                {
                    RxBuff[i]=(RxBuff[i]<<4);
                    a=1;
                }
                else
                {
                    RxBuff[(i-1)/2]=RxBuff[i-1]|RxBuff[i];
                    a=0;
                }
            }
            s = EEPROM_ERASE(12288,256);//擦除
            s = EEPROM_WRITE(12288, RxBuff, 6);//写入MAC
            EEPROM_READ(12288, MacAddr, 6);//读取MAC
            if((MacAddr[0]==RxBuff[0])&&(MacAddr[1]==RxBuff[1])&&(MacAddr[2]==RxBuff[2])&&(MacAddr[3]==RxBuff[3])&&(MacAddr[4]==RxBuff[4])&&(MacAddr[5]==RxBuff[5]))
            {
                uart_tx_buffer[6]=1;
            }
            else
            {
                uart_tx_buffer[6]=0;
            }
            uart_tx_buffer[7]=0x5D;
            UART1_SendString(uart_tx_buffer,8);
        }break;
        case 3://读取SN
        {
            uart_tx_buffer[3]=0x0E;
            uart_tx_buffer[4]=0x02;
            uart_tx_buffer[5]=0x11;
            EEPROM_READ(12544, SN, 14);//读取SN号
            for(i=0;i<14;i++)
            {
                uart_tx_buffer[6+i]=SN[i];
            }
            uart_tx_buffer[20]=0x5D;
            UART1_SendString(uart_tx_buffer,21);
        }break;
        case 4://写入SN
        {
            uart_tx_buffer[3]=0x01;
            uart_tx_buffer[4]=0x02;
            uart_tx_buffer[5]=0x12;
            for(i=0;i<14;i++)
            {
                RxBuff[i]=RxBuff[6+i];
            }
            s = EEPROM_ERASE(12544,256);//擦除
            s = EEPROM_WRITE(12544, RxBuff, 14);//写入
            EEPROM_READ(12544, SN, 14);//读取
            if((SN[0]==RxBuff[0])&&(SN[1]==RxBuff[1])&&(SN[2]==RxBuff[2])&&(SN[3]==RxBuff[3])&&(SN[4]==RxBuff[4])&&(SN[5]==RxBuff[5])&&(SN[12]==RxBuff[12])&&(SN[13]==RxBuff[13]))
            {
                uart_tx_buffer[6]=1;
            }
            else
            {
                uart_tx_buffer[6]=0;
            }
            uart_tx_buffer[7]=0x5D;
            UART1_SendString(uart_tx_buffer,8);
        }break;
        case 5://版本号
        {
            uart_tx_buffer[3]=0x01;
            uart_tx_buffer[4]=0x03;
            uart_tx_buffer[5]=0x11;
            uart_tx_buffer[6]=charValue1[5];
            uart_tx_buffer[7]=0x5D;
            UART1_SendString(uart_tx_buffer,8);
        }break;  
        case 6://检测数据
        {
            switch(RxBuff[6])
            {
                case 0x11:
                {
                    EEPROM_READ(0, C_DATA, 920);//读取C
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x01;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x12:
                {
                    EEPROM_READ(1024, C_DATA, 920);//读取B
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x02;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x13:
                {
                    EEPROM_READ(2048, C_DATA, 920);//读取T
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x03;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x21:
                {
                    EEPROM_READ(3072, C_DATA, 920);//读取C
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x01;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x22:
                {
                    EEPROM_READ(4096, C_DATA, 920);//读取B
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x02;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x23:
                {
                    EEPROM_READ(5120, C_DATA, 920);//读取T
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x03;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x31:
                {
                    EEPROM_READ(6144, C_DATA, 920);//读取C
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x01;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x32:
                {
                    EEPROM_READ(7168, C_DATA, 920);//读取B
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x02;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x33:
                {
                    EEPROM_READ(8192, C_DATA, 920);//读取T
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x03;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x41:
                {
                    EEPROM_READ(9216, C_DATA, 920);//读取C
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x01;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x42:
                {
                    EEPROM_READ(10240, C_DATA, 920);//读取B
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x02;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
                case 0x43:
                {
                    EEPROM_READ(11264, C_DATA, 920);//读取T
                    for(i=0;i<920;i++)
                    {
                        C_DATA[927-i]=C_DATA[919-i];
                    }
                    C_DATA[0]=0x5B;
                    C_DATA[1]=0x0D;
                    C_DATA[2]=0x0A;
                    C_DATA[3]=0xFF;
                    C_DATA[4]=0x04;
                    C_DATA[5]=0x11;
                    C_DATA[6]=0x03;
                    C_DATA[7]=0x01;
                    C_DATA[928]=0x5D;
                    UART1_SendString(C_DATA,929);
                }break;
            }
        }break;
        case 7://测试结果
        {
            EEPROM_READ(12800,jeiguo, 16);//读取结果
            uart_tx_buffer[3]=0x04;
            uart_tx_buffer[4]=0x05;
            uart_tx_buffer[5]=0x11;
            uart_tx_buffer[10]=0x5D;
            switch (RxBuff[6])
             {
                case 1:
                {
                uart_tx_buffer[6]=jeiguo[0];
                uart_tx_buffer[7]=jeiguo[1];
                uart_tx_buffer[8]=jeiguo[2];
                uart_tx_buffer[9]=jeiguo[3];
                }break;
                case 2:
                {
                uart_tx_buffer[6]=jeiguo[4];
                uart_tx_buffer[7]=jeiguo[5];
                uart_tx_buffer[8]=jeiguo[6];
                uart_tx_buffer[9]=jeiguo[7];
                }break;
                case 3:
                {
                uart_tx_buffer[6]=jeiguo[8];
                uart_tx_buffer[7]=jeiguo[9];
                uart_tx_buffer[8]=jeiguo[10];
                uart_tx_buffer[9]=jeiguo[11];
                }break;
                case 4:
                {
                uart_tx_buffer[6]=jeiguo[12];
                uart_tx_buffer[7]=jeiguo[13];
                uart_tx_buffer[8]=jeiguo[14];
                uart_tx_buffer[9]=jeiguo[15];
                }break;
             }
            UART1_SendString(uart_tx_buffer,11);
        }break;
        case 9:
        {
            tmos_stop_task(TaskID_test1,TASK1_EVENT1);
            tmos_stop_task(TaskID_test1,TASK1_EVENT2);
            tmos_stop_task(TaskID_test1,TASK1_EVENT3);

            LED_flag=0;

            tmos_start_reload_task(TaskID_test1, TASK1_TEST, 400);//开始检测事件
        }break;
    }
}
uint8_t hex_2_char(uint8_t src)
{
    uint8_t desc;

    if((src >= 0) && (src <= 9))
        desc = src + 0x30;
    else if((src >= 0x0A) && (src <= 0x0F))
        desc = src + 0x37;

    return desc;
}
uint8_t char_2_hex(uint8_t src)
{
    uint8_t desc;
    if((src>=0x30)&&(src<0x40))
    {
        desc=src-0x30;
    }
    else if((src >= 0x40) && (src <= 0x67))
    {
        desc = src - 0x37;
    }
    return desc;
}