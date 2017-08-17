/**
  ******************************************************************************
  * @file    main.c
  * @author  Andy
  * @version V1.0
  * @date    6-11-2015
  * @brief   The Main Fuction      "Fire Alarm system -004/005--Released"
  ******************************************************************************/


#include "MyHeader.h"



byte Time_S=0;          //传感器工作时间-S
byte Time_M=0;          //传感器工作时间-M
byte Time_H=0;          //传感器工作时间-H
byte WarningMaskBit=0;      //传感器屏蔽位
byte CAN_Period=0;          //传感器CAN周期
word CAN_ExtID=0;           //传感器CAN ID
unsigned char DS18B20NUM;   //总线上DS18B20数量



RCC_ClocksTypeDef  RCC_Clocks;  //总线时钟


void main(void)
{
  
   //SystemInit();      
   OpenClock();        //Enable Clock
	 LED_IOInit();
	 Key_IOInit();       //Temp Line IO
   Flash_Wr1();       //第一次上电写入默认CAN ID
   Flash_Wr2();       //第一次上电写入默认参数
	 CAN_Config();      
   WarningMaskBit = *(byte *)MaskBit_Add;                 //读取屏蔽位标志
   CAN_Period     = *(byte *)CommunicationTime_Add;       //读取CAN通信周期
   CAN_ExtID      = *(word *)CANID_Add;                   //读取CAN ID号   
	 ADC1_DMA_Config();
	 DS18B20NUM = SearchRomID(DS18B20ID);       //search ds18b20
	 TimeOut_50ms();
	 Time3Init();          
	 IWDG_Init(5,400);     //Watch Dog config-->1.28S
	
   //RCC_GetClocksFreq(&RCC_Clocks);			//读取各个模块时钟
   
	 while(1)	 
	 {
		 DS18B20_ReadDesignateTemper();
#ifdef TemplineFuc
     TempLineCheck();
#endif
		 update_CANID();
		 IWDG_Feed();    
		 delay_ms(150);
#if IonTypedef==2
     WriteIonValue();             //第一次上电写入光电初始值
#endif
	 }  
}





