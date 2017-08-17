/**
  ******************************************************************************
  * @file    main.c
  * @author  Andy
  * @version V1.0
  * @date    6-11-2015
  * @brief   The Main Fuction      "Fire Alarm system -004/005--Released"
  ******************************************************************************/


#include "MyHeader.h"



byte Time_S=0;          //����������ʱ��-S
byte Time_M=0;          //����������ʱ��-M
byte Time_H=0;          //����������ʱ��-H
byte WarningMaskBit=0;      //����������λ
byte CAN_Period=0;          //������CAN����
word CAN_ExtID=0;           //������CAN ID
unsigned char DS18B20NUM;   //������DS18B20����



RCC_ClocksTypeDef  RCC_Clocks;  //����ʱ��


void main(void)
{
  
   //SystemInit();      
   OpenClock();        //Enable Clock
	 LED_IOInit();
	 Key_IOInit();       //Temp Line IO
   Flash_Wr1();       //��һ���ϵ�д��Ĭ��CAN ID
   Flash_Wr2();       //��һ���ϵ�д��Ĭ�ϲ���
	 CAN_Config();      
   WarningMaskBit = *(byte *)MaskBit_Add;                 //��ȡ����λ��־
   CAN_Period     = *(byte *)CommunicationTime_Add;       //��ȡCANͨ������
   CAN_ExtID      = *(word *)CANID_Add;                   //��ȡCAN ID��   
	 ADC1_DMA_Config();
	 DS18B20NUM = SearchRomID(DS18B20ID);       //search ds18b20
	 TimeOut_50ms();
	 Time3Init();          
	 IWDG_Init(5,400);     //Watch Dog config-->1.28S
	
   //RCC_GetClocksFreq(&RCC_Clocks);			//��ȡ����ģ��ʱ��
   
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
     WriteIonValue();             //��һ���ϵ�д�����ʼֵ
#endif
	 }  
}





