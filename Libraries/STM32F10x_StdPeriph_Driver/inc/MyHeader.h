/**
  ******************************************************************************
  * @file    MyHeader.h
  * @author  Andy
  * @version V1.0
  * @date    6-11-2015
  * @brief   Myself header file
  ******************************************************************************/


#ifndef  _MyHeader_
#define  _MyHeader_



/******************************ͷ�ļ�����*********************************/

#include "stm32f10x.h"       //����stm32�Դ�ͷ�ļ�
#include "STM32F10x_GPIO.h"  //����stm32 IO��ͷ�ļ�
#include "STM32F10x_TIM.h"   //stm32��ʱ��ͷ�ļ�
#include "STM32F10x_USART.h"
#include "misc.h"            //stm32�жϿ���ͷ�ļ�
#include "stdio.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_iwdg.h"   //Watch dog
#include "stm32f10x_flash.h"  //Flash
#include "core_cm3.h"
#include "MainTask.h"
#include "string.h"



/***********��������˵��**********/
typedef  unsigned char  byte;
typedef  unsigned short halfword;
typedef  unsigned int   word; 
typedef  float 					fp32;






/*******stm32λ����������******/

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

/***************Flash Add Define************/
#define  IonFlashFlag (0x0800e800)      ///Flash��ʶ��ַ
#define  IonValueadd  (0x0800e800+4)    //���������ӳ�ֵ�洢��ʼ��ַ

#define  FlashWadd    (0x0800f000)     
#define  CANID_Add    (FlashWadd+4)				//CAN ID �洢��ַ����

#define  Config_Add   (0x0800f800)      //���ò����洢��ַ
#define  MaskBit_Add  (Config_Add+7)
#define  BaudRate_Add (Config_Add+6)
#define  CommunicationTime_Add (Config_Add+5)  
#define  Reserved_Add (Config_Add+4)

/***************�˿ں궨��*****************/

#define  LED_B       PAout(4)
#define  LED_R       PAout(5)
#define  TempLineIO  PBin(0)      //�����߶�·����
#define  IrdTx       PAout(8)     //���ⷢ��ʹ��
#define  Fan_Ctr     PBout(6)
//#define  ion_sg      PAin(6)
#define  DS18B20_Pin GPIO_Pin_3   //��ʼ��ʹ�õ����ţ�ע������������˿�ʱ��Ҫ��ʼ����Ӧ�Ķ˿�
#define  DQR PAin(3)		
#define  DQW PAout(3)					    //����DS18B20���ݶ�д��
#define  IonLED      PBout(7) 

/************��������궨��**************/
#define   IonTypedef    2       //����������������� 1:Ion(����) 2:Ird(����)
//#define   SmokeEasy             //Smoke = Ion|CO;
#define   TemplineFuc           //�Ƿ����ø��µ���(105�ȶ�·���)-->�������
#define   CAN_TimeMul      2    //����ͨ��ʱ������--100ms
#define   TempLineTime     60   //������µ����ϵ���Ч���ʱ��(600*1000/1000=60S)
#define 	MAXNUM     1        	//����������18b20�ĸ���(<=10)
#define   CAN_ExID   0x18ff5A51	//����CANĬ��ID(ID�ɸ���Э���޸�)
#define   Parameter  0x00fa0500 //��Ʒ��Ĭ�ϲ���(����λ-������-CAN����-Ԥ��)
#define 	TempLimit  0x5f				//���±�־ֵ95��
#define 	Tempoffset 0x28       //�¶�ƫ����40
#define 	CO_ADLimit 2500			  //2500 CO 2.0V��ֵ����
#define   Flame_ADLimit  1200   //����AD��ֵ����
#define   IonTopADLimit  700    //Ion��ֵ
#define   IonBotADLimit  1200   //Ion�͵�ƽAD��ֵ
#define   Fire2_Use             //�Ƿ������̽ͷ2(PA2)   (004��̽ͷ2,005����)
#define 	ADC1_DR_Address ((uint32_t)0x4001244C)  //����Ӳ��ADC1�������ַ



typedef union
 {
  byte State;
  struct
  {
   byte Bit1        :      1;               
   byte Bit2         :      1;
   byte Bit3         :      1;
   byte SdTemp        :      1; 
   byte Ion        :      1;
   byte CO  		  :      1;
   byte HiTemp      	:      1;
   byte Flame        :      1;     
  }Bits;
 }SorState;
extern SorState StateByte;


typedef union
 {
  byte DATA;
  struct
  {
   byte Smoke        :      1;               
   byte Fire         :      1;
   byte Bit3        :      1;
   byte Bit4        :      1; 
   byte Bit5        :      1;
   byte Bit6  		  :      1;
   byte Bit7      	:      1;
   byte Bit8        :      1;     
  }Bits;
 }CAN_FALAG2;
extern CAN_FALAG2 CAN_Byte2;


typedef union
 {
  byte DATA;
  struct
  {
   byte HiTemp        :      1;               
   byte SdTemp        :      1;
   byte Bit3        :      1;
   byte Bit4        :      1; 
   byte Bit5        :      1;
   byte Bit6  		  :      1;
   byte Bit7      	:      1;
   byte Bit8        :      1;     
  }Bits;
 }CAN_FALAG3;
extern CAN_FALAG3 CAN_Byte3;



extern 	 unsigned short ADCConvertedValue[20];
extern 	 unsigned char DS18B20ID[10][8];
extern	 byte CAN_ReOk;
extern  CanRxMsg RxMessage;
extern  short SdTempValue;
extern  byte TempOverFlag;
extern  byte HiTempFlag;
extern  byte TempLineSort;
extern  byte WarningMaskBit;
extern  byte CAN_Period;
extern  word CAN_ExtID;
extern  byte FedBackFlag;
extern  byte Time_S;
extern  byte Time_M;
extern  byte Time_H;



/******************��������*********************/
void delay(unsigned short z);
void delay_us(u32 nus);
void delay_ms(u16 nms);
void Delay_Second(byte time);
void DS18B20_IO_IN(void);
void DS18B20_IO_OUT(void);
unsigned char DS18B20_Reset(void);
void wr_one_bit(unsigned char Data); 
void wr_ds18b20(unsigned char BYTE);
unsigned char Read_Bit(void);
unsigned char rd_two_bits(void) ;
unsigned char DS18B20_Read_Byte(void);
void OpenClock(void);
void IWDG_Init(byte prer,halfword rlr);
void IWDG_Feed(void);
void TimeOut_50ms(void);
void Time3Init(void);
void PWM_Set(void);
void Key_IOInit(void);
void SPI1_Init(void);
void LED_IOInit(void);
void CAN_Config(void);
void OtherIO_Init(void);
void ADC1_DMA_Config(void);
void USART1_Init(void);
void Time_Fuc(void);



unsigned char SearchRomID(unsigned char RomID[10][8]);
void DS18B20_ReadDesignateTemper(void);
byte searchHighTemp(void);
void ChenckIon(void);
void CheckCO(void);
void CheckFlame(void);
void SmokeEnsure(void);
void TempLineCheck(void);
void update_CANID(void);
void Flash_Wr1(void);
void Flash_Wr2(void);
void WriteIonValue(void);



#endif


