/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "MyHeader.h"

CanRxMsg RxMessage;
byte CAN_ReOk=0;
CAN_FALAG2 CAN_Byte2={0x00};
CAN_FALAG3 CAN_Byte3={0x00};		
short SdTempValue=0x00;
byte TempOverFlag=0;
byte FedBackFlag=0x00;


/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	
}


/*��������:TIM2_IRQHandler()                     
* ��������:Time2��ʱ�жϷ�����
* �������:��
* �������:��
* �� �� ֵ:��
* ˵    ��:no
*/

void TIM2_IRQHandler(void)
{
  static byte count=0;      	 //CAN����ʱ��ȷ��
	static byte FireCnt=0;
	static byte SmokeCnt=0;
	static CanTxMsg TxMessage;
  static byte CanLife=0;
	static byte CANData1=0x02;
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //Clear Interrupt Flag
  
	count++;
	
	/**********LED Status************/
	if(1==CAN_Byte2.Bits.Fire)
		{
			SmokeCnt=0;
			FireCnt++;
			if(FireCnt==5)     //0.25S��˸
				{
					FireCnt=0;
					LED_R=!LED_R;
				}
		}
	else if((1==CAN_Byte3.Bits.HiTemp)||(1==CAN_Byte3.Bits.SdTemp))
		{
			FireCnt=0;
			SmokeCnt++;
			if(20==SmokeCnt)  //1S��˸
				{
					SmokeCnt=0;
					LED_R=!LED_R;
				}
		}
	else if(CAN_Byte2.Bits.Smoke==1)
			LED_R=0;
	else
			LED_R=1;

	/**********CAN����***************/
  if(count==(CAN_Period*CAN_TimeMul))     //����ʱ��ȷ��
    {
      count=0;
        
      if((0!=CAN_Byte2.DATA)||(0!=CAN_Byte3.DATA))
					CANData1=0x04;
			else if((0==CAN_Byte2.DATA)&&(0==CAN_Byte3.DATA))
					CANData1=0x02;
			else{;}
			
			TxMessage.ExtId = CAN_ExtID;						
			TxMessage.RTR = CAN_RTR_DATA;										
			TxMessage.IDE = CAN_ID_EXT;											
			TxMessage.DLC = 8;															
			TxMessage.Data[0] = CANData1;							
			TxMessage.Data[1] = CAN_Byte2.DATA;     
			TxMessage.Data[2] =  CAN_Byte3.DATA;			
			TxMessage.Data[3] = searchHighTemp();					  	
			TxMessage.Data[4] = 0x00;							
			TxMessage.Data[5] = 0x00;							
			TxMessage.Data[6] = 0x00;							
			TxMessage.Data[7] = CanLife;		    			
			CAN_Transmit(CAN1,&TxMessage);	
			if(255==CanLife)
				CanLife=0;
				CanLife++;
    }
  
  /*******��������Ϣ���ش���->����������(2017-7-17���)******/
  if(FedBackFlag == 0x55)
    {
      TxMessage.ExtId = 0x18ff4321;						
			TxMessage.RTR = CAN_RTR_DATA;										
			TxMessage.IDE = CAN_ID_EXT;											
			TxMessage.DLC = 8;															
			TxMessage.Data[0] = (byte)CAN_ExtID;	      //����ID��						
			TxMessage.Data[1] = *(byte *)MaskBit_Add;   //��������λ״̬   
			TxMessage.Data[2] = CAN_Period;			        //����CAN����
			TxMessage.Data[3] = 0x20;					  	//Ӳ���汾-V2.0
			TxMessage.Data[4] = 0x22;							//����汾--->���Ӱ汾Ϊ:V2.1����汾Ϊ:V2.2
			TxMessage.Data[5] = Time_S;					  //�ϵ���������ʱ��-S
			TxMessage.Data[6] = Time_M;					  //�ϵ���������ʱ��-M
			TxMessage.Data[7] = Time_H;		    		//�ϵ���������ʱ��-H
			CAN_Transmit(CAN1,&TxMessage);	
      
      FedBackFlag = 0;
    }
}
	

/*��������:TIM3_IRQHandler()                     
* ��������:Time3��ʱ�жϷ�����
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:Time out 100ms
*/

void TIM3_IRQHandler(void)
{
	static byte SdTempCnt=0;
	static byte SdFlag=0;
	static short oldTemp=0xff;
  static byte TimeCnt=0;
  
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //Clear Interrupt Flag
  
  /**********������ϵͳʱ��***********/
  TimeCnt++;
  if(TimeCnt==10)
    {
      TimeCnt=0;
      Time_S++;
      if(Time_S==60)
        {
          Time_S=0;
          Time_M++;
          if(Time_M==60)
            {
              Time_M=0;
              Time_H++;
            }
        }
    } 
  
	/************�¶�ͻ����***********/
	if((1!=SdFlag)&&(0x55==TempOverFlag))
		{
			SdFlag=1;
			oldTemp=searchHighTemp();    //ȡ��ֵ�¶�
		}
	
	if((SdTempValue>oldTemp)&&(oldTemp>=50)&&(0x55==TempOverFlag))    //50������10S������10��
		{
			SdTempCnt++;
			if(SdTempCnt==100)
				{
					SdTempCnt=0;
					if((oldTemp+10)<SdTempValue)
						{
              if(0==(WarningMaskBit&0x08))
                  CAN_Byte3.Bits.SdTemp=1;
              else{CAN_Byte3.Bits.SdTemp=0;}
							oldTemp += 10;
						}else{oldTemp=SdTempValue;CAN_Byte3.Bits.SdTemp=0;StateByte.Bits.SdTemp=0;}
				}
		}else{oldTemp=SdTempValue;CAN_Byte3.Bits.SdTemp=0;StateByte.Bits.SdTemp=0;SdTempCnt=0;}
	
	/**********������ֱ�־�ж�**********/
	SmokeEnsure();
	CheckFlame();
  
#ifdef TemplineFuc
	if(((1==CAN_Byte3.Bits.HiTemp)&&(1==StateByte.Bits.Flame))||((1==CAN_Byte2.Bits.Smoke)&&\
		((1==StateByte.Bits.Flame)||(1==CAN_Byte3.Bits.HiTemp)||(1==CAN_Byte3.Bits.SdTemp)||(0x55==TempLineSort))))         
    {
      CAN_Byte2.Bits.Fire=1;  
    }else{CAN_Byte2.Bits.Fire=0;}
#else
  if(((1==CAN_Byte3.Bits.HiTemp)&&(1==StateByte.Bits.Flame))||((1==CAN_Byte2.Bits.Smoke)&&\
		((1==StateByte.Bits.Flame)||(1==CAN_Byte3.Bits.HiTemp)||(1==CAN_Byte3.Bits.SdTemp))))         
    {
      CAN_Byte2.Bits.Fire=1;  
    }else{CAN_Byte2.Bits.Fire=0;}
#endif
  
}


/*��������:TIM4_IRQHandler(void)                     
* ��������:TIM4 Interrupt Serve 
* �������:��
* �������:��
* �� �� ֵ:��
* ˵    ��:��
*/

void TIM4_IRQHandler(void)
{
	
}


/*��������:USB_LP_CAN1_RX0_IRQHandler(void)                     
* ��������:CAN Receive fuction
* �������:no
* �������:no
* �� �� ֵ:no
* ˵    ��:no
*/

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);	 //ʧ��CAN1��Ϣ�����ж�
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); //���FIFO0��Ϣ�Һ��жϱ�־λ 
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage); //��FIFO0�н���������Ϣ������Ϣ�ṹ����
  if(CAN_ReOk < 5)
    CAN_ReOk += 1;
	CAN_FIFORelease(CAN1,CAN_FIFO0); 			//���жϱ�־
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn); //ʹ��CAN1���ݽ����ж�
	
}

/*��������:USART1_IRQHandler(void)                     
* ��������:USART1���ͽ����ж�
* �������:no
* �������:no
* �� �� ֵ:no
* ˵    ��:no
*/

void USART1_IRQHandler(void)
{
	
}


/*��������:EXTI9_5_IRQHandler(void)                     
* ��������:5-9 Interrupt Line Fuction
* �������:no
* �������:no
* �� �� ֵ:no
* ˵    ��:no
*/

void EXTI9_5_IRQHandler(void)
{

}


/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	//OS_TimeMS ++;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
