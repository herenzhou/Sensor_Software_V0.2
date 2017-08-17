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


/*函数名称:TIM2_IRQHandler()                     
* 函数介绍:Time2定时中断服务函数
* 输入参数:无
* 输出参数:无
* 返 回 值:无
* 说    明:no
*/

void TIM2_IRQHandler(void)
{
  static byte count=0;      	 //CAN发送时间确定
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
			if(FireCnt==5)     //0.25S闪烁
				{
					FireCnt=0;
					LED_R=!LED_R;
				}
		}
	else if((1==CAN_Byte3.Bits.HiTemp)||(1==CAN_Byte3.Bits.SdTemp))
		{
			FireCnt=0;
			SmokeCnt++;
			if(20==SmokeCnt)  //1S闪烁
				{
					SmokeCnt=0;
					LED_R=!LED_R;
				}
		}
	else if(CAN_Byte2.Bits.Smoke==1)
			LED_R=0;
	else
			LED_R=1;

	/**********CAN发送***************/
  if(count==(CAN_Period*CAN_TimeMul))     //发送时间确定
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
  
  /*******传感器信息返回代码->不主动发送(2017-7-17添加)******/
  if(FedBackFlag == 0x55)
    {
      TxMessage.ExtId = 0x18ff4321;						
			TxMessage.RTR = CAN_RTR_DATA;										
			TxMessage.IDE = CAN_ID_EXT;											
			TxMessage.DLC = 8;															
			TxMessage.Data[0] = (byte)CAN_ExtID;	      //本机ID号						
			TxMessage.Data[1] = *(byte *)MaskBit_Add;   //设置屏蔽位状态   
			TxMessage.Data[2] = CAN_Period;			        //设置CAN周期
			TxMessage.Data[3] = 0x20;					  	//硬件版本-V2.0
			TxMessage.Data[4] = 0x22;							//软件版本--->离子版本为:V2.1红外版本为:V2.2
			TxMessage.Data[5] = Time_S;					  //上电连续工作时间-S
			TxMessage.Data[6] = Time_M;					  //上电连续工作时间-M
			TxMessage.Data[7] = Time_H;		    		//上电连续工作时间-H
			CAN_Transmit(CAN1,&TxMessage);	
      
      FedBackFlag = 0;
    }
}
	

/*函数名称:TIM3_IRQHandler()                     
* 函数介绍:Time3定时中断服务函数
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:Time out 100ms
*/

void TIM3_IRQHandler(void)
{
	static byte SdTempCnt=0;
	static byte SdFlag=0;
	static short oldTemp=0xff;
  static byte TimeCnt=0;
  
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //Clear Interrupt Flag
  
  /**********传感器系统时间***********/
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
  
	/************温度突变检测***********/
	if((1!=SdFlag)&&(0x55==TempOverFlag))
		{
			SdFlag=1;
			oldTemp=searchHighTemp();    //取初值温度
		}
	
	if((SdTempValue>oldTemp)&&(oldTemp>=50)&&(0x55==TempOverFlag))    //50℃以上10S内温升10℃
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
	
	/**********烟雾火灾标志判断**********/
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


/*函数名称:TIM4_IRQHandler(void)                     
* 函数介绍:TIM4 Interrupt Serve 
* 输入参数:无
* 输出参数:无
* 返 回 值:无
* 说    明:无
*/

void TIM4_IRQHandler(void)
{
	
}


/*函数名称:USB_LP_CAN1_RX0_IRQHandler(void)                     
* 函数介绍:CAN Receive fuction
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
*/

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);	 //失能CAN1消息接收中断
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); //清除FIFO0消息挂号中断标志位 
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage); //将FIFO0中接收数据信息存入消息结构体中
  if(CAN_ReOk < 5)
    CAN_ReOk += 1;
	CAN_FIFORelease(CAN1,CAN_FIFO0); 			//清中断标志
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn); //使能CAN1数据接收中断
	
}

/*函数名称:USART1_IRQHandler(void)                     
* 函数介绍:USART1发送接收中断
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
*/

void USART1_IRQHandler(void)
{
	
}


/*函数名称:EXTI9_5_IRQHandler(void)                     
* 函数介绍:5-9 Interrupt Line Fuction
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
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
