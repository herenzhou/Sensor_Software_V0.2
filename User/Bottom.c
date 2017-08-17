/**
  ******************************************************************************
  * @file    Bottom.c
  * @author  Andy
  * @version V1.0
  * @date    6-11-2015
  * @brief   Bottom drive and initialization
  ******************************************************************************/

#include "MyHeader.h"


unsigned short ADCConvertedValue[20]={0,0,0,0,0,
                                      0,0,0,0,0,
                                      0,0,0,0,0,
                                      0,0,0,0,0};

/*��������:delay(unsigned char z)                     
* ��������:�򵥵���ʱ����
* �������:z
* �������:NO
* �� �� ֵ:NO
*/

void delay(unsigned short z)
{
  unsigned short x,y;
  for(x=200;x>0;x--)
     for(y=z;y>0;y--);
}


/*��   �ƣ�delay_us(u32 nus)
* ��   �ܣ�΢����ʱ����
* ��ڲ�����u32  nus
* ���ڲ�������
* ˵    ����
* ���÷������� 
*/ 

void delay_us(u32 nus)
{
	 u32 temp;
	 SysTick->LOAD = 9*nus;
	 SysTick->VAL=0x00;//��ռ�����
	 SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
	 do
	 {
	  temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
	 }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
	 
	 SysTick->CTRL=0x00; //�رռ�����
	 SysTick->VAL =0X00; //��ռ�����
}


/*��    �ƣ�delay_ms(u16 nms)
* ��    �ܣ�������ʱ����
* ��ڲ�����u16 nms
* ���ڲ�������
* ˵    ����
* ���÷������� 
*/

void delay_ms(u16 nms)
{
     //ע�� delay_ms�������뷶Χ��1-1863
	 //���������ʱΪ1.8��

	 u32 temp;
	 SysTick->LOAD = 9000*nms;
	 SysTick->VAL=0X00;//��ռ�����
	 SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
	 do
	 {
	  temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
	 }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
	 SysTick->CTRL=0x00; //�رռ�����
	 SysTick->VAL =0X00; //��ռ�����
}

/*��������:Delay_Second(byte time)                     
* ��������:��ȷ��ʱs
* �������:time
* �������:no
* �� �� ֵ:no
* ˵    ��:�˺���ֻ�����ڳ�ʼ������
*/

void Delay_Second(byte time)
{
	byte WaitTime=0;
	do
	{
		SysTick->CTRL &= 0xffffff0;      //�ر�SysTick ʱ��=ϵͳʱ��/8  ��ֹ�ж�
		//Clearflag = SysTick->CTRL;         //clear flag 
		SysTick->VAL = 0x0;               //��ռ�����
		//time = time * 1000;
		SysTick->LOAD = 9000*1000*1;       //1S����ʱʱ��
		SysTick->CTRL |= 0x00000001;       //ENABLE����ʼ����	Disable Interrupt
		while(0==(SysTick->CTRL>>16));		 //�ȴ���ʱʱ�䵽
		SysTick->CTRL &= 0xffffff0;        //�رն�ʱ��
		WaitTime++;
	}while(WaitTime<time);
	/*
	SysTick_Config(SystemCoreClock / (1000*1000));  //1s��ʱ
	while(!(SysTick->CTRL>>16));
	SysTick->CTRL &= 0xffffff0;        //�رն�ʱ��
	*/
}


/*��������:DS18B20_IO_IN(void)                     
* ��������:DBS18B20 IO��������
* �������:��
* �������:��
* �� �� ֵ:��
*/ 

void DS18B20_IO_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin=DS18B20_Pin;			//DS18B20 IO������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}


/*��������:DS18B20_IO_OUT(void)                     
* ��������:DBS18B20 IO�������
* �������:��
* �������:��
* �� �� ֵ:��
*/ 

void DS18B20_IO_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin=DS18B20_Pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}


/*��������:DS18B20_Reset(void)                     
* ��������:��λDS18B20
* �������:��
* �������:��
* �� �� ֵ:��
*/ 

unsigned char DS18B20_Reset(void)
{
	static unsigned char x=0;
	DS18B20_IO_OUT();//���
	DQW=1;
	delay_us(10);
	DQW=0;
	delay_us(500);		//��ʱ480΢��	
	DQW=1;
	delay_us(300);		//��ʱ480΢��	
	DS18B20_IO_IN();	//����
	x=DQR;
	delay_us(20);
	DS18B20_IO_OUT();
	DQW=1;
	return x;
}


/*��������:wr_one_bit(unsigned char Data)                     
* ��������:������д1λ����
* �������:unsigned char Data
* �������:��
* �� �� ֵ:��
*/ 

void wr_one_bit(unsigned char Data) 
{ 
  asm("CPSID I");
  
	 DS18B20_IO_OUT(); 
	 DQW=0;
   delay_us(2); 
   if(Data&0x01) 
	 	{
			DQW=1;
		} 
	 else
	 {
			DQW=0;
	 } 
   delay_us(60); 
   DQW=1;  
   
   asm("CPSIE  I");
}


/*��������:wr_ds18b20(unsigned char BYTE)                     
* ��������:������д1Byte����
* �������:unsigned char BYTE
* �������:��
* �� �� ֵ:��
*/ 

void wr_ds18b20(unsigned char BYTE) 
{ 
  unsigned char i; 
	
  asm("CPSID I");
  
	DS18B20_IO_OUT();  
  for(i=0;i<8;i++) 
   { 
      DQW=0; 
      delay_us(5); 
      if((BYTE>>i)&0x01) 
				{
					DQW=1;
				} 
      else DQW=0; 
      delay_us(50);  
			DQW=1; 
      delay_us(10); 
   } 
  asm("CPSIE  I");
}


/*��������:Read_Bit(void)                     
* ��������:���ܶ�1λ����
* �������:��
* �������:��
* �� �� ֵ:ret
*/ 

unsigned char Read_Bit(void)
{
  unsigned char ret;
  
  asm("CPSID I");
  
	DS18B20_IO_OUT();		//���
	DQW=0;  
	delay_us(2);
	DQW=1;
	delay_us(7);
	DS18B20_IO_IN();	//����
  ret=DQR;						//��ʱ϶����7 us���ȡ�������ݡ������ߵĶ�ȡ��������15usʱ�����Ƶĺ�����Ϊ�˱�֤���ݶ�ȡ����Ч��
  delay_us(60);			//��ʱ60us�������ʱ϶��ʱ�䳤��Ҫ�� 
	DS18B20_IO_OUT();	//���
  DQW=1;	//�ͷ����� 
  
  asm("CPSIE  I");
  
  return ret; 			//���ض�ȡ�������� 
}


/*��������:rd_two_bits(void)                     
* ��������:�������϶�2bit����
* �������:��
* �������:��
* �� �� ֵ:Data
*/

unsigned char rd_two_bits(void) 
{ 
 unsigned char i,Data=0x00; 
 
 asm("CPSID I");
 
 for(i=0;i<2;i++) 
   { 
		 DS18B20_IO_OUT();
      Data>>=1;      
      DQW=0; 
      delay_us(5); 
      DQW=1; 
      delay_us(8); 
			DS18B20_IO_IN();
      if(DQR) Data|=0x02; 
      delay_us(45);
			DS18B20_IO_OUT();
      DQW=1; 
   } 
 asm("CPSIE  I");
   return Data; 
}  


/*��������:DS18B20_Read_Byte(void)                     
* ��������:�������϶�1Byte����
* �������:��
* �������:��
* �� �� ֵ:��
*/

unsigned char DS18B20_Read_Byte(void)
{
	unsigned char i=0,TempData=0;
  
  asm("CPSID I");
  
	for(i=0;i<8;i++)
	{
		TempData>>=1;

		DS18B20_IO_OUT();//���
		DQW=0;	 //����
		delay_us(4);//��ʱ4΢��
		DQW=1;
		delay_us(10);//��ʱ10΢��
		DS18B20_IO_IN();

		if(DQR==1)
		{
		   TempData|=0x80;//������ �ӵ�λ��ʼ
		}
		delay_us(45);//��ʱ45΢��
	}
  asm("CPSIE  I");
  
	return TempData;
}


/*��������:OpenClock(void)                     
* ��������:������Ҫʹ�õĸ���ģ���ʱ��
* �������:��
* �������:��
* �� �� ֵ:��
*/

void OpenClock(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);   //����APB2������TIME2ʱ��(TIME2����APB2����)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);   
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);		//����CAN1ʱ��
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);     //����APB2������TIME5ʱ��(TIME2����APB2����)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    //ʹ��GPIO�����ܸ���IOʱ��
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);			  //STemwin��Ҫ��ʱ��,�����޷�����
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
  											 RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE ,ENABLE);  //����IO��A��B��C��D��E�˿ڵ�ʱ��
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
}


/*��������:IWDG_Init(byte prer,halfword rlr)                     
* ��������:Set WatchDog And Enable It 
* �������:prer,rlr  
* �������:��
* �� �� ֵ:��
* ˵    ��:�������prer�Ƿ�Ƶ��rlrװ�س�ֵ�� Time=((4*2^prer)*rlr)/40MS  rlr[11:0] prer[2:0] 4*2^prer=(0-256)
*/

void IWDG_Init(byte prer,halfword rlr)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);   //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
  IWDG_SetPrescaler(prer);    //����IWDGԤ��Ƶֵ:����IWDGԤ��Ƶֵ
  IWDG_SetReload(rlr);        //����IWDG��װ��ֵ
  IWDG_ReloadCounter();       //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
  IWDG_Enable();              //ʹ��IWDG
}


/*��������:IWDG_Feed(void)                     
* ��������:Feed WatchDog
* �������:��
* �������:��
* �� �� ֵ:��
* ˵    ��:��
*/

void IWDG_Feed(void)
{
 	IWDG_ReloadCounter();    //Feed Dog
}


/*��������:TimeOut_10ms(void)                     
* ��������:10ms��ʱ�ж�
* �������:��
* �������:��
* �� �� ֵ:��
*/

void TimeOut_50ms(void)
{
  TIM_DeInit(TIM2);          //Time2��ʱ������ȱʡֵ
  TIM_TimerBaseStruct.TIM_Period=99;      //�������ؼĴ�����ֵ (����Ϊ99��������ʱ50ms)
  TIM_TimerBaseStruct.TIM_Prescaler=35999;  //ʱ�ӷ�Ƶ��(72MHZ��Ƶ��ʱ��Ϊ2.0KHZ)
  TIM_TimerBaseStruct.TIM_ClockDivision=0;  //����ʱ�ӷָ�(��ʱδ��)
  TIM_TimerBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;   //���ü��������ϼ���ģʽ
  TIM_TimeBaseInit(TIM2,&TIM_TimerBaseStruct);  //��ʼ������
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;   //�ж�ͨ��ѡ��=TIM2�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�����0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;       //��Ӧ���ȼ�����2
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;          //ʹ���жϹ�������
  NVIC_Init(&NVIC_InitStructure);                        //���ݲ���ȷ������
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);  //ʹ��TIM2�ж�Դ
  TIM_Cmd(TIM2,ENABLE);                     //ʹ��TIM2��ʱ��	
}


/*��������:Time3Init(void)                     
* ��������:Timer3 Init
* �������:��
* �������:��
* �� �� ֵ:��
*/

void Time3Init(void)
{
  TIM_DeInit(TIM3);          //Time3��ʱ������ȱʡֵ
  TIM_TimerBaseStruct.TIM_Period=199;      //�������ؼĴ�����ֵ (����Ϊ199��������ʱ100ms)
  TIM_TimerBaseStruct.TIM_Prescaler=35999; //ʱ�ӷ�Ƶ��(72MHZ��Ƶ��ʱ��Ϊ2.0MHZ)
  TIM_TimerBaseStruct.TIM_ClockDivision=0;  //����ʱ�ӷָ�(��ʱδ��)
  TIM_TimerBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;   //���ü��������ϼ���ģʽ
  TIM_TimeBaseInit(TIM3,&TIM_TimerBaseStruct);  //��ʼ������
	//TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;   //�ж�ͨ��ѡ��=TIM2�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�����0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;       //��Ӧ���ȼ�����2
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;          //ʹ���жϹ�������
  NVIC_Init(&NVIC_InitStructure);                        //���ݲ���ȷ������
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);  //ʹ��TIM3�ж�Դ

  TIM_Cmd(TIM3,ENABLE);                     //ʹ��TIM3��ʱ��	
	
/*
	RCC->APB1ENR  |= (uint32_t)0x01<<2;
	RCC->APB1RSTR |= (uint32_t)0x01<<2;
	RCC->APB1RSTR &= ~((uint32_t)0x01<<2);

	
	TIM4->CR1 = 0x0004;
	TIM4->CR2 = 0x0000;
	TIM4->DIER= 0x0001;		//�жϿ���
	TIM4->PSC = 36-1;			// ��Ƶϵ��
	TIM4->ARR = 0xffff;		// ����ֵ
	
	TIM4->CR1 |=0x0001;		//����

	NVIC->IP[30]  =  0x90;
	NVIC->ISER[0] = (uint32_t)0x01<<30;
	//NVIC->ICER[1] = (uint32_t)0x01<<30;
*/
}


/*��������:PWM_Set(void)                     
* ��������:SET OUTPUT PWM
* �������:��
* �������:��
* �� �� ֵ:��
* ˵    ��:����ģ�����������
*/

void PWM_Set(void)
{ 
  TIM_DeInit(TIM4);          //Time4��ʱ������ȱʡֵ
  TIM_TimerBaseStruct.TIM_Period=18;      //�������ؼĴ�����ֵ (����Ϊ60��������ʱ20ms)
  TIM_TimerBaseStruct.TIM_Prescaler=719;  //ʱ�ӷ�Ƶ��(72MHZ��Ƶ��ʱ��Ϊ8.0KHZ)
  TIM_TimerBaseStruct.TIM_ClockDivision=0;  //����ʱ�ӷָ�(��ʱδ��)
  TIM_TimerBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;   //���ü��������ϼ���ģʽ
  TIM_TimeBaseInit(TIM4,&TIM_TimerBaseStruct);  //��ʼ������
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //ѡ��pin3��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //I/Oʱ��Ϊ50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);    //��������ָ��������ʼ��GPIOA�ṹ��
      
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        //ѡ��PWM���Ϊģʽ1
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM����Ƚϼ��Ը�
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ͨ��1���ʹ��
  TIM_OCInitStructure.TIM_Pulse =9;  //����ֵΪ9  ��PWM=50%
  TIM_OC4Init(TIM4,&TIM_OCInitStructure); //����TIM_OCInitStruct��ָ���Ĳ�����ʼ��TIM2
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ClearITPendingBit(TIM4,TIM_IT_CC4);//Ԥ����������ж�λ
  TIM_ARRPreloadConfig(TIM4, ENABLE);    //ʹ��TIM4��ARR�ϵ�Ԥװ�ؼĴ���
  TIM_Cmd(TIM4,ENABLE);                     //ʹ��TIM4��ʱ��
}


/*��������:Key_IOInit(void)                   
* ��������:Touch Key IO Init
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void Key_IOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
//  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;   
//  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;    	 //����Ϊ10MHZ
//  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;       	 //��������
//  GPIO_Init(GPIOA,&GPIO_InitStructure);                //��ʼ������	
  
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;              //�����ⷢ�͹�ʹ��
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;    	 
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;       	 
  GPIO_Init(GPIOA,&GPIO_InitStructure);   
  IrdTx = 0;
  
#ifdef TemplineFuc
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;   
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;    	 
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;       	 //��������
  GPIO_Init(GPIOB,&GPIO_InitStructure);
#endif
}


/*��������:SPI1_Init(void)                   
* ��������:SPI1 Config
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;    //����������Ϊ10MHZ
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;      //IO������Ϊ�����������
  GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��IO_A�� PIN5(SCK)��PIN6(MISO)��PIN7(MOSI)
	
	SPI_InitTypeDef         SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPIΪ˫��˫��ȫ˫��
  SPI_Cmd(SPI1, DISABLE);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;      //����Ϊ��SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;  //���ݽṹΪ8bit
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	       //ʱ�ӿ���ʱΪ0
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       //���ݲ����ڵ�һ��ʱ����
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				   //�ڲ�NSS�ź����������
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;     //������Ԥ��ƵֵΪ32
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;			     //���ݴ���ӵ�λ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial = 7;			             //CRCֵ����Ķ���ʽ���Ϊ7��
  SPI_Init(SPI1, &SPI_InitStructure);   //�������ϲ�����ʼ��SPI�ṹ��
  //SPI_SSOutputCmd(SPI1, ENABLE);		//Enable SPI1.NSS as a GPIO
  SPI_Cmd(SPI1, ENABLE);	 		//ʹ��SPI1
}


/*��������:RF4332_IOInit(void)                     
* ��������:RF4432 IO Init
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void RF4332_IOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_12;		 //SDN��CS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);   //PB1��12    
	
	/***************PB13---RF4322�ⲿ�ж�����**************/
	EXTI_InitTypeDef        EXTI_InitStructure;           	//�ⲿ�жϽṹ�����
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);  //PB13��Ϊ�ⲿ�жϽ�
  //EXTI_ClearITPendingBit(EXTI_Line0);					//���0�߱�־λ
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;              //�ⲿ�ж���13
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      //ģʽΪ=�ж�ģʽ
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //����Ϊ�½��ش����ж�
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                //ʹ���ⲿ�ж�
  EXTI_Init(&EXTI_InitStructure);                          //����ȷ������
   
 /**********�����Ƕ��жϽ�������(Priority)***********/
	NVIC_InitTypeDef        NVIC_InitStructure; 
  NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;          //�ж�ͨ��ѡ��=�ⲿ�ж���0
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);         //�ж����ȼ��������2��������
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ�����0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;        //��Ӧ���ȼ�����3
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;           //ʹ���жϹ�������
  NVIC_Init(&NVIC_InitStructure);   //���ݲ���ȷ������
	
}


/*��������:TB31371_IOInit(void)                     
* ��������:TB31371 IO Init
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void TB31371_IOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_0;		 //AMFM��EN��RSSI
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
	/*Data���Ŵ�����*/
#if 0
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);  //TIM3������ȫ��ӳ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimerBaseStruct.TIM_ClockDivision = 0; //ʱ�ӷָ�
  TIM_TimerBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;  //����������
  TIM_TimerBaseStruct.TIM_Prescaler = 71;   //Timer clock = sysclock /(TIM_Prescaler+1) = 1MHZ
  TIM_TimerBaseStruct.TIM_RepetitionCounter = 0;
  TIM_TimerBaseStruct.TIM_Period = 0xFFFF;    //��������65535 
  TIM_TimeBaseInit(TIM3,&TIM_TimerBaseStruct);
	
	
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	//TIM_ICInitStructure.TIM_ICMode = TIM_ICMode_ICAP;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	//TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   //�����½��ز���
  //TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		
	
	
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     //ÿ�μ�⵽��������ʹ���һ�β���
  TIM_ICInitStructure.TIM_ICFilter = 0x0;       //ѡ������Ƚ��˲������˲����ã������������������϶������ȶ�0x0��0xF
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_SelectInputTrigger(TIM3, TIM_TS_ITR3);

   //TIM enable counter 
   TIM_Cmd(TIM3, ENABLE);
   //Enable the CC2 Interrupt Request
	
   TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	 	 
	 NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);          //�ж����ȼ��������2��������
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  //��ռ���ȼ�����0
   NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;         //��Ӧ���ȼ�����0
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
#endif
}



/*��������:Extic8_init(void)                     
* ��������:Set C8 as external Interrupt Pin
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void Extic8_init(void)
{
	/*
	AFIO->EXTICR[0] &= ~0x0f;
	AFIO->EXTICR[0] |=  0x01;		//pb0 

	EXTI->IMR	  |=  0x01;			  //MR0 
	EXTI->FTSR	  |=  0x01;			//�½��� 

	NVIC->IP[6]   = 0x80;
	NVIC->ISER[0] = (uint32_t)0x01<<6;
	//NVIC->ICER[0] = (uint32_t)0x01<<6;
	*/
	
  GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;        //C��Pin8���ų�ʼ�� 
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;    	 //����Ϊ50MHZ
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;       	 //��������
  GPIO_Init(GPIOC,&GPIO_InitStructure);  
		
	EXTI_InitTypeDef        EXTI_InitStructure;           	//�ⲿ�жϽṹ�����
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource8);  //PC8��Ϊ�ⲿ�жϽ�
  EXTI_ClearITPendingBit(EXTI_Line8);					//���8�߱�־λ
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;              //�ⲿ�ж���8
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      //ģʽΪ=�ж�ģʽ
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//Falling;  //���ô�������
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                //ʹ���ⲿ�ж�
  EXTI_Init(&EXTI_InitStructure);                          //����ȷ������
   
 /**********�����Ƕ��жϽ�������(Priority)***********/
	NVIC_InitTypeDef        NVIC_InitStructure; 
  NVIC_InitStructure.NVIC_IRQChannel=EXTI9_5_IRQn;          //�ж�ͨ��ѡ��=�ⲿ�ж���0
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);         //�ж����ȼ��������2��������
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ�����0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;        //��Ӧ���ȼ�����3
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;           //ʹ���жϹ�������
  NVIC_Init(&NVIC_InitStructure);  
	
}


/*��������:EX_Flash_Init(void)                     
* ��������:External Flash IO Init
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void EX_Flash_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		 				  		//SFlash CS---PB(2)/BOOT1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 	
}


/*��������:LED_IOinit(void)                     
* ��������:R G B LED Init
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void LED_IOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  IonLED=1;                //������ЧLED����������
  
  
	LED_B=1;
	LED_R=0;
	delay_ms(200);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure); 
//	Fan_Ctr=0;      //FAN ctrl
}


/*��������:USART1_Init(void)                    
* ��������:USART1 Config
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;        	//USART1-TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;      	 	//USART1-RX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    	//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  USART_InitTypeDef       USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;      //������9600
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
  USART_InitStructure.USART_StopBits = USART_StopBits_1;  //1��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;     //��żʧ��
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //Ӳ��������ʧ��
  USART_InitStructure.USART_Mode = USART_Mode_Tx;  		//����ʹ��  
  
	USART_ClockInitTypeDef  USART_ClockInitStructure;
  USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;   //����ʱ��Ϊ�͵�ƽ
  USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge; //ʱ�ӵڶ������ؽ������ݲ���
  USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���
  
  USART_ClockInit(USART1, &USART_ClockInitStructure);  //��ʼ��ʱ������
  USART_Init(USART1,&USART_InitStructure);	//��ʼ���ṹ������
   
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�������2	 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//���ô����ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//ռ��ʽ���ȼ�����Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ�����Ϊ3
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;    //�ж�ʹ��
  NVIC_Init(&NVIC_InitStructure);  //�жϳ�ʼ��
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);  //ʹ��USART1�ж�
  USART_Cmd(USART1, ENABLE);  		 //����USART1
  
  USART_GetFlagStatus(USART1, USART_FLAG_TC);    //�ȶ�һ��SR�Ĵ���������ֱ�ӽ�����д��DR��һ���ֽڻᷢ��ʧ�ܡ�
}


/*��������:Signal_IOInit(void)                    
* ��������:Signal Input IO Init
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void Signal_IOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;		 //74HC4051 ISO_2��ISO_1��ISO_0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}


/*��������:CAN_Config(void)                    
* ��������:CAN Communication Config
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void CAN_Config(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 static byte CAN_BaudRate=0;
   
   
   CAN_BaudRate = *(byte *)BaudRate_Add;  //��ȡCAN������
   
	//***CAN-TX  IO Init***//
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//�����������
 	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	 //***CAN-RX  IO Init***//
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//��������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	 CAN_InitTypeDef CAN_InitStructure;
	 CAN_DeInit(CAN1);		//��ʼ����
	 CAN_StructInit(&CAN_InitStructure);
	 
	 CAN_InitStructure.CAN_TTCM=DISABLE;           // ʱ�䴥��ͨ�Ž�ֹ
 	 CAN_InitStructure.CAN_ABOM=ENABLE;            // һ��Ӳ����⵽128��11λ����������λ�����Զ��˳�����״̬
 	 CAN_InitStructure.CAN_AWUM=DISABLE;           // �Զ�����ģʽ������sleep
 	 CAN_InitStructure.CAN_NART=DISABLE;           // �Զ����´��ͱ��ģ�ֱ�����ͳɹ�
 	 CAN_InitStructure.CAN_RFLM=DISABLE;           // FIFOû���������±��ĸ��Ǿɱ���
 	 CAN_InitStructure.CAN_TXFP=DISABLE;           // ���ͱ������ȼ�ȷ������־��ȷ��
 	 CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   // ģʽ: CAN_Mode_Normal /CAN_Mode_LoopBack/CAN_Mode_Silent_LoopBack
	 CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;        // SWJ:(0-4)
if(0x64==CAN_BaudRate)        //100K
   {
     CAN_InitStructure.CAN_BS1=CAN_BS1_16tq;		//BS1:(0-16)
     CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;			//BS2:(0-8)
     CAN_InitStructure.CAN_Prescaler=18;        //��Ƶ��:([9:0]) CAN������={36MHZ/(1+16+3)}/18=100KB  (1+16)/(1+16+3)=85%
   }
else if(0x7d==CAN_BaudRate)  //125K
   {
     CAN_InitStructure.CAN_BS1=CAN_BS1_13tq;			 
     CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;				 
     CAN_InitStructure.CAN_Prescaler=18;
   }
else if(0xfa==CAN_BaudRate)  //250k
   {
     CAN_InitStructure.CAN_BS1=CAN_BS1_13tq;				 
     CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;				 
     CAN_InitStructure.CAN_Prescaler=9;
   }
else if(0x32==CAN_BaudRate)  //500K
   {
     CAN_InitStructure.CAN_BS1=CAN_BS1_14tq;				 
     CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;				 
     CAN_InitStructure.CAN_Prescaler=4; 
   }
else
  {
     CAN_InitStructure.CAN_BS1=CAN_BS1_13tq;				 
     CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;				 
     CAN_InitStructure.CAN_Prescaler=9;
  }
	 CAN_Init(CAN1, &CAN_InitStructure);       // ��ʼ��CAN1
	 CAN_FilterInitTypeDef CAN_FilterInitStructure;
	 CAN_FilterInitStructure.CAN_FilterNumber=0;			//������0
	 CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //����λģʽ
	 CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	 CAN_FilterInitStructure.CAN_FilterIdHigh=((0x18ff1234<<3)>>16)&0xffff;;				//������չ֡ID��λ
	 CAN_FilterInitStructure.CAN_FilterIdLow=(halfword)(0x18ff1234<<3);;				//��չ֡ID��λ
	 CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	 CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xfff8;				//ID������ȫƥ��
	 CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //������0
	 CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			 //ENABLE;
	 CAN_FilterInit(&CAN_FilterInitStructure);
	 
	 CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //�ر�FIFO0�ж�
	 
	 NVIC_InitTypeDef NVIC_InitStructure;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // �����ȼ�Ϊ0
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // �����ȼ�Ϊ0
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
}


/*��������:OtherIO_Init(void)                     
* ��������:Other IO Init
* �������:NO
* �������:NO
* �� �� ֵ:NO
* ˵    ��:NO
*/

void OtherIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_9;		 			//Buzzer  ALARM
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}


/*��������:ADC1_DMA_Config(void)                    
* ��������:ADC Config
* �������:no
* �������:no
* �� �� ֵ:no
* ˵    ��:no
*/

void ADC1_DMA_Config(void)
{
  ADC_InitTypeDef ADC_InitStructure;			//����ADC�ṹ��
  DMA_InitTypeDef DMA_InitStructure;			//����DMA�ṹ��  
  GPIO_InitTypeDef GPIO_InitStructure;		//����IO�ṹ��

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//ʹ��DMA1ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1| RCC_APB2Periph_GPIOA, ENABLE ); //ʹ��ADC1��GPIOAʱ��
  //PA0,1,2����Ϊģ������

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ģ������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*DMA1��ͨ��1����*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//�����Դͷ��ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;//Ŀ���ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //������Դͷ
  DMA_InitStructure.DMA_BufferSize = 20;	           //���ݳ���Ϊ20
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ���������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           //�ڴ��ַ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���贫�����ֽ�Ϊ��λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;        //�ڴ��԰���Ϊ��λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;			//ѭ��ģʽ
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//4���ȼ�֮һ��(������)
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 				//���ڴ浽�ڴ�
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);				//�������ϲ�����ʼ��DMA_InitStructure

  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);		//�ر�DMA�ж�
  DMA_Cmd(DMA1_Channel1, ENABLE);     //ʹ��DMA1
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    //ADC1�����ڶ���ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;			    //ģ��ת��������ɨ��ģʽ����ͨ����
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;    //ģ��ת������������ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת��������������ⲿ��������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						 //ADC�����Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = 4; //ת����ADCͨ������ĿΪ3
  ADC_Init(ADC1, &ADC_InitStructure);			

  //����ADC1��2��������ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
  //ת��ʱ��Tconv=����ʱ��+71.5������
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_71Cycles5); //CO
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5); //Fire1 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_71Cycles5); //Fire2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 4, ADC_SampleTime_71Cycles5); //Ion
  ADC_DMACmd(ADC1, ENABLE);   //ʹ��ADC1��DMA���䷽ʽ
  ADC_Cmd(ADC1, ENABLE);      //ʹ��ADC1  
  ADC_ResetCalibration(ADC1); //����ADC1��У׼�Ĵ���
  while(ADC_GetResetCalibrationStatus(ADC1)); //��ȡADC����У׼�Ĵ�����״̬
  ADC_StartCalibration(ADC1); 						  //��ʼУ׼ADC1
  while(ADC_GetCalibrationStatus(ADC1));   //�ȴ�У׼���
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);  //ʹ��ADC1���ת��
}






