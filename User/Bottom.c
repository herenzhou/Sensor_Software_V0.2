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

/*函数名称:delay(unsigned char z)                     
* 函数介绍:简单的延时函数
* 输入参数:z
* 输出参数:NO
* 返 回 值:NO
*/

void delay(unsigned short z)
{
  unsigned short x,y;
  for(x=200;x>0;x--)
     for(y=z;y>0;y--);
}


/*名   称：delay_us(u32 nus)
* 功   能：微秒延时函数
* 入口参数：u32  nus
* 出口参数：无
* 说    明：
* 调用方法：无 
*/ 

void delay_us(u32 nus)
{
	 u32 temp;
	 SysTick->LOAD = 9*nus;
	 SysTick->VAL=0x00;//清空计数器
	 SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
	 do
	 {
	  temp=SysTick->CTRL;//读取当前倒计数值
	 }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
	 
	 SysTick->CTRL=0x00; //关闭计数器
	 SysTick->VAL =0X00; //清空计数器
}


/*名    称：delay_ms(u16 nms)
* 功    能：毫秒延时函数
* 入口参数：u16 nms
* 出口参数：无
* 说    明：
* 调用方法：无 
*/

void delay_ms(u16 nms)
{
     //注意 delay_ms函数输入范围是1-1863
	 //所以最大延时为1.8秒

	 u32 temp;
	 SysTick->LOAD = 9000*nms;
	 SysTick->VAL=0X00;//清空计数器
	 SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
	 do
	 {
	  temp=SysTick->CTRL;//读取当前倒计数值
	 }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
	 SysTick->CTRL=0x00; //关闭计数器
	 SysTick->VAL =0X00; //清空计数器
}

/*函数名称:Delay_Second(byte time)                     
* 函数介绍:精确延时s
* 输入参数:time
* 输出参数:no
* 返 回 值:no
* 说    明:此函数只能用于初始化部分
*/

void Delay_Second(byte time)
{
	byte WaitTime=0;
	do
	{
		SysTick->CTRL &= 0xffffff0;      //关闭SysTick 时钟=系统时钟/8  禁止中断
		//Clearflag = SysTick->CTRL;         //clear flag 
		SysTick->VAL = 0x0;               //清空计数器
		//time = time * 1000;
		SysTick->LOAD = 9000*1000*1;       //1S的延时时基
		SysTick->CTRL |= 0x00000001;       //ENABLE，开始计数	Disable Interrupt
		while(0==(SysTick->CTRL>>16));		 //等待延时时间到
		SysTick->CTRL &= 0xffffff0;        //关闭定时器
		WaitTime++;
	}while(WaitTime<time);
	/*
	SysTick_Config(SystemCoreClock / (1000*1000));  //1s延时
	while(!(SysTick->CTRL>>16));
	SysTick->CTRL &= 0xffffff0;        //关闭定时器
	*/
}


/*函数名称:DS18B20_IO_IN(void)                     
* 函数介绍:DBS18B20 IO输入配置
* 输入参数:无
* 输出参数:无
* 返 回 值:无
*/ 

void DS18B20_IO_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin=DS18B20_Pin;			//DS18B20 IO口设置
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}


/*函数名称:DS18B20_IO_OUT(void)                     
* 函数介绍:DBS18B20 IO输出配置
* 输入参数:无
* 输出参数:无
* 返 回 值:无
*/ 

void DS18B20_IO_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin=DS18B20_Pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}


/*函数名称:DS18B20_Reset(void)                     
* 函数介绍:复位DS18B20
* 输入参数:无
* 输出参数:无
* 返 回 值:无
*/ 

unsigned char DS18B20_Reset(void)
{
	static unsigned char x=0;
	DS18B20_IO_OUT();//输出
	DQW=1;
	delay_us(10);
	DQW=0;
	delay_us(500);		//延时480微妙	
	DQW=1;
	delay_us(300);		//延时480微妙	
	DS18B20_IO_IN();	//输入
	x=DQR;
	delay_us(20);
	DS18B20_IO_OUT();
	DQW=1;
	return x;
}


/*函数名称:wr_one_bit(unsigned char Data)                     
* 函数介绍:向总线写1位数据
* 输入参数:unsigned char Data
* 输出参数:无
* 返 回 值:无
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


/*函数名称:wr_ds18b20(unsigned char BYTE)                     
* 函数介绍:向总线写1Byte数据
* 输入参数:unsigned char BYTE
* 输出参数:无
* 返 回 值:无
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


/*函数名称:Read_Bit(void)                     
* 函数介绍:向总读1位数据
* 输入参数:无
* 输出参数:无
* 返 回 值:ret
*/ 

unsigned char Read_Bit(void)
{
  unsigned char ret;
  
  asm("CPSID I");
  
	DS18B20_IO_OUT();		//输出
	DQW=0;  
	delay_us(2);
	DQW=1;
	delay_us(7);
	DS18B20_IO_IN();	//输入
  ret=DQR;						//读时隙产生7 us后读取总线数据。把总线的读取动作放在15us时间限制的后面是为了保证数据读取的有效性
  delay_us(60);			//延时60us，满足读时隙的时间长度要求 
	DS18B20_IO_OUT();	//输出
  DQW=1;	//释放总线 
  
  asm("CPSIE  I");
  
  return ret; 			//返回读取到的数据 
}


/*函数名称:rd_two_bits(void)                     
* 函数介绍:从总线上读2bit数据
* 输入参数:无
* 输出参数:无
* 返 回 值:Data
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


/*函数名称:DS18B20_Read_Byte(void)                     
* 函数介绍:从总线上读1Byte数据
* 输入参数:无
* 输出参数:无
* 返 回 值:无
*/

unsigned char DS18B20_Read_Byte(void)
{
	unsigned char i=0,TempData=0;
  
  asm("CPSID I");
  
	for(i=0;i<8;i++)
	{
		TempData>>=1;

		DS18B20_IO_OUT();//输出
		DQW=0;	 //拉低
		delay_us(4);//延时4微妙
		DQW=1;
		delay_us(10);//延时10微妙
		DS18B20_IO_IN();

		if(DQR==1)
		{
		   TempData|=0x80;//读数据 从低位开始
		}
		delay_us(45);//延时45微妙
	}
  asm("CPSIE  I");
  
	return TempData;
}


/*函数名称:OpenClock(void)                     
* 函数介绍:开启需要使用的各个模块的时钟
* 输入参数:无
* 输出参数:无
* 返 回 值:无
*/

void OpenClock(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);   //开启APB2总线上TIME2时钟(TIME2属于APB2总线)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);   
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);		//开启CAN1时钟
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);     //开启APB2总线上TIME5时钟(TIME2属于APB2总线)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    //使能GPIO及功能复用IO时钟
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);			  //STemwin需要此时钟,否则无法启动
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
  											 RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE ,ENABLE);  //开启IO口A、B、C、D、E端口的时钟
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
}


/*函数名称:IWDG_Init(byte prer,halfword rlr)                     
* 函数介绍:Set WatchDog And Enable It 
* 输入参数:prer,rlr  
* 输出参数:无
* 返 回 值:无
* 说    明:输入参数prer是分频，rlr装载初值， Time=((4*2^prer)*rlr)/40MS  rlr[11:0] prer[2:0] 4*2^prer=(0-256)
*/

void IWDG_Init(byte prer,halfword rlr)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);   //使能对寄存器IWDG_PR和IWDG_RLR的写操作
  IWDG_SetPrescaler(prer);    //设置IWDG预分频值:设置IWDG预分频值
  IWDG_SetReload(rlr);        //设置IWDG重装载值
  IWDG_ReloadCounter();       //按照IWDG重装载寄存器的值重装载IWDG计数器
  IWDG_Enable();              //使能IWDG
}


/*函数名称:IWDG_Feed(void)                     
* 函数介绍:Feed WatchDog
* 输入参数:无
* 输出参数:无
* 返 回 值:无
* 说    明:无
*/

void IWDG_Feed(void)
{
 	IWDG_ReloadCounter();    //Feed Dog
}


/*函数名称:TimeOut_10ms(void)                     
* 函数介绍:10ms定时中断
* 输入参数:无
* 输出参数:无
* 返 回 值:无
*/

void TimeOut_50ms(void)
{
  TIM_DeInit(TIM2);          //Time2定时器重设缺省值
  TIM_TimerBaseStruct.TIM_Period=99;      //设置重载寄存器初值 (设置为99，即：定时50ms)
  TIM_TimerBaseStruct.TIM_Prescaler=35999;  //时钟分频数(72MHZ分频后时钟为2.0KHZ)
  TIM_TimerBaseStruct.TIM_ClockDivision=0;  //设置时钟分割(暂时未用)
  TIM_TimerBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;   //设置计数器向上计数模式
  TIM_TimeBaseInit(TIM2,&TIM_TimerBaseStruct);  //初始化设置
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;   //中断通道选择=TIM2中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级设置0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;       //响应优先级设置2
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;          //使能中断管理设置
  NVIC_Init(&NVIC_InitStructure);                        //传递参数确认设置
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);  //使能TIM2中断源
  TIM_Cmd(TIM2,ENABLE);                     //使能TIM2定时器	
}


/*函数名称:Time3Init(void)                     
* 函数介绍:Timer3 Init
* 输入参数:无
* 输出参数:无
* 返 回 值:无
*/

void Time3Init(void)
{
  TIM_DeInit(TIM3);          //Time3定时器重设缺省值
  TIM_TimerBaseStruct.TIM_Period=199;      //设置重载寄存器初值 (设置为199，即：定时100ms)
  TIM_TimerBaseStruct.TIM_Prescaler=35999; //时钟分频数(72MHZ分频后时钟为2.0MHZ)
  TIM_TimerBaseStruct.TIM_ClockDivision=0;  //设置时钟分割(暂时未用)
  TIM_TimerBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;   //设置计数器向上计数模式
  TIM_TimeBaseInit(TIM3,&TIM_TimerBaseStruct);  //初始化设置
	//TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;   //中断通道选择=TIM2中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级设置0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;       //响应优先级设置2
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;          //使能中断管理设置
  NVIC_Init(&NVIC_InitStructure);                        //传递参数确认设置
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);  //使能TIM3中断源

  TIM_Cmd(TIM3,ENABLE);                     //使能TIM3定时器	
	
/*
	RCC->APB1ENR  |= (uint32_t)0x01<<2;
	RCC->APB1RSTR |= (uint32_t)0x01<<2;
	RCC->APB1RSTR &= ~((uint32_t)0x01<<2);

	
	TIM4->CR1 = 0x0004;
	TIM4->CR2 = 0x0000;
	TIM4->DIER= 0x0001;		//中断控制
	TIM4->PSC = 36-1;			// 分频系数
	TIM4->ARR = 0xffff;		// 加载值
	
	TIM4->CR1 |=0x0001;		//启动

	NVIC->IP[30]  =  0x90;
	NVIC->ISER[0] = (uint32_t)0x01<<30;
	//NVIC->ICER[1] = (uint32_t)0x01<<30;
*/
}


/*函数名称:PWM_Set(void)                     
* 函数介绍:SET OUTPUT PWM
* 输入参数:无
* 输出参数:无
* 返 回 值:无
* 说    明:用于模拟蜂鸣器叫声
*/

void PWM_Set(void)
{ 
  TIM_DeInit(TIM4);          //Time4定时器重设缺省值
  TIM_TimerBaseStruct.TIM_Period=18;      //设置重载寄存器初值 (设置为60，即：定时20ms)
  TIM_TimerBaseStruct.TIM_Prescaler=719;  //时钟分频数(72MHZ分频后时钟为8.0KHZ)
  TIM_TimerBaseStruct.TIM_ClockDivision=0;  //设置时钟分割(暂时未用)
  TIM_TimerBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;   //设置计数器向上计数模式
  TIM_TimeBaseInit(TIM4,&TIM_TimerBaseStruct);  //初始化设置
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //选中pin3口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //I/O时钟为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);    //根据上面指定参数初始化GPIOA结构体
      
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        //选择PWM输出为模式1
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM输出比较极性高
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//通道1输出使能
  TIM_OCInitStructure.TIM_Pulse =9;  //脉宽值为9  即PWM=50%
  TIM_OC4Init(TIM4,&TIM_OCInitStructure); //根据TIM_OCInitStruct中指定的参数初始化TIM2
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能TIM4在CCR1上的预装载寄存器
  TIM_ClearITPendingBit(TIM4,TIM_IT_CC4);//预先清除所有中断位
  TIM_ARRPreloadConfig(TIM4, ENABLE);    //使能TIM4在ARR上的预装载寄存器
  TIM_Cmd(TIM4,ENABLE);                     //使能TIM4定时器
}


/*函数名称:Key_IOInit(void)                   
* 函数介绍:Touch Key IO Init
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void Key_IOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
//  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;   
//  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;    	 //速率为10MHZ
//  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;       	 //上拉输入
//  GPIO_Init(GPIOA,&GPIO_InitStructure);                //初始化设置	
  
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;              //光电红外发送管使能
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;    	 
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;       	 
  GPIO_Init(GPIOA,&GPIO_InitStructure);   
  IrdTx = 0;
  
#ifdef TemplineFuc
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;   
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;    	 
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;       	 //上拉输入
  GPIO_Init(GPIOB,&GPIO_InitStructure);
#endif
}


/*函数名称:SPI1_Init(void)                   
* 函数介绍:SPI1 Config
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;    //设置其速率为10MHZ
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;      //IO口设置为复用推挽输出
  GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化IO_A口 PIN5(SCK)、PIN6(MISO)、PIN7(MOSI)
	
	SPI_InitTypeDef         SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI为双线双向全双工
  SPI_Cmd(SPI1, DISABLE);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;      //设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;  //数据结构为8bit
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	       //时钟空闲时为0
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       //数据捕获于第一个时钟沿
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				   //内部NSS信号由软件控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;     //波特率预分频值为32
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;			     //数据传输从低位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;			             //CRC值计算的多项式最高为7次
  SPI_Init(SPI1, &SPI_InitStructure);   //根据以上参数初始化SPI结构体
  //SPI_SSOutputCmd(SPI1, ENABLE);		//Enable SPI1.NSS as a GPIO
  SPI_Cmd(SPI1, ENABLE);	 		//使能SPI1
}


/*函数名称:RF4332_IOInit(void)                     
* 函数介绍:RF4432 IO Init
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void RF4332_IOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_12;		 //SDN、CS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);   //PB1、12    
	
	/***************PB13---RF4322外部中断配置**************/
	EXTI_InitTypeDef        EXTI_InitStructure;           	//外部中断结构体变量
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);  //PB13脚为外部中断脚
  //EXTI_ClearITPendingBit(EXTI_Line0);					//清除0线标志位
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;              //外部中断线13
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      //模式为=中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //设置为下降沿触发中断
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                //使能外部中断
  EXTI_Init(&EXTI_InitStructure);                          //传参确定设置
   
 /**********下面是对中断进行配置(Priority)***********/
	NVIC_InitTypeDef        NVIC_InitStructure; 
  NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;          //中断通道选择=外部中断线0
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);         //中断优先级采用组别2进行配置
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级设置0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;        //响应优先级设置3
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;           //使能中断管理设置
  NVIC_Init(&NVIC_InitStructure);   //传递参数确认设置
	
}


/*函数名称:TB31371_IOInit(void)                     
* 函数介绍:TB31371 IO Init
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void TB31371_IOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_0;		 //AMFM、EN、RSSI
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
	/*Data引脚待配置*/
#if 0
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);  //TIM3功能完全重映射
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimerBaseStruct.TIM_ClockDivision = 0; //时钟分割
  TIM_TimerBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;  //计数器方向
  TIM_TimerBaseStruct.TIM_Prescaler = 71;   //Timer clock = sysclock /(TIM_Prescaler+1) = 1MHZ
  TIM_TimerBaseStruct.TIM_RepetitionCounter = 0;
  TIM_TimerBaseStruct.TIM_Period = 0xFFFF;    //计数上限65535 
  TIM_TimeBaseInit(TIM3,&TIM_TimerBaseStruct);
	
	
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	//TIM_ICInitStructure.TIM_ICMode = TIM_ICMode_ICAP;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	//TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   //上升下降沿捕获
  //TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		
	
	
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     //每次检测到捕获输入就触发一次捕获
  TIM_ICInitStructure.TIM_ICFilter = 0x0;       //选择输入比较滤波器，滤波设置，经历几个周期跳变认定波形稳定0x0～0xF
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_SelectInputTrigger(TIM3, TIM_TS_ITR3);

   //TIM enable counter 
   TIM_Cmd(TIM3, ENABLE);
   //Enable the CC2 Interrupt Request
	
   TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	 	 
	 NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);          //中断优先级采用组别2进行配置
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  //抢占优先级设置0
   NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;         //响应优先级设置0
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
#endif
}



/*函数名称:Extic8_init(void)                     
* 函数介绍:Set C8 as external Interrupt Pin
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void Extic8_init(void)
{
	/*
	AFIO->EXTICR[0] &= ~0x0f;
	AFIO->EXTICR[0] |=  0x01;		//pb0 

	EXTI->IMR	  |=  0x01;			  //MR0 
	EXTI->FTSR	  |=  0x01;			//下降沿 

	NVIC->IP[6]   = 0x80;
	NVIC->ISER[0] = (uint32_t)0x01<<6;
	//NVIC->ICER[0] = (uint32_t)0x01<<6;
	*/
	
  GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;        //C口Pin8引脚初始化 
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;    	 //速率为50MHZ
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;       	 //上拉输入
  GPIO_Init(GPIOC,&GPIO_InitStructure);  
		
	EXTI_InitTypeDef        EXTI_InitStructure;           	//外部中断结构体变量
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource8);  //PC8脚为外部中断脚
  EXTI_ClearITPendingBit(EXTI_Line8);					//清除8线标志位
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;              //外部中断线8
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      //模式为=中断模式
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//Falling;  //设置触发边沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                //使能外部中断
  EXTI_Init(&EXTI_InitStructure);                          //传参确定设置
   
 /**********下面是对中断进行配置(Priority)***********/
	NVIC_InitTypeDef        NVIC_InitStructure; 
  NVIC_InitStructure.NVIC_IRQChannel=EXTI9_5_IRQn;          //中断通道选择=外部中断线0
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);         //中断优先级采用组别2进行配置
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级设置0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;        //响应优先级设置3
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;           //使能中断管理设置
  NVIC_Init(&NVIC_InitStructure);  
	
}


/*函数名称:EX_Flash_Init(void)                     
* 函数介绍:External Flash IO Init
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void EX_Flash_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		 				  		//SFlash CS---PB(2)/BOOT1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 	
}


/*函数名称:LED_IOinit(void)                     
* 函数介绍:R G B LED Init
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
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
  IonLED=1;                //离子有效LED控制器引脚
  
  
	LED_B=1;
	LED_R=0;
	delay_ms(200);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure); 
//	Fan_Ctr=0;      //FAN ctrl
}


/*函数名称:USART1_Init(void)                    
* 函数介绍:USART1 Config
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;        	//USART1-TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	//复用输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;      	 	//USART1-RX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    	//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  USART_InitTypeDef       USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;      //波特率9600
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
  USART_InitStructure.USART_StopBits = USART_StopBits_1;  //1个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;     //奇偶失能
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //硬件流控制失能
  USART_InitStructure.USART_Mode = USART_Mode_Tx;  		//接收使能  
  
	USART_ClockInitTypeDef  USART_ClockInitStructure;
  USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;   //空闲时钟为低电平
  USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge; //时钟第二个边沿进行数据捕获
  USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
  
  USART_ClockInit(USART1, &USART_ClockInitStructure);  //初始化时钟设置
  USART_Init(USART1,&USART_InitStructure);	//初始化结构体配置
   
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //采用组别2	 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//配置串口中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//占先式优先级设置为0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //副优先级设置为3
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;    //中断使能
  NVIC_Init(&NVIC_InitStructure);  //中断初始化
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);  //使能USART1中断
  USART_Cmd(USART1, ENABLE);  		 //开启USART1
  
  USART_GetFlagStatus(USART1, USART_FLAG_TC);    //先读一次SR寄存器，否则直接将数据写入DR第一个字节会发送失败。
}


/*函数名称:Signal_IOInit(void)                    
* 函数介绍:Signal Input IO Init
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void Signal_IOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;		 //74HC4051 ISO_2、ISO_1、ISO_0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}


/*函数名称:CAN_Config(void)                    
* 函数介绍:CAN Communication Config
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void CAN_Config(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 static byte CAN_BaudRate=0;
   
   
   CAN_BaudRate = *(byte *)BaudRate_Add;  //读取CAN波特率
   
	//***CAN-TX  IO Init***//
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//复用推挽输出
 	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	 //***CAN-RX  IO Init***//
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//浮空输入
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	 CAN_InitTypeDef CAN_InitStructure;
	 CAN_DeInit(CAN1);		//初始设置
	 CAN_StructInit(&CAN_InitStructure);
	 
	 CAN_InitStructure.CAN_TTCM=DISABLE;           // 时间触发通信禁止
 	 CAN_InitStructure.CAN_ABOM=ENABLE;            // 一旦硬件检测到128次11位连续的隐性位，则自动退出离线状态
 	 CAN_InitStructure.CAN_AWUM=DISABLE;           // 自动唤醒模式：清零sleep
 	 CAN_InitStructure.CAN_NART=DISABLE;           // 自动重新传送报文，直到发送成功
 	 CAN_InitStructure.CAN_RFLM=DISABLE;           // FIFO没有锁定，新报文覆盖旧报文
 	 CAN_InitStructure.CAN_TXFP=DISABLE;           // 发送报文优先级确定：标志符确定
 	 CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   // 模式: CAN_Mode_Normal /CAN_Mode_LoopBack/CAN_Mode_Silent_LoopBack
	 CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;        // SWJ:(0-4)
if(0x64==CAN_BaudRate)        //100K
   {
     CAN_InitStructure.CAN_BS1=CAN_BS1_16tq;		//BS1:(0-16)
     CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;			//BS2:(0-8)
     CAN_InitStructure.CAN_Prescaler=18;        //分频数:([9:0]) CAN波特率={36MHZ/(1+16+3)}/18=100KB  (1+16)/(1+16+3)=85%
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
	 CAN_Init(CAN1, &CAN_InitStructure);       // 初始化CAN1
	 CAN_FilterInitTypeDef CAN_FilterInitStructure;
	 CAN_FilterInitStructure.CAN_FilterNumber=0;			//过滤组0
	 CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //屏蔽位模式
	 CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	 CAN_FilterInitStructure.CAN_FilterIdHigh=((0x18ff1234<<3)>>16)&0xffff;;				//设置扩展帧ID高位
	 CAN_FilterInitStructure.CAN_FilterIdLow=(halfword)(0x18ff1234<<3);;				//扩展帧ID低位
	 CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	 CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xfff8;				//ID不必完全匹配
	 CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //缓冲器0
	 CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			 //ENABLE;
	 CAN_FilterInit(&CAN_FilterInitStructure);
	 
	 CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //关闭FIFO0中断
	 
	 NVIC_InitTypeDef NVIC_InitStructure;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级为0
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // 次优先级为0
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
}


/*函数名称:OtherIO_Init(void)                     
* 函数介绍:Other IO Init
* 输入参数:NO
* 输出参数:NO
* 返 回 值:NO
* 说    明:NO
*/

void OtherIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_9;		 			//Buzzer  ALARM
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}


/*函数名称:ADC1_DMA_Config(void)                    
* 函数介绍:ADC Config
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
*/

void ADC1_DMA_Config(void)
{
  ADC_InitTypeDef ADC_InitStructure;			//定义ADC结构体
  DMA_InitTypeDef DMA_InitStructure;			//定义DMA结构体  
  GPIO_InitTypeDef GPIO_InitStructure;		//定义IO结构体

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//使能DMA1时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1| RCC_APB2Periph_GPIOA, ENABLE ); //使能ADC1及GPIOA时钟
  //PA0,1,2配置为模拟输入

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*DMA1的通道1配置*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//传输的源头地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;//目标地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //外设作源头
  DMA_InitStructure.DMA_BufferSize = 20;	           //数据长度为20
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不递增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           //内存地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设传输以字节为单位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;        //内存以半字为单位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;			//循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//4优先级之一的(高优先)
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 				//非内存到内存
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);				//根据以上参数初始化DMA_InitStructure

  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);		//关闭DMA中断
  DMA_Cmd(DMA1_Channel1, ENABLE);     //使能DMA1
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    //ADC1工作在独立模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;			    //模数转换工作在扫描模式（多通道）
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;    //模数转换工作在连续模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						 //ADC数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 4; //转换的ADC通道的数目为3
  ADC_Init(ADC1, &ADC_InitStructure);			

  //设置ADC1的2个规则组通道，设置它们的转化顺序和采样时间
  //转换时间Tconv=采样时间+71.5个周期
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_71Cycles5); //CO
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5); //Fire1 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_71Cycles5); //Fire2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 4, ADC_SampleTime_71Cycles5); //Ion
  ADC_DMACmd(ADC1, ENABLE);   //使能ADC1的DMA传输方式
  ADC_Cmd(ADC1, ENABLE);      //使能ADC1  
  ADC_ResetCalibration(ADC1); //重置ADC1的校准寄存器
  while(ADC_GetResetCalibrationStatus(ADC1)); //获取ADC重置校准寄存器的状态
  ADC_StartCalibration(ADC1); 						  //开始校准ADC1
  while(ADC_GetCalibrationStatus(ADC1));   //等待校准完成
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);  //使能ADC1软件转换
}






