/**
  ******************************************************************************
  * @file    Logic.c
  * @author  Andy
  * @version V1.0
  * @date    6-11-2015
  * @brief   Logic fuction
  ******************************************************************************/

#include "MyHeader.h"



short temparray[10]={0};
unsigned char DS18B20ID[10][8]={0};
SorState StateByte={0x00};
word FlashBuffer1[2]={0x5555aaaa,CAN_ExID};   //Flash标识，CAN ID
word FlashBuffer2[2]={0xaaaa5555,Parameter};  //Flash标志
word FlashBuffer3[2]={0x5a5a5a5a,0};          //Flash标识，Ion初值
byte HiTempFlag=0;
byte TempLineSort=0;


/*函数名称:unsigned char SearchRomID(unsigned char RomID[16][8])                     
* 函数介绍:搜索总线上DS18B20的ID及个数
* 输入参数:无
* 输出参数:无
* 返 回 值:DS18B20在总线上的个数
*/

unsigned char SearchRomID(unsigned char RomID[10][8])
{
	static unsigned char Rom[64];
	static unsigned char c,i,j,R1,R2,a,b,d;
	static unsigned char _00wbit[10]={2,2,2,2,2,2,2,2,2,2}; //初始化00写位组全部为填充位2
  
#if MAXNUM>10|MAXNUM==0
#error "MAXNUM value between 1-10"
#endif
  
	for(i=0;i<MAXNUM;i++)
	{
		c=0;
		DS18B20_Reset();
		wr_ds18b20(0xf0);
		for(j=0;j<64;j++)
			{
				R1=Read_Bit();
				delay_us(2);
				R2=Read_Bit();		//读补码
				delay_us(2);
				if(R1==0&&R2==1)  //未出现数据冲突，主机写0
					{
						Rom[j]=0;
						wr_one_bit(0);
					}
				else if(R1==1&&R2==0)  //未出现数据冲突，主机写1
					{
						Rom[j]=1;
						wr_one_bit(1);
					}
				else
					{
						if(_00wbit[c]==2) //出现新00写位
							{
								wr_one_bit(0);
								Rom[j]=0;
								_00wbit[c]=0; //新00写位赋值为0
								c+=1;
							}
						else if(_00wbit[c]==1)//00写位组中00写位为1，主机写1
							{
								wr_one_bit(1);
								Rom[j]=1;
								c+=1;
							}
						else if(_00wbit[c]==0) //00写位组中00写位为0，主机写0
							{
								wr_one_bit(0);
								Rom[j]=0;
								c+=1;
							}
					}
			}
		for(j=0;j<64;j+=8)  //将64位ROM编码整理成8字节存入RomID[n][8]中
			{
				for(d=0;d<8;d++)
					{
						if(Rom[j+d]&0x01)
							{
								RomID[i][j/8]>>=1;
								RomID[i][j/8]|=0x80;
							}
						else RomID[i][j/8]>>=1;
					}
			}
		for(a=0,c=MAXNUM;c>0;c--)  //更新00写位数组   c>=0
			{
				//a=0;
				if(_00wbit[c]==2) //跳过00写位组中的填充位
				continue;
				if(_00wbit[c]==0&&a==0)//更改最高00写位并跳出
					{
						_00wbit[c]=1;
						break;
					}
				else if(_00wbit[c]==1)//最高00写位为1
					{
						if(c!=0) 		//为1的00写位不为00写位组的最低位
							{
								a+=1;		//记录不为00写位组的最低位且为1的连续00写位个数
								continue;
							}
						else
							{
								b=1;  //00写位组全部为1，搜索结束置标志位
								break;
							}
					}
				else if(_00wbit[c]==0&&a!=0)//连续为1的00写位后第一个为0的00写位
					{
						_00wbit[c]=1;//赋次高00写位为1
						for(;a>0;a--)//连续弃去为1的最高00写位
							{
								c+=1;
								_00wbit[c]=2;
							}
						break;
					}
			}
		if(b==1) 
			break; 				//搜索结束标志位为1跳出
	}
	return(i+1);			//返回总线上器件个数
}



/*函数名称:ds18b20_crc_ensure(void)                    
* 函数介绍:ensure ds18b20 id is right
* 输入参数:*p,len
* 输出参数:no
* 返 回 值:crc
* 说    明:if ds18b20 id crc check is ok,record it.
*/
#if 0
void ds18b20_crc_ensure(void)
{
	byte CRCValue=0;
	byte i,j=0;
	word value1,value2=0;
	for(i=0;i<8;i++)
	{
		CRCValue = do_crc8(&ID[i][0],7);  //Get CRC value 
		if(CRCValue==ID[i][7])    //If CRC check is OK
			{
				value1=(DS18B20ID[i][0]<<6)|(DS18B20ID[i][1]<<4)|(DS18B20ID[i][2]<<2)|(DS18B20ID[i][3]);		  //记住ID到Flash
				value2=(DS18B20ID[i][4]<<6)|(DS18B20ID[i][5]<<4)|(DS18B20ID[i][6]<<2)|(DS18B20ID[i][7]);
				Flash_Write_Data(Temp1Add,value1);
				Flash_Write_Data(Temp1Add,value2);
			}	
		else{;}
	}
}
#endif

/*函数名称:Change_Resolution_ds18b20(void)                    
* 函数介绍:Change Resolution ds18b20 9bit
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:第一次上电修改ds18b20的精度
*/

void Change_Resolution_ds18b20(void)
{
	if(1)				//如果ds18b20的精度已经修改过
	{
		return;  //退出
	}
	else				//如果精度没有改变
	{
		DS18B20_Reset();         //Reset
		wr_ds18b20(0xcc);        //跳过读序列号
		wr_ds18b20(0X4e);				 //写暂存指令
		wr_ds18b20(0x02);     		 //写报警值上限TH
		wr_ds18b20(0x02);					 //写报警值下限TL
		wr_ds18b20(0x1f);					 //写设置分辨率 9位
		//写flash，标注精度已经更改过了
	}
}


/*函数名称:DS18B20_ReadDesignateTemper(void)                     
* 函数介绍:循环读取温度
* 输入参数:no
* 输出参数:no
* 返 回 值:data
* 说    明:读取序列号指定的ID的传感器温度
*/

void DS18B20_ReadDesignateTemper(void)  
{  
    static unsigned char th,tl;  
    static short  data;         //温度可能为负数
		unsigned char N,Count=0;
 
		DS18B20_Reset();         //Reset
		//delay_ms(100);
		wr_ds18b20(0xcc);        //跳过读序列号
		wr_ds18b20(0x44);        //启动温度转换 
		delay_ms(750);
		for(N=0;N<MAXNUM;N++)
		{
			DS18B20_Reset();         //Reset
			//wr_ds18b20(0xcc);        //跳过读序列号
			//wr_ds18b20(0X4e);				 //写暂存指令
			//wr_ds18b20(0x02);     		 //写报警值上限TH
			//wr_ds18b20(0x02);					 //写报警值下限TL
			//wr_ds18b20(0x1f);					 //写设置分辨率 9位
			//wr_ds18b20(0xcc);        //跳过读序列号  
			//wr_ds18b20(0x44);        //启动温度转换 
			//DS18B20_Reset();  
			wr_ds18b20(0x55);        //发送序列号匹配命令  
			for(Count=0;Count<8;Count++)   //发送8byte的序列号     
			{  
				 wr_ds18b20(DS18B20ID[N][Count]);  
			}   
			//delay_ms(200);
			wr_ds18b20(0xbe);    //读取温度  
			tl = DS18B20_Read_Byte();    //读取低八位  
			th = DS18B20_Read_Byte();    //读取高八位  
			data = th;  
			data <<= 8;  
			data |= tl;                  
			if(0xf800==(data&0xf800))   	//负温度判断
			{
				data=(~data)+1;
				data *= 0.0625;
				data=0-data;					//负温度实际值
			}else{data *= 0.0625;}   //实际温度值	
      if((data!=0)&&(data!=0x55)&&(data<0x7d))       //温度取值必须在18b20测试范围内(<125)，并且不为85度
			temparray[N]=data;
//			delay_ms(50);
		} 
		TempOverFlag=0x55;     //启动一次转换标志
}


/*函数名称:void searchHighTemp(void)                     
* 函数介绍:no
* 输入参数:no
* 输出参数:no
* 返 回 值:hightemp
* 说    明:no
*/

byte searchHighTemp(void)
{
	byte i=0;
	short hightemp=-Tempoffset;
	static byte HiTempCount=0;
//	static byte Sdtempcnt=0;
  
#if (MAXNUM==0)||(MAXNUM>10)
#error "MAXNUM value between 1 to 10"
#endif
	
	for(i=0;i<MAXNUM;i++)
		{
			if(temparray[i]>hightemp)
				hightemp=temparray[i];			
		}
  
	SdTempValue=hightemp;    //新的突变温度
  if(0==(WarningMaskBit&0x04))
  {
    if(hightemp>=TempLimit)   
      {
        HiTempCount++;
        if(HiTempCount>=5)
          {
            HiTempCount=5;
//            HiTempFlag=0x55;
            CAN_Byte3.Bits.HiTemp=1;
          }
      }else{HiTempCount=0;CAN_Byte3.Bits.HiTemp=0;}
  }else{CAN_Byte3.Bits.HiTemp=0;}
	
	hightemp += Tempoffset;
	return hightemp;
}


/*函数名称:void ChenckIon(void)                     
* 函数介绍:no
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:检测离子传感器是否报警
*/

void ChenckIon(void)
{
#if IonTypedef==1
  static byte inocnt=0;
#elif IonTypedef==2
  static byte ionPowercnt=0;
  static byte IonCheckOK=0;
#endif
  
  halfword IonADValue = 0;
  
  
  IonADValue = (ADCConvertedValue[3]+ADCConvertedValue[7]+ADCConvertedValue[11]+ADCConvertedValue[15]+ADCConvertedValue[19])/5;
  
#if IonTypedef==2
  if(ionPowercnt<30)    //上电检测3S
    {
      ionPowercnt++;
      if(IonADValue>=1800)
          IonCheckOK=0x55;
//      else
//          IonCheckOK=0;
    }
#endif
  
  
  
  if(0==(WarningMaskBit&0x01))
  {
#if IonTypedef==1
    if(IonADValue<=IonBotADLimit)       //老版本离子有效输出低电平
      {
        inocnt++;
        if(inocnt>=30)
          {
            inocnt=30;
            StateByte.Bits.Ion=1;   
          }
      }else{StateByte.Bits.Ion=0;inocnt=0;}
#elif IonTypedef==2
    if((IonCheckOK!=0x55)&&(ionPowercnt>=30))      //上电3S后开始检测
      {
        if(StateByte.Bits.CO==1)                          
          {
            if(IonADValue >= (*(word *)IonValueadd)+200)     //CO有效时  Ion=初值+200
              {
                StateByte.Bits.Ion=1;
                IonLED=0;
              }
            else{StateByte.Bits.Ion=0;IonLED=1;}
          }      
        else
          {
            if(IonADValue >= (*(word *)IonValueadd)+1000)    //CO无效时  Ion=初值+300
              {
                StateByte.Bits.Ion=1; 
                IonLED=0;
              }
            else{StateByte.Bits.Ion=0;IonLED=1;}
          }
      }else{StateByte.Bits.Ion=0;IonLED=1;}
#endif
  }else{StateByte.Bits.Ion=0;IonLED=1;}
}


/*函数名称:void ChenckCO(void)                     
* 函数介绍:no
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:检测CO传感器是否报警(co阀值1.6V)
*/

void CheckCO(void)
{
	unsigned short COADValue;
  static byte CoCnt=0;
  static byte CoOKFlag=0;
//  static byte  COWarningCnt=0;
  
	COADValue=(ADCConvertedValue[0]+ADCConvertedValue[4]+ADCConvertedValue[8]\
						+ADCConvertedValue[12]+ADCConvertedValue[16])/5;
	
  if(CoCnt<30)
    {
      CoCnt++;
      if(COADValue>=3850)
        CoOKFlag=0x55;
//      else
//        CoOKFlag=0;
    }
  
  if(0==(WarningMaskBit&0x02))
  {
    if((CoOKFlag!=0x55)&&(CoCnt>=30))
      {
        if(COADValue>=CO_ADLimit)
          {
            StateByte.Bits.CO=1;				//CO置位
          }else{StateByte.Bits.CO=0;}        
      }else{StateByte.Bits.CO=0;}
  }else{StateByte.Bits.CO=0;}
}


/*函数名称:void SmokeEnsure(void)                     
* 函数介绍:no
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
*/

void SmokeEnsure(void)
{
 #if IonTypedef==2
  static byte SmokeStatus=0;
#endif
  
  
  CheckCO();
	ChenckIon();
	
#ifdef SmokeEasy
  if((1==StateByte.Bits.CO)||(1==StateByte.Bits.Ion))   //Smoke=co|ion
      CAN_Byte2.Bits.Smoke=1;
  else{CAN_Byte2.Bits.Smoke=0;}
#else
#if IonTypedef==2
  if((1==StateByte.Bits.CO)&&(1==StateByte.Bits.Ion))   //Smoke=co+ion
    {
      SmokeStatus=0;
      CAN_Byte2.Bits.Smoke=1;
    }
  else
    {
      SmokeStatus++;
      if(SmokeStatus>=30)
        {
          SmokeStatus=30;
          CAN_Byte2.Bits.Smoke=0;
        } 
    }
#else
	if((1==StateByte.Bits.CO)&&(1==StateByte.Bits.Ion))   //Smoke=co+ion
    {
       CAN_Byte2.Bits.Smoke=1;
    }else{CAN_Byte2.Bits.Smoke=0;}
#endif
#endif
}


/*函数名称:void CheckFlame(void)                     
* 函数介绍:no
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:检测火焰标志是否置位
*/

void CheckFlame(void)
{
  static unsigned short Flame1Value;
  static byte FlameCnt1=0;
  static byte FireCnt=0;
  static byte FireOKFlag1=0;
  
#ifdef Fire2_Use
  static byte FlameCnt2=0;
  static byte Flame1Flag=0;
  static byte Flame2Flag=0;
  static byte FireOKFlag2=0;
  static unsigned short Flame2Value;
#endif
	
	Flame1Value=(ADCConvertedValue[1]+ADCConvertedValue[5]+ADCConvertedValue[9]\
							+ADCConvertedValue[13]+ADCConvertedValue[17])/5;
#ifdef Fire2_Use
	Flame2Value=(ADCConvertedValue[2]+ADCConvertedValue[6]+ADCConvertedValue[10]\
							+ADCConvertedValue[14]+ADCConvertedValue[18])/5;
#endif

  /**************上电2S检测*************/  
    if(FireCnt<=20)     
    {
      FireCnt++;
      if(Flame1Value>=3850)       //短路或者没下拉电阻
          FireOKFlag1=0x55;
//      else
//          FireOKFlag1=0;
#ifdef Fire2_Use
      if(Flame2Value>=3850)   
          FireOKFlag2=0x55;
//      else 
//          FireOKFlag2=0;
#endif
    }//else{FireCnt=11;}
    
    
#ifdef Fire2_Use
  if(0==(WarningMaskBit&0x10))
  {
     if((Flame2Value>=Flame_ADLimit)&&(FireOKFlag2!=0x55))
     {
        FlameCnt2++;
        if(FlameCnt2>=20)
          {
            FlameCnt2=20;
//            StateByte.Bits.Flame=1;   //火焰2S后生效
            Flame2Flag=0x55;
          }
     }else{Flame2Flag=0;FlameCnt2=0;}   
     
     if((Flame1Value>=Flame_ADLimit)&&(FireOKFlag1!=0x55))
     {
        FlameCnt1++;
        if(FlameCnt1>=20)
        {
          FlameCnt1=20;
          Flame1Flag=0x55;
        }
     }else{Flame1Flag=0;FlameCnt1=0;}
     
     /********火焰二者之一有效则有效*********/
     if((0x55==Flame1Flag)||(0x55==Flame2Flag))
        StateByte.Bits.Flame=1;
     else if((0==Flame1Flag)&&(0==Flame2Flag))
        StateByte.Bits.Flame=0;
  }else{StateByte.Bits.Flame=0;}
  
#else
  if(0==(WarningMaskBit&0x10))   
	{
		if((Flame1Value>=Flame_ADLimit)&&(FireOKFlag1!=0x55)) 
    {
      FlameCnt1++;
      if(FlameCnt1>=20)
         {
           FlameCnt1=20;
           StateByte.Bits.Flame=1;   
         }
    }else{StateByte.Bits.Flame=0;FlameCnt1=0;}							
	}else{StateByte.Bits.Flame=0;}
#endif
}


/*函数名称:void TempLineCheck(void)                     
* 函数介绍:no
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:感温线检测，上电延时一段时间(1min)，在这期间感温线不能短路，否则认为感温线失效
*/

void TempLineCheck(void)
{
//  static byte TempLineCheck=0;
  static byte TempWarningCnt=0;  
  static byte TemplineCnt=0;
  static byte TempLineCheckFlag=0;
  
  if(TemplineCnt<=TempLineTime)    //感温电缆上电检测60S内短路则认为感温线失效
    {
      TemplineCnt++;
      if((TempLineIO==0)&&(TempLineCheckFlag!=0x55))
        {
          TempLineCheckFlag=0x55;
        }
    }
  
  if((0==TempLineCheckFlag)&&(TemplineCnt>=TempLineTime))   //感温电缆校验OK
    {
      if(0==(WarningMaskBit&0x04))
      {
        if(0==TempLineIO)  
        {
          TempWarningCnt++;
          if(TempWarningCnt>=3)      //持续约3S才置标志
            {
              TempWarningCnt=3;
              TempLineSort=0x55;        //感温线短路
            }
        }else{TempLineSort=0;TempWarningCnt=0;}
      }else{TempLineSort=0;}    
    }else{TempLineSort=0;}
}


/*函数名称:void Flash_Wr(void)                     
* 函数介绍:Flash Erase and Write
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
*/

void Flash_Wr1(void)
{
	byte i=0;
	word WrAdd = FlashWadd;
	
	if(*(word *)FlashWadd!=0x5555aaaa)  //写入初始数据
	{
		asm("CPSID I");    //Flash Operate Should Disable All Interrupt	
		FLASH_Status FLASHstatus = FLASH_COMPLETE;
		RCC_HSICmd(ENABLE); //Flash operate must enable HSI Clock
		FLASH_Unlock();     //Unlock Flash Register
		FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPRTERR);		//Clear All Fault Flag
		FLASHstatus = FLASH_ErasePage(FlashWadd);  //Erase one page
		if(FLASHstatus==FLASH_COMPLETE)   //Wait Erase is OK
			{
				while(i<2)
				{
					FLASH_ProgramWord(WrAdd,FlashBuffer1[i]);
					i++;
					WrAdd+=4;
				}
			}
		asm("CPSIE  I");		//Flash Done Enable All Interrupt	
		FLASH_Lock();				//Lock	
	}
}


/*函数名称:void Flash_Wr(void)                     
* 函数介绍:Flash Erase and Write
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:处于CAN通信时，主要用于写入产品默认参数(报警屏蔽位-CAN波特率-CAN通信周期-预留(0x00))
*/

void Flash_Wr2(void)
{
	byte i=0;
	word WrAdd = Config_Add;
	
	if(*(word *)Config_Add!=0xaaaa5555)  //写入初始数据
	{
		asm("CPSID I");    //Flash Operate Should Disable All Interrupt	
		FLASH_Status FLASHstatus = FLASH_COMPLETE;
		RCC_HSICmd(ENABLE); //Flash operate must enable HSI Clock
		FLASH_Unlock();     //Unlock Flash Register
		FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPRTERR);		//Clear All Fault Flag
		FLASHstatus = FLASH_ErasePage(Config_Add);  //Erase one page
		if(FLASHstatus==FLASH_COMPLETE)   //Wait Erase is OK
			{
				while(i<2)
				{
					FLASH_ProgramWord(WrAdd,FlashBuffer2[i]);
					i++;
					WrAdd+=4;
				}
			}
		asm("CPSIE  I");		//Flash Done Enable All Interrupt	
		FLASH_Lock();				//Lock	
	}
}


/*函数名称:void updata_CANID(void)                     
* 函数介绍:receive new CAN ID and update 
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
*/

void update_CANID(void)
{
	byte i=0;
	word Wradd1 = FlashWadd;
  word Wardd2 = Config_Add;
	static word new_canid;
	static byte Can_Buffer[8]={0};
  static word ConfigBytes;
	
	if(CAN_ReOk > 0)   //接收到CAN ID
	{
		CAN_ReOk--;
		memcpy(&Can_Buffer,&RxMessage.Data,sizeof(Can_Buffer));    
		if((0xa3==Can_Buffer[0])&&(0xf1==Can_Buffer[1])&&(0xaa==Can_Buffer[2])&&(0x55==Can_Buffer[3]))  //校验OK
			{
				new_canid = Can_Buffer[4]<<24|Can_Buffer[5]<<16|Can_Buffer[6]<<8|Can_Buffer[7];
				if(new_canid != *(word *)CANID_Add)    //如果新ID和本地ID号不一样，则记录此ID号
					{
						FlashBuffer1[1] = new_canid;
						asm("CPSID I");    //Flash Operate Should Disable All Interrupt	
						FLASH_Status FLASHstatus = FLASH_COMPLETE;
						RCC_HSICmd(ENABLE); //Flash operate must enable HSI Clock
						FLASH_Unlock();     //Unlock Flash Register
						FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPRTERR);		//Clear All Fault Flag
						FLASHstatus = FLASH_ErasePage(FlashWadd);  //Erase one page
						if(FLASHstatus==FLASH_COMPLETE)   //Wait Erase is OK
							{
								while(i<2)
								{
									FLASH_ProgramWord(Wradd1,FlashBuffer1[i]);
									i++;
									Wradd1+=4;
								}
							}
						asm("CPSIE  I");		//Flash Done Enable All Interrupt	
						FLASH_Lock();				//Lock
            
            CAN_ExtID      = *(word *)CANID_Add; 
					}

			}
    else if((0xa3==Can_Buffer[0])&&(0xf1==Can_Buffer[1])&&(0xaa==Can_Buffer[2])&&(0xaa==Can_Buffer[3]))
      {
        FlashBuffer2[0]=0xaaaa5555;
        ConfigBytes = Can_Buffer[4]<<24|Can_Buffer[5]<<16|Can_Buffer[6]<<8|Can_Buffer[7];
				if(ConfigBytes != *(word *)Reserved_Add)     //配置字节不一样
					{
						FlashBuffer2[1] = ConfigBytes;
						asm("CPSID I");    	
						FLASH_Status FLASHstatus = FLASH_COMPLETE;
						RCC_HSICmd(ENABLE); 
						FLASH_Unlock();     
						FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPRTERR);		//Clear All Fault Flag
						FLASHstatus = FLASH_ErasePage(Config_Add);  
						if(FLASHstatus==FLASH_COMPLETE)   
							{
								while(i<2)
								{
									FLASH_ProgramWord(Wardd2,FlashBuffer2[i]);
									i++;
									Wardd2+=4;
								}
							}
						asm("CPSIE  I");			
						FLASH_Lock();	
            FedBackFlag = 0x55;       //参数修改成功后返回一帧数据至CAN总线
				}
      }
    else if((0xa3==Can_Buffer[0])&&(0xf1==Can_Buffer[1])&&(0xaa==Can_Buffer[2])&&(0x5a==Can_Buffer[3]))  //需要返回信息
     {
        switch(CAN_ExtID)
          {
            case 0x18ff5a51:
              if(0x01==(Can_Buffer[4]&0x01))
                 FedBackFlag = 0x55; 
              break;
            case 0x18ff5a52:
              if(0x02==(Can_Buffer[4]&0x02))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a53:
              if(0x04==(Can_Buffer[4]&0x04))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a54:
              if(0x08==(Can_Buffer[4]&0x08))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a55:
              if(0x10==(Can_Buffer[4]&0x10))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a56:
              if(0x20==(Can_Buffer[4]&0x20))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a57:
              if(0x40==(Can_Buffer[4]&0x40))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a58:
              if(0x80==(Can_Buffer[4]&0x80))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a59:
              if(0x01==(Can_Buffer[5]&0x01))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a5a:
              if(0x02==(Can_Buffer[5]&0x02))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a5b:
              if(0x04==(Can_Buffer[5]&0x04))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a5c:
              if(0x08==(Can_Buffer[5]&0x08))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a5d:
              if(0x10==(Can_Buffer[5]&0x10))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a5e:
              if(0x20==(Can_Buffer[5]&0x20))
                 FedBackFlag = 0x55;
              break;  
            case 0x18ff5a5f:
              if(0x40==(Can_Buffer[5]&0x40))
                 FedBackFlag = 0x55;
              break;
            case 0x18ff5a60:
              if(0x80==(Can_Buffer[5]&0x80))
                 FedBackFlag = 0x55;
              break; 
            case 0x18ff5a61:
              if(0x01==(Can_Buffer[6]&0x01))
                 FedBackFlag = 0x55;
              break; 
            case 0x18ff5a62:
              if(0x02==(Can_Buffer[6]&0x02))
                 FedBackFlag = 0x55;
              break; 
            case 0x18ff5a63:
              if(0x04==(Can_Buffer[6]&0x04))
                 FedBackFlag = 0x55;
              break;  
            case 0x18ff5a64:
              if(0x08==(Can_Buffer[6]&0x08))
                 FedBackFlag = 0x55;
              break;
          default:
            break;
          }   
     }else{;}
	}
}


/*函数名称:void Time_Fuc(void)                     
* 函数介绍:传感器系统时间
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
*/

void Time_Fuc(void)
{
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


/*函数名称:void WriteIonValue(void)                     
* 函数介绍:传感器第一次上电采集Ion初值并记录
* 输入参数:no
* 输出参数:no
* 返 回 值:no
* 说    明:no
*/

void WriteIonValue(void)
{
  byte i=0;
	word WrAdd = IonFlashFlag; 
  
  if(0x5a5a5a5a != *(word *)IonFlashFlag)   
    {
      FlashBuffer3[1] = (ADCConvertedValue[3]+ADCConvertedValue[7]+ADCConvertedValue[11]+ADCConvertedValue[15]+ADCConvertedValue[19])/5;
        
			asm("CPSID I");    	
      FLASH_Status FLASHstatus = FLASH_COMPLETE;
      RCC_HSICmd(ENABLE); 
      FLASH_Unlock();     
      FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPRTERR);		//Clear All Fault Flag
      FLASHstatus = FLASH_ErasePage(IonFlashFlag);  
      if(FLASHstatus==FLASH_COMPLETE)   
        {
          while(i<2)
          {
            FLASH_ProgramWord(WrAdd,FlashBuffer3[i]);
            i++;
            WrAdd+=4;
          }
        }
      asm("CPSIE  I");			
      FLASH_Lock();	
    }else{;}
}




