#include "main.h"
#include "stm32f4xx_hal.h"
//SDO(MISO) PA6
//SDI(MOSI) PA7
//CLK(SCLK) PA5
//SCSN(CS) PA4

//CLK or SCK
//MOSI/SDI/RX
//MISO/SDO/TX

#define CLK_H     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)
#define CLK_L     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)
#define CS_H 			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define CS_L 			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define MOSI_H 		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET)
#define MOSI_L 		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET)
#define ReadMISO  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)

extern unsigned char v7761Data[4];		//获取CSE7761数据
extern unsigned char v7761Data_Repeat[4];//再次获取CSE7761数据
extern unsigned char v7761C_Data[20]; //cse7761系数
extern unsigned int  v7761_Result;  //函数返回结果
extern unsigned int  v7761_EA_CFSUM; //能量ad值总和


void cse7761_SPI_Write(void);
void cse7761_SPI_Read(unsigned char vaddr,unsigned char *vpp,unsigned char vsize);
void cse7761_SPI_WriteRegister(void);

void delay1uS(void)
{
	unsigned char i;
	for(i=0;i<100;i++) ;
}

//void delay1uS(void)
//{
//	unsigned char i;
//	for(i=0;i<20;i++) ;
//}

void delay2uS(void)
{
	uint16_t i;
	for(i=0;i<300;i++) ;
}

/**********************************
1、通讯引脚初始化
2、上电读取CSE7761系数，系数存在于数组extern unsigned char v7761C_Data[20]中。
v7761C_Data[0]		CSE7761 70H高byte  A电流系数
v7761C_Data[1]		CSE7761 70H低byte	 A电流系数
v7761C_Data[2]		CSE7761 71H高byte  B电流系数	
v7761C_Data[3]		CSE7761 71H低byte  B电流系数
v7761C_Data[4]		CSE7761 72H高byte  电压系数
v7761C_Data[5]		CSE7761 72H低byte  电压系数
v7761C_Data[6]		CSE7761 73H高byte  A有功功率系数
v7761C_Data[7]		CSE7761 73H低byte  A有功功率系数
v7761C_Data[8]		CSE7761 74H高byte  B有功功率系数
v7761C_Data[9]		CSE7761 74H低byte  B有功功率系数
v7761C_Data[10]		CSE7761 75H高byte  视在功率系数
v7761C_Data[11]		CSE7761 75H低byte  视在功率系数
v7761C_Data[12]		CSE7761 76H高byte  A能量转换系数
v7761C_Data[13]		CSE7761 76H低byte  A能量转换系数
v7761C_Data[14]		CSE7761 77H高byte  B能量转换系数
v7761C_Data[15]		CSE7761 77H低byte  B能量转换系数
v7761C_Data[16]		CSE7761 6EH高byte  预留
v7761C_Data[17]		CSE7761 6EH低byte  预留
v7761C_Data[18]		CSE7761 6FH高byte  校验和取反
v7761C_Data[19]		CSE7761 6FH低byte  校验和取反
***********************************/
void cse7761_SPI_init(void)
{
//	GPIO_InitTypeDef GPIO_InitStr;
//	EXTI_InitTypeDef EXTI_InitStr;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOH, ENABLE);
//	vHaveData=0;
//	//MISO
//	GPIO_InitStr.GPIO_Pin=GPIO_Pin_6;
//	GPIO_InitStr.GPIO_Mode=GPIO_Mode_IN;
//	GPIO_InitStr.GPIO_Speed=GPIO_High_Speed;
//	GPIO_InitStr.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStr.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_Init(GPIOF,&GPIO_InitStr);
//	
//	//MOSI
//	GPIO_InitStr.GPIO_Pin=GPIO_Pin_9;
//	GPIO_InitStr.GPIO_Mode=GPIO_Mode_OUT;
//	GPIO_InitStr.GPIO_Speed=GPIO_High_Speed;
//	GPIO_InitStr.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStr.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_Init(GPIOF,&GPIO_InitStr);
//	
//	//CLK
//	GPIO_InitStr.GPIO_Pin=GPIO_Pin_6;
//	GPIO_InitStr.GPIO_Mode=GPIO_Mode_OUT;
//	GPIO_InitStr.GPIO_Speed=GPIO_High_Speed;
//	GPIO_InitStr.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStr.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_Init(GPIOD,&GPIO_InitStr);
//	
//	//CS
//	GPIO_InitStr.GPIO_Pin=GPIO_Pin_4;
//	GPIO_InitStr.GPIO_Mode=GPIO_Mode_OUT;
//	GPIO_InitStr.GPIO_Speed=GPIO_High_Speed;
//	GPIO_InitStr.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStr.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_Init(GPIOH,&GPIO_InitStr);
	
//	//--------------------------------------------------------------------
//	//信号中断设置PH5
//	GPIO_InitStr.GPIO_Pin=GPIO_Pin_5;
//	GPIO_InitStr.GPIO_Mode=GPIO_Mode_IN;
//	GPIO_InitStr.GPIO_Speed=GPIO_High_Speed;
//	GPIO_InitStr.GPIO_OType=GPIO_OType_OD;
//	GPIO_InitStr.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOH,&GPIO_InitStr);
//	
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOH, EXTI_PinSource5);
//	
//	EXTI_InitStr.EXTI_Line=EXTI_Line5;
//	EXTI_InitStr.EXTI_Mode=EXTI_Mode_Interrupt;
//	EXTI_InitStr.EXTI_Trigger=EXTI_Trigger_Rising;
//	EXTI_InitStr.EXTI_LineCmd=ENABLE;
//	EXTI_Init(&EXTI_InitStr);
//	
//	/* Enable and set Button EXTI Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure); 	
	
	//SPI Bus Initialize
	CS_H;
	delay1uS();
	CLK_L;
	delay1uS();
	MOSI_L;
	cse7761_SPI_WriteRegister();   //初始化cse7761
	
	
	
	//读取css7761系数 70~77H  6EH、6FH
	unsigned char i;
	unsigned char *P;
	for(i=0;i<8;i++)    //读取css7761系数 70~77H  
	{
		P=&v7761C_Data[i*2];
		cse7761_SPI_Read(0x70+i,P,2);
	}
	cse7761_SPI_Read(0x6E,&v7761C_Data[16],2);
	cse7761_SPI_Read(0x6F,&v7761C_Data[18],2);   //读取css7761系数6EH、6FH
		
	unsigned int  sum0,sum1;
	i=0;
	while(i<18)
	{
		sum0=sum0+v7761C_Data[i]*256+v7761C_Data[i+1];
		i=i+2;		
	}
	sum1=v7761C_Data[i]*256+v7761C_Data[i+1];             //此处待处理，系数读不对怎么办
																												//此处待处理，系数读不对怎么办 sum0(低16bit)=sum1（低16bit）系数才有效，注意
 

}





void cse7761_SPI_WriteByte(unsigned char vdata)
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		if(vdata&(0x80>>i))
			MOSI_H;
		else
			MOSI_L;		
		CLK_H;
		delay1uS();
		CLK_L;
		delay1uS();
	}
}

void cse7761_SPI_WriteCmd(unsigned char vdata)
{
	cse7761_SPI_WriteByte(vdata|0x80);
}

void cse7761_SPI_WriteData(unsigned char vdata)
{
	cse7761_SPI_WriteByte(vdata);
}

//vpdata : 数组指针
//vpdata : 多少个字节
void cse7761_SPI_ReadByte(unsigned char *vpdata,unsigned char vBytelong)
{
	unsigned char i,j;
	for(i=0;i<vBytelong;i++)
	{
		*vpdata=0;
		for(j=0;j<8;j++)
		{
			CLK_H;
			delay1uS();
			CLK_L;
			delay1uS();
			(*vpdata)<<=1;
			if(ReadMISO==GPIO_PIN_SET)
				(*vpdata)|=1;
		}
		(*vpdata)=~(*vpdata);
		vpdata++;
	}
}
//读取CSE7761函数，vaddr读取的地址，*vpp数据存储首地址，vsize读取的地址对应的数据量（byte数）
void cse7761_SPI_Read(unsigned char vaddr,unsigned char *vpp,unsigned char vsize)
{
	CS_L;
	delay1uS();
	cse7761_SPI_WriteByte(vaddr);
	cse7761_SPI_ReadByte(vpp,vsize);
	CS_H;
	delay1uS();
}

void cse7761_SPI_WriteRegister(void)
{
	//unsigned char vtestdata[5];
	CS_L;
	//写使能命令,使能写操作
	cse7761_SPI_WriteByte(0xea);
	cse7761_SPI_WriteByte(0xe5);
	CS_H;
	delay2uS();
//	css7761_SPI_Read(0xea,vtestdata,1);
	//设置为数据采集瞬时值
	CS_L;
	cse7761_SPI_WriteCmd(0x01);
	cse7761_SPI_WriteData(0x00);
	cse7761_SPI_WriteData(0x01);
	CS_H;
	delay2uS();
	
	
//	//设置为数据采集瞬时值PULSE1引脚输出PIN15
//	CS_L;
//	css7761_SPI_WriteCmd(0x1d);
//	css7761_SPI_WriteData(0x00);
//	css7761_SPI_WriteData(0x77);
//	CS_H;
//	delay2uS();
//	//设置瞬时值中断使能
//	CS_L;
//	css7761_SPI_WriteCmd(0x40);
//	css7761_SPI_WriteData(0x00);
//	css7761_SPI_WriteData(0x41);
//	CS_H;
//	delay2uS();


	CS_L;
	//写使能关闭命令,关闭写使能
	cse7761_SPI_WriteByte(0xea);
	cse7761_SPI_WriteByte(0xdc);
	CS_H;
	delay2uS();
}

/*****************************************
函数实现读取CSE7761 A通道电流有效值，结果放大1000倍，如5001表示5.001A
返回结果：unsigned int  v7761_Result
******************************************/
int CSE7761_Read_IA(void)
{
	cse7761_SPI_Read(0x24,v7761Data,3);   //高地址在前v7761Data[0]高byte,v7761Data[1]低byte
	cse7761_SPI_Read(0x44,v7761Data_Repeat,4); //v7761Data与v7761Data_Repeat值必须相等数据才有效，此处需自行做策略。注意
	                                            //位数向v7761Data对齐，如v7761Data  3byte（0~2）则v7761Data_Repeat也是3byte （0~2）
	if((v7761Data[0]&0x80)!=0)
	{
		v7761Data[0]=0;
		v7761Data[1]=0;
		v7761Data[2]=0;
	}
	//A电流计算公式=RmsIA*RmsIAC/2^23=RmsIA*RmsIAC/800000H ，结果放大1000倍 如5000=5A
	v7761_Result=((uint64_t)(((v7761Data[0]<<16)+(v7761Data[1]<<8)+v7761Data[2])) *((v7761C_Data[0]<<8)+v7761C_Data[1]))>>23;
	return v7761_Result;
	
}



/*****************************************
函数实现读取CSE7761 电压有效值，结果放大1000倍，如200123表示200.123V
返回结果：unsigned int  v7761_Result
******************************************/
int CSE7761_Read_U(void)
{
	cse7761_SPI_Read(0x26,v7761Data,3);   //高地址在前v7761Data[0]高byte,v7761Data[1]低byte
	cse7761_SPI_Read(0x44,v7761Data_Repeat,4);//v7761Data与v7761Data_Repeat值必须相等数据才有效，此处需自行做策略。注意
	if((v7761Data[0]&0x80)!=0)
	{
		v7761Data[0]=0;
		v7761Data[1]=0;
		v7761Data[2]=0;
	}
	//电压计算公式=RmsU*RmsUC*10/2^22=RmsU*RmsUC*10/400000H
	v7761_Result=(((uint64_t)((v7761Data[0]<<16)+(v7761Data[1]<<8)+v7761Data[2]))*((v7761C_Data[4]<<8)+v7761C_Data[5])*10)>>22;
	return v7761_Result;
}



/*****************************************
函数实现读取CSE7761 A通道有功功率，结果放大1000倍，如1000123表示1000.123W
返回结果：unsigned int  v7761_Result
******************************************/
int CSE7761_Read_PA(void)
{
	cse7761_SPI_Read(0x2c,v7761Data,4);   //高地址在前v7761Data[0]高byte,v7761Data[1]低byte
	cse7761_SPI_Read(0x44,v7761Data_Repeat,4);	//v7761Data与v7761Data_Repeat值必须相等数据才有效，此处需自行做策略。注意
	if((v7761Data[0]&0x80)!=0)
	{
			
		v7761_Result=((~((uint64_t)(v7761Data[0]<<24)+(v7761Data[1]<<16)+(v7761Data[2]<<8)+v7761Data[3])+1)*((v7761C_Data[6]<<8)+v7761C_Data[7])*1000)>>31;
		
	}
	else
	//A有功功率计算公式=PowerPA* PowerPAC*1000/2^31=PowerPA* PowerPAC*3E8H/80000000H
	{
		v7761_Result=(((uint64_t)(v7761Data[0]<<24)+(v7761Data[1]<<16)+(v7761Data[2]<<8)+v7761Data[3])*((v7761C_Data[6]<<8)+v7761C_Data[7])*1000)>>31;
	}
		return v7761_Result;
}



/*****************************************
函数实现读取CSE7761 A通道能量，结果放大1000倍，如1000123表示1000.123kW.h
返回结果：unsigned int  v7761_Result
******************************************/
int CSE7761_Read_EA(void)
{
	cse7761_SPI_Read(0x28,v7761Data,3);   //高地址在前v7761Data[0]高byte,v7761Data[1]低byte
	cse7761_SPI_Read(0x44,v7761Data_Repeat,4);//v7761Data与v7761Data_Repeat值必须相等数据才有效，此处需自行做策略。注意
	
	v7761_EA_CFSUM=v7761_EA_CFSUM+(v7761Data[0]<<16)+(v7761Data[1]<<8)+v7761Data[2];
	//能量计算公式=系数*脉冲个数*常数*1000/2^29/2^12=系数*脉冲个数*1000/2^29;    常数=7761的HFConst   02H地址   默认值1000H
	v7761_Result=(((uint64_t)v7761_EA_CFSUM)*((v7761C_Data[12]<<8)+v7761C_Data[13])*1000)>>29;
	return v7761_Result;
}

