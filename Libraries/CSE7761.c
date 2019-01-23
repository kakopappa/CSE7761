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

extern unsigned char v7761Data[4];		//��ȡCSE7761����
extern unsigned char v7761Data_Repeat[4];//�ٴλ�ȡCSE7761����
extern unsigned char v7761C_Data[20]; //cse7761ϵ��
extern unsigned int  v7761_Result;  //�������ؽ��
extern unsigned int  v7761_EA_CFSUM; //����adֵ�ܺ�


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
1��ͨѶ���ų�ʼ��
2���ϵ��ȡCSE7761ϵ����ϵ������������extern unsigned char v7761C_Data[20]�С�
v7761C_Data[0]		CSE7761 70H��byte  A����ϵ��
v7761C_Data[1]		CSE7761 70H��byte	 A����ϵ��
v7761C_Data[2]		CSE7761 71H��byte  B����ϵ��	
v7761C_Data[3]		CSE7761 71H��byte  B����ϵ��
v7761C_Data[4]		CSE7761 72H��byte  ��ѹϵ��
v7761C_Data[5]		CSE7761 72H��byte  ��ѹϵ��
v7761C_Data[6]		CSE7761 73H��byte  A�й�����ϵ��
v7761C_Data[7]		CSE7761 73H��byte  A�й�����ϵ��
v7761C_Data[8]		CSE7761 74H��byte  B�й�����ϵ��
v7761C_Data[9]		CSE7761 74H��byte  B�й�����ϵ��
v7761C_Data[10]		CSE7761 75H��byte  ���ڹ���ϵ��
v7761C_Data[11]		CSE7761 75H��byte  ���ڹ���ϵ��
v7761C_Data[12]		CSE7761 76H��byte  A����ת��ϵ��
v7761C_Data[13]		CSE7761 76H��byte  A����ת��ϵ��
v7761C_Data[14]		CSE7761 77H��byte  B����ת��ϵ��
v7761C_Data[15]		CSE7761 77H��byte  B����ת��ϵ��
v7761C_Data[16]		CSE7761 6EH��byte  Ԥ��
v7761C_Data[17]		CSE7761 6EH��byte  Ԥ��
v7761C_Data[18]		CSE7761 6FH��byte  У���ȡ��
v7761C_Data[19]		CSE7761 6FH��byte  У���ȡ��
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
//	//�ź��ж�����PH5
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
	cse7761_SPI_WriteRegister();   //��ʼ��cse7761
	
	
	
	//��ȡcss7761ϵ�� 70~77H  6EH��6FH
	unsigned char i;
	unsigned char *P;
	for(i=0;i<8;i++)    //��ȡcss7761ϵ�� 70~77H  
	{
		P=&v7761C_Data[i*2];
		cse7761_SPI_Read(0x70+i,P,2);
	}
	cse7761_SPI_Read(0x6E,&v7761C_Data[16],2);
	cse7761_SPI_Read(0x6F,&v7761C_Data[18],2);   //��ȡcss7761ϵ��6EH��6FH
		
	unsigned int  sum0,sum1;
	i=0;
	while(i<18)
	{
		sum0=sum0+v7761C_Data[i]*256+v7761C_Data[i+1];
		i=i+2;		
	}
	sum1=v7761C_Data[i]*256+v7761C_Data[i+1];             //�˴�������ϵ����������ô��
																												//�˴�������ϵ����������ô�� sum0(��16bit)=sum1����16bit��ϵ������Ч��ע��
 

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

//vpdata : ����ָ��
//vpdata : ���ٸ��ֽ�
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
//��ȡCSE7761������vaddr��ȡ�ĵ�ַ��*vpp���ݴ洢�׵�ַ��vsize��ȡ�ĵ�ַ��Ӧ����������byte����
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
	//дʹ������,ʹ��д����
	cse7761_SPI_WriteByte(0xea);
	cse7761_SPI_WriteByte(0xe5);
	CS_H;
	delay2uS();
//	css7761_SPI_Read(0xea,vtestdata,1);
	//����Ϊ���ݲɼ�˲ʱֵ
	CS_L;
	cse7761_SPI_WriteCmd(0x01);
	cse7761_SPI_WriteData(0x00);
	cse7761_SPI_WriteData(0x01);
	CS_H;
	delay2uS();
	
	
//	//����Ϊ���ݲɼ�˲ʱֵPULSE1�������PIN15
//	CS_L;
//	css7761_SPI_WriteCmd(0x1d);
//	css7761_SPI_WriteData(0x00);
//	css7761_SPI_WriteData(0x77);
//	CS_H;
//	delay2uS();
//	//����˲ʱֵ�ж�ʹ��
//	CS_L;
//	css7761_SPI_WriteCmd(0x40);
//	css7761_SPI_WriteData(0x00);
//	css7761_SPI_WriteData(0x41);
//	CS_H;
//	delay2uS();


	CS_L;
	//дʹ�ܹر�����,�ر�дʹ��
	cse7761_SPI_WriteByte(0xea);
	cse7761_SPI_WriteByte(0xdc);
	CS_H;
	delay2uS();
}

/*****************************************
����ʵ�ֶ�ȡCSE7761 Aͨ��������Чֵ������Ŵ�1000������5001��ʾ5.001A
���ؽ����unsigned int  v7761_Result
******************************************/
int CSE7761_Read_IA(void)
{
	cse7761_SPI_Read(0x24,v7761Data,3);   //�ߵ�ַ��ǰv7761Data[0]��byte,v7761Data[1]��byte
	cse7761_SPI_Read(0x44,v7761Data_Repeat,4); //v7761Data��v7761Data_Repeatֵ����������ݲ���Ч���˴������������ԡ�ע��
	                                            //λ����v7761Data���룬��v7761Data  3byte��0~2����v7761Data_RepeatҲ��3byte ��0~2��
	if((v7761Data[0]&0x80)!=0)
	{
		v7761Data[0]=0;
		v7761Data[1]=0;
		v7761Data[2]=0;
	}
	//A�������㹫ʽ=RmsIA*RmsIAC/2^23=RmsIA*RmsIAC/800000H ������Ŵ�1000�� ��5000=5A
	v7761_Result=((uint64_t)(((v7761Data[0]<<16)+(v7761Data[1]<<8)+v7761Data[2])) *((v7761C_Data[0]<<8)+v7761C_Data[1]))>>23;
	return v7761_Result;
	
}



/*****************************************
����ʵ�ֶ�ȡCSE7761 ��ѹ��Чֵ������Ŵ�1000������200123��ʾ200.123V
���ؽ����unsigned int  v7761_Result
******************************************/
int CSE7761_Read_U(void)
{
	cse7761_SPI_Read(0x26,v7761Data,3);   //�ߵ�ַ��ǰv7761Data[0]��byte,v7761Data[1]��byte
	cse7761_SPI_Read(0x44,v7761Data_Repeat,4);//v7761Data��v7761Data_Repeatֵ����������ݲ���Ч���˴������������ԡ�ע��
	if((v7761Data[0]&0x80)!=0)
	{
		v7761Data[0]=0;
		v7761Data[1]=0;
		v7761Data[2]=0;
	}
	//��ѹ���㹫ʽ=RmsU*RmsUC*10/2^22=RmsU*RmsUC*10/400000H
	v7761_Result=(((uint64_t)((v7761Data[0]<<16)+(v7761Data[1]<<8)+v7761Data[2]))*((v7761C_Data[4]<<8)+v7761C_Data[5])*10)>>22;
	return v7761_Result;
}



/*****************************************
����ʵ�ֶ�ȡCSE7761 Aͨ���й����ʣ�����Ŵ�1000������1000123��ʾ1000.123W
���ؽ����unsigned int  v7761_Result
******************************************/
int CSE7761_Read_PA(void)
{
	cse7761_SPI_Read(0x2c,v7761Data,4);   //�ߵ�ַ��ǰv7761Data[0]��byte,v7761Data[1]��byte
	cse7761_SPI_Read(0x44,v7761Data_Repeat,4);	//v7761Data��v7761Data_Repeatֵ����������ݲ���Ч���˴������������ԡ�ע��
	if((v7761Data[0]&0x80)!=0)
	{
			
		v7761_Result=((~((uint64_t)(v7761Data[0]<<24)+(v7761Data[1]<<16)+(v7761Data[2]<<8)+v7761Data[3])+1)*((v7761C_Data[6]<<8)+v7761C_Data[7])*1000)>>31;
		
	}
	else
	//A�й����ʼ��㹫ʽ=PowerPA* PowerPAC*1000/2^31=PowerPA* PowerPAC*3E8H/80000000H
	{
		v7761_Result=(((uint64_t)(v7761Data[0]<<24)+(v7761Data[1]<<16)+(v7761Data[2]<<8)+v7761Data[3])*((v7761C_Data[6]<<8)+v7761C_Data[7])*1000)>>31;
	}
		return v7761_Result;
}



/*****************************************
����ʵ�ֶ�ȡCSE7761 Aͨ������������Ŵ�1000������1000123��ʾ1000.123kW.h
���ؽ����unsigned int  v7761_Result
******************************************/
int CSE7761_Read_EA(void)
{
	cse7761_SPI_Read(0x28,v7761Data,3);   //�ߵ�ַ��ǰv7761Data[0]��byte,v7761Data[1]��byte
	cse7761_SPI_Read(0x44,v7761Data_Repeat,4);//v7761Data��v7761Data_Repeatֵ����������ݲ���Ч���˴������������ԡ�ע��
	
	v7761_EA_CFSUM=v7761_EA_CFSUM+(v7761Data[0]<<16)+(v7761Data[1]<<8)+v7761Data[2];
	//�������㹫ʽ=ϵ��*�������*����*1000/2^29/2^12=ϵ��*�������*1000/2^29;    ����=7761��HFConst   02H��ַ   Ĭ��ֵ1000H
	v7761_Result=(((uint64_t)v7761_EA_CFSUM)*((v7761C_Data[12]<<8)+v7761C_Data[13])*1000)>>29;
	return v7761_Result;
}

