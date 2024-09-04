#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"


// SCL  ------>  PB8   
// SDA  ------>  PB9
//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ

//IO����
#define IIC_SCL   PBout(8) //SCL
#define IIC_SDA   PBout(9) //SDA
#define READ_SDA  PBin(9)  //����SDA

//IIC���в�������
void SoftSim_IIC_Init(void);                //��ʼ��IIC��IO��				 
void SoftSim_IIC_Start(void);				//����IIC��ʼ�ź�
void SoftSim_IIC_Stop(void);	  			//����IICֹͣ�ź�
void SoftSim_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 SoftSim_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 SoftSim_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void SoftSim_IIC_Ack(void);					//IIC����ACK�ź�
void SoftSim_IIC_NAck(void);				//IIC������ACK�ź�

void SoftSim_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 SoftSim_IIC_Read_One_Byte(u8 daddr,u8 addr);	 


#endif

