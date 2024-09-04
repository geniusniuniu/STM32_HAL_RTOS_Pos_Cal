#include "mpu9250.h"
#include "myiic.h"
#include "delay.h"
#include <stdio.h>

float Pitch_9AX,Roll_9AX,Yaw_9AX;	//ŷ����
short Temp_9AX;                     //�¶�


//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}

//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	} 	
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    SoftSim_IIC_Start();
    SoftSim_IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(SoftSim_IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        SoftSim_IIC_Stop();
        return 1;
    }
    SoftSim_IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    SoftSim_IIC_Wait_Ack();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        SoftSim_IIC_Send_Byte(buf[i]);  //��������
        if(SoftSim_IIC_Wait_Ack())      //�ȴ�ACK
        {
            SoftSim_IIC_Stop();
            return 1;
        }
    }
    SoftSim_IIC_Stop();
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    SoftSim_IIC_Start();
    SoftSim_IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(SoftSim_IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        SoftSim_IIC_Stop();
        return 1;
    }
    SoftSim_IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    SoftSim_IIC_Wait_Ack();             //�ȴ�Ӧ��
	SoftSim_IIC_Start();        
    SoftSim_IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    SoftSim_IIC_Wait_Ack();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=SoftSim_IIC_Read_Byte(0);//������,����nACK 
		else *buf=SoftSim_IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++;  
    }
    SoftSim_IIC_Stop();                 //����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    SoftSim_IIC_Start();
    SoftSim_IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(SoftSim_IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        SoftSim_IIC_Stop();
        return 1;
    }
    SoftSim_IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    SoftSim_IIC_Wait_Ack();             //�ȴ�Ӧ��
    SoftSim_IIC_Send_Byte(data);        //��������
    if(SoftSim_IIC_Wait_Ack())          //�ȴ�ACK
    {
        SoftSim_IIC_Stop();
        return 1;
    }
    SoftSim_IIC_Stop();
    return 0;
}

