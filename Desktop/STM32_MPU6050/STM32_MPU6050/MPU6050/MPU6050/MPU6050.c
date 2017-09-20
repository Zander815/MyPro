#include "stm32f10x.h"
#include "MPU6050.h"
#include "I2C.h"
#include "delay.h"

//**************************************
//��I2C�豸д��һ���ֽ�����
//**************************************
void Single_WriteI2C(u8 REG_Address,u8 REG_data)
{
    IIC_Start();                  //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress);   //�����豸��ַ+д�ź�
    IIC_Send_Byte(REG_Address);    //�ڲ��Ĵ�����ַ��
    IIC_Send_Byte(REG_data);       //�ڲ��Ĵ������ݣ�
    IIC_Stop();                   //����ֹͣ�ź�
}

//**************************************
//��I2C�豸��ȡһ���ֽ�����
//**************************************
u8 Single_ReadI2C(u8 REG_Address)
{
	u8 REG_data;
	IIC_Start();                   //��ʼ�ź�
	IIC_Send_Byte(SlaveAddress);    //�����豸��ַ+д�ź�
	IIC_Send_Byte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ	
	IIC_Start();                   //��ʼ�ź�
	IIC_Send_Byte(SlaveAddress+1);  //�����豸��ַ+���ź�
	REG_data=IIC_Read_Byte(0);       //�����Ĵ�������	0 	NACK   1  ACK
	IIC_Stop();                    //ֹͣ�ź�
	return REG_data;
}

//**************************************
//��ʼ��MPU6050
//**************************************
void InitMPU6050(void)
{
	//IIC(&dis[0],1,0x1c,GY_ADDR,WRITE);
/*	Single_WriteI2C(PWR_MGMT_1, 0x00);	//�������״̬
	Single_WriteI2C(SMPLRT_DIV, 0x07);
	Single_WriteI2C(CONFIG, 0x06);
	Single_WriteI2C(GYRO_CONFIG, 0x18);
	Single_WriteI2C(ACCEL_CONFIG, 0x01);   */

	IIC_Init();
	delay_ms(10);  
    Single_WriteI2C(MPU_60X0_PWR_MGMT_1_REG_ADDR, MPU_60X0_RESET_REG_VALU);   
    delay_ms(50); 
    Single_WriteI2C(MPU_60X0_PWR_MGMT_1_REG_ADDR, MPU_60X0_PWR_MGMT_1_REG_VALU);  
    Single_WriteI2C(MPU_60X0_USER_CTRL_REG_ADDR, MPU_60X0_USER_CTRL_REG_VALU);  
    Single_WriteI2C(MPU_60X0_SMPLRT_DIV_REG_ADDR, MPU_60X0_SMPLRT_DIV_REG_VALU);  
    Single_WriteI2C(MPU_60X0_CONFIG_REG_ADDR, MPU_60X0_CONFIG_REG_VALU);  
    Single_WriteI2C(MPU_60X0_GYRO_CONFIG_REG_ADDR, MPU_60X0_GYRO_CONFIG_REG_VALU);  
    Single_WriteI2C(MPU_60X0_ACCEL_CONFIG_REG_ADDR, MPU_60X0_ACCEL_CONFIG_REG_VALU);  
    Single_WriteI2C(MPU_60X0_FIFO_EN_REG_ADDR, MPU_60X0_FIFO_EN_REG_VALU);  
  
}


//**************************************  
//�ϳ�����  
//**************************************  
u16 GetData(u8 REG_Address)  
{  
    u8 H,L;  
    H=Single_ReadI2C(REG_Address);  
    L=Single_ReadI2C(REG_Address+1);  
    return (H<<8)+L;   //�ϳ�����  
}  






















