#include "led.h"
#include "delay.h"
#include "sys.h"
#include "key.h"
#include "usart.h"
#include "MPU6050.h"
#include "HMC5883L.h"
//ALIENTEK Mini STM32�����巶������3
//����ʵ��
//������̳:www.openedv.com

int16_t ax, ay, az;	
int16_t gx, gy, gz;
int16_t hx, hy, hz;
int16_t hmcvalue[3],XA;

u8	hbuffer[6];
u8 	abuffer[6];
float sum=0,sum1=0,sum2=0;	

 int main(void)
 {
	u8 t;
	u8 len;	
	u16 times=0;  
	u8 X[6],A[10];
	u8 AX[6],AY[6],AZ[6],HX[6],HY[6],HZ[6];  //������
	float angle,direction;	
 	SystemInit();//ϵͳʱ�ӵȳ�ʼ��
	delay_init(72);	     //��ʱ��ʼ��
	NVIC_Configuration();//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	IIC_Init();	 //��ʼ��I2C�ӿ�
	uart_init(115200);//���ڳ�ʼ��Ϊ9600
 	LED_Init();	 //LED�˿ڳ�ʼ��115200
	MPU6050_initialize();
	HMC5883L_SetUp();
	delay_ms(50);
	HMC5883L_SetUp();
	IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 2, abuffer);
	XA=(((int16_t)abuffer[0]) << 8) | abuffer[1];
	while(1)
	{
		IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, abuffer);
		ax=(((int16_t)abuffer[0]) << 8) | abuffer[1];
		ay=(((int16_t)abuffer[2]) << 8) | abuffer[3];
		az=(((int16_t)abuffer[4]) << 8) | abuffer[5];
		sum=(39.2/65536.0)*(ax-XA);
		sum1=ax*0.0025;
		sum2+=sum1;
		IICreadBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,hbuffer);
		hx=(((int16_t)hbuffer[0]) << 8) | hbuffer[1];
		hz=(((int16_t)hbuffer[2]) << 8) | hbuffer[3];
		hy=(((int16_t)hbuffer[4]) << 8) | hbuffer[5];
		
//		acc_x= GetData(ACCEL_XOUT_H);
//		acc_y= GetData(ACCEL_YOUT_H);
//		acc_z= GetData(ACCEL_ZOUT_H);
//		gy_x= GetData(GYRO_XOUT_H);
//		gy_y= GetData(GYRO_YOUT_H);
//		gy_z= GetData(GYRO_ZOUT_H);

		printf("ACC:  X=%d   Y=%d   Z=%d  \n",ax,ay,az);
		printf("GYRO:  X=%d   Y=%d   Z=%d  \n",hx,hy,hz);
		delay_ms(500);   
	}	 

 }

