#include "stm32f10x.h"
#include "usart.h"
#include "esp8266.h"
#include "time.h"
#include "delay.h"
#include "mpu6050-STM.h"
/************************************************

************************************************/
void Led_init(void);
int net_state=0;//标志是否连接到WiFi主机
short accel_x,accel_y,accel_z;//水平放置时，z轴会有1个g的加速度
short gyro_x,gyro_y,gyro_z; 
short ac_state,gr_state;
short az;
 int main(void)
{	
	u8 data;
	
	//LED指示灯初始化
	Led_init();
	//设置NVIC中断分组2 : 2位抢占优先级(中断嵌套次数)，2位响应优先级(中断响应先后顺序)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	//延时函数初始化	
	delay_init();
	//初始化 串口USART3  波特率921600 8位数据位 1位停止位 0位校验位
	UART_Init(921600);
	//MPU6050初始化
	MPU_Init(); 
		
	//连接主机--失败后每隔2秒重新尝试
	while( !(Connect_master()) )
	{
		delay_ms(1000);delay_ms(1000);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);//LED灯亮
		delay_ms(1000);delay_ms(1000);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);//LED灯灭
	}
	
	//LED灯亮--表示连接主机成功
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	while(1)
	{
		//MPU_Get_Accelerometer_Z(&accel_z);
		/*
		ac_state=MPU_Get_Accelerometer(&accel_x,&accel_y,&accel_z);	//得到加速度传感器数据
		gr_state=MPU_Get_Gyroscope(&gyro_x,&gyro_y,&gyro_z);	//得到陀螺仪数据
		delay_ms(200);
		*/
		
		/*
		//快闪标志进入工作状态
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);//LED灯亮
		delay_ms(500);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);//LED灯灭
		delay_ms(500);*/
		
		if( USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_13);//LED灯灭
			data = USART_ReceiveData(USART3);
			//R--收到主机的数据请求
			if( data == 'R')
			{
				//设置请求信息长度
				printf("AT+CIPSEND=6\r\n");
				if(Re(10)==1)
				{
					while(1)//等待允许发送的指令
					{
						if( USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
						{if( USART_ReceiveData(USART3) == '>'){break;}}
					}
					//向主机发送Z轴加速度数据
					if( MPU_Get_Accelerometer_Z(&az)==0 )
					{
						printf("OK");
						//加速度高8位
						USART3->DR = (u8)(az>>8);
						while((USART3->SR&0X40)==0);//循环发送,直到发送完毕
						//加速度低8位
						USART3->DR = (u8)(az&0xff);
						while((USART3->SR&0X40)==0);//循环发送,直到发送完毕
						printf("\r\n");
						if(Re(10)==1){;}//LCD_Print(0,6,"Send-Request--OK");}
						else{;}//LCD_Print(0,6,"ERROR");}
					}
					else{;}//printf("ERROR");}
				}
			}
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);//LED灯亮
		}
	}
 }
void Led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;		//PC.13
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//灯灭
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}
