#include "mpu6050.h"
#include  "sys.h"
#include "delay.h"

#define  uchar unsigned char 

#define  uint  unsigned int 

#define SMPLRT_DIV  0x19 //陀螺仪采样率典型值0x07(125Hz) 
#define CONFIG   0x1A //低通滤波频率典型值0x06(5Hz) 
#define GYRO_CONFIG  0x1B //陀螺仪自检及测量范围典型值0x18(不自检2000deg/s) 
#define ACCEL_CONFIG 0x1C //加速计自检、测量范围及高通滤波频率典型值0x01(不自检2G5Hz) 
#define ACCEL_XOUT_H 0x3B 
#define ACCEL_XOUT_L 0x3C 
#define ACCEL_YOUT_H 0x3D 
#define ACCEL_YOUT_L 0x3E 
#define ACCEL_ZOUT_H 0x3F 
#define ACCEL_ZOUT_L 0x40 
#define TEMP_OUT_H  0x41 
#define TEMP_OUT_L  0x42 
#define GYRO_XOUT_H  0x43 
#define GYRO_XOUT_L 0x44  
#define GYRO_YOUT_H  0x45 
#define GYRO_YOUT_L  0x46 
#define GYRO_ZOUT_H  0x47 
#define GYRO_ZOUT_L  0x48 
#define PWR_MGMT_1  0x6B //电源管理典型值0x00(正常启用) 
#define WHO_AM_I   0x75 //IIC地址寄存器(默认数值0x68只读) 
#define SlaveAddress 0xD0 //IIC写入时的地址字节数据+1为读取 


//IO方向设置
//PB.7 -> SDA
#define MPU_SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;} 
#define MPU_SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}

//IO操作函数	 
#define MPU_IIC_SCL    PBout(6) 		//SCL
#define MPU_IIC_SDA    PBout(7) 		//SDA	 
#define MPU_READ_SDA   PBin(7) 		//输入SDA 

	
#define SCL PBout(6) 		//SCL
#define SDA PBout(7) 		//SDA


int Accel_x,Accel_y,Accel_z ;
float Gyro_y;

float x_zero=0.0,y_zero=0.0,z_zero=0.0;

int Angle_Speed;

short Left_DeadMotor = 36,Right_DeadMotor = 36;

uchar value;

int buchang=0,buchang_L = 0;

float Lx,Rx;

//******卡尔曼参数************
float Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.005;//注意：dt的取值为kalman滤波器采样时间;
float PP[2][2] = {{1.0,0.0},{0.0,1.0}};
float Pdot[4]={0,0,0,0};
const float C_0 = 1;
float Q_bias = 0, Angle_err = 0, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
char cy; 


//函数：
void delayM(unsigned int t) 
{       
   delay_us(2);
}          

//初始化模拟IIC端口设置
void MPU_IIC_Init(void)
{					     
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//先使能外设IO PORTB时钟 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;// 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//根据设定参数初始化GPIO 

	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);//PB6,PB7 输出高	
}


void sda_high()    //SDA高电平的产生 

{ 
    MPU_SDA_OUT();

	MPU_IIC_SDA=1; 

    _asm(nop);                     

    _asm(nop); 

    _asm(nop);                    

    _asm(nop); 

} 

//************************************** 

//SDA低电平的产生 

//************************************** 

void sda_low()   //SDA低电平的产生 

{ 

    MPU_SDA_OUT();

    MPU_IIC_SDA=0; 

    _asm(nop);                     

    _asm(nop); 

    _asm(nop);                    

    _asm(nop);   
                        
} 

//************************************** 

//SCL高电平产生 

//************************************** 

void scl_high()  //SCL高电平产生 

{ 
    MPU_SDA_OUT();

    MPU_IIC_SCL=1; 

    _asm(nop);                     

    _asm(nop); 

    _asm(nop);                    

    _asm(nop); 

} 

//************************************** 

//SCL低电平的产生 

//************************************** 

void scl_low() //SCL低电平的产生 

{ 
    MPU_SDA_OUT();

    MPU_IIC_SCL=0; 

    _asm(nop);                     

    _asm(nop); 

    _asm(nop);                    

    _asm(nop); 
} 

//************************************** 

//I2C起始信号 

//************************************** 

void I2C_Start() 

{ 

    sda_high();                    //拉高数据线 

    scl_high();                    //拉高时钟线 

    delayM(1);                 //延时 

    sda_low();                    //产生下降沿 

    delayM(1);                 //延时 

    scl_low();                    //拉低时钟线 

} 

//************************************** 

//I2C停止信号 

//************************************** 

void I2C_Stop() 
{ 

    sda_low();                    //拉低数据线 

    scl_high();                    //拉高时钟线 

    delayM(1);                 //延时 

    sda_high();                    //产生上升沿 

    delayM(1);                //延时 

} 

//************************************** 

//I2C发送应答信号 

//入口参数:ack (0:ACK 1:NAK) 

//************************************** 

void I2C_SendACK(char ack) 

{    

    if(ack)   sda_high(); 

    else    

     sda_low(); //写应答信号 

     scl_high();                    //拉高时钟线 

     delayM(1);                   //延时 

     scl_low();                    //拉低时钟线 

     delayM(1);                   //延时 

} 

//************************************** 

//I2C接收应答信号 

//************************************** 

void I2C_RecvACK() 

{    

	MPU_SDA_IN();
    scl_high();                    //拉高时钟线 

    MPU_SDA_IN();

    delayM(1);                //延时 

    if(MPU_IIC_SDA==1) 

    { 
        cy=1; 
    } 

    else  

    { 
        cy=0; 
    } 

//    cy = SDA;                   //读应答信号 

    scl_low();                    //拉低时钟线 

    delayM(1);                //延时 

    MPU_SDA_OUT();  
    
 //   return cy; 

} 

//************************************** 

//向I2C总线发送一个字节数据 

//************************************** 

void I2C_SendByte(uchar dat) 

{ 

    uchar i; 

    for (i=0; i<8; i++)         //8位计数器 

    { 

        if((dat<<i)&0x80) 

        { 

            sda_high(); 

        } 

        else  

        { 

            sda_low(); 

        } 

       // SDA = cy;               //送数据口 

        scl_high();                //拉高时钟线 

        delayM(1);              //延时 

        scl_low();                //拉低时钟线 

        delayM(1);              //延时 

    } 

    I2C_RecvACK(); 

} 

//************************************** 

//从I2C总线接收一个字节数据 

//************************************** 

uchar I2C_RecvByte() 

{ 

    uchar i; 

    uchar dat = 0,cy; 

    sda_high();                    //使能内部上拉,准备读取数据, 

	MPU_SDA_IN();
	
    for (i=0; i<8; i++)         //8位计数器 

    { 

        dat <<= 1; 

        scl_high();                //拉高时钟线 

        delayM(1);             //延时 

        if(MPU_READ_SDA==1) 

        { 

            cy=1;         
        } 

        else  

        { 

            cy=0; 

        } 

        dat |= cy;             //读数据              

        scl_low();                //拉低时钟线 
                       
        delayM(1);             //延时 

    } 

	MPU_SDA_OUT();
    return dat; 

} 

//************************************** 

//向I2C设备写入一个字节数据 

//************************************** 

void Single_WriteI2C(uchar REG_Address,uchar REG_data) 

{ 

    I2C_Start();                  //起始信号 

    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号 

    I2C_SendByte(REG_Address);    //内部寄存器地址 

    I2C_SendByte(REG_data);       //内部寄存器数据 

    I2C_Stop();                   //发送停止信号 

} 

//************************************** 

//从I2C设备读取一个字节数据 

//************************************** 

uchar Single_ReadI2C(uchar REG_Address) 

{ 

   uchar REG_data; 

   I2C_Start();                   //起始信号 

   I2C_SendByte(SlaveAddress);    //发送设备地址+写信号 

   I2C_SendByte(REG_Address);     //发送存储单元地址从0开始  

   I2C_Start();                   //起始信号 

   I2C_SendByte(SlaveAddress+1);  //发送设备地址+读信号 

   REG_data=I2C_RecvByte();       //读出寄存器数据 

   I2C_SendACK(1);                //接收应答信号 

   I2C_Stop();                    //停止信号 

   return REG_data; 

} 

//************************************** 

//初始化MPU6050 

//************************************** 

void InitMPU6050() 

{ 
   Single_WriteI2C(PWR_MGMT_1, 0x00); //解除休眠状态 

   Single_WriteI2C(SMPLRT_DIV, 0x07); 

   Single_WriteI2C(CONFIG, 0x06); 

   Single_WriteI2C(GYRO_CONFIG, 0x18); 

   Single_WriteI2C(ACCEL_CONFIG, 0x01); 

} 

//************************************** 

//合成数据 

//************************************** 

short GetData(uchar REG_Address) 

{ 

   uchar H,L; 

   H=Single_ReadI2C(REG_Address); 

   L=Single_ReadI2C(REG_Address+1); 

   return (H<<8)+L;   //合成数据 

} 


float MPU6050_Get_Angle(float x,float z) //,float y,
{ 

    float temp , res; 
 
    temp=x/sqrt(/*y*y+*/z*z); 

    res=atan(temp); 

    return res*180.0/3.1415926;//把弧度转换成角度 
/**************
    与自然Z轴的角度
        temp=sqrt((x*x+y*y))/z; 

        res=atan(temp); 
        
    与自然Y轴的角度 
        temp=y/sqrt((x*x+z*z)); 

        res=atan(temp); 
***************/
} 


                                  
void Get_Gyro_Zero(void) 
{
    int i;
    gyro_zero=0;
    for(i=0;i<1000;i++) 
    {
        gyro_zero += (unsigned int)GetData(GYRO_YOUT_H); //Y轴角速度
    }  
    gyro_zero = (gyro_zero/1000); 
}

void Get_Gravity_Zero(void) 
{
    int i;
    for(i=0;i<1000;i++) 
    {
        x_zero +=GetData(ACCEL_XOUT_H)*0.001; //X轴输出值待转化为角度 

        y_zero +=GetData(ACCEL_YOUT_H)*0.001; 

        z_zero +=GetData(ACCEL_ZOUT_H)*0.001; 
    }
}

// 卡尔曼滤波
//*********************************************************
//Kalman滤波，20MHz的处理时间约0.77ms；
void Kalman_Filter(float Accel,float Gyro)		
{
	Angle+=(Gyro - Q_bias) * dt; //先验估计
  Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - Angle;	//zk-先验估计
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;		
	Angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
//Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
//Angle_gy = Angle_gy + Gyro_y*0.005 ;  //角速度值积分计算角度
}

// 倾角计算（卡尔曼融合）
void Angle_Calcu(void)	 
{
	//------加速度--------------------------
	//范围为2g时，换算关系：16384 LSB/g
	//角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
	//因为x>=sinx,故乘以1.3适当放大
	
	  Accel_x = (GetData(ACCEL_XOUT_H)-Gravity_X_Zero); //X轴加速度待转化为角度 

    Accel_y = (GetData(ACCEL_YOUT_H)-Gravity_Y_Zero); //减去零点 

    Accel_z = (GetData(ACCEL_ZOUT_H)-Gravity_Z_Zero);  
    
    Gyro_y=-((GetData(0x45)-gyro_zero))/16.4; //陀螺仪Y轴角速度 ,除去零点漂移 

    Angle_ax=MPU6050_Get_Angle( Accel_x,Accel_z );//Accel_y,//由加速度计推算角度
    
  //Get_MMA7361();
	
//	Accel_x  = GetData(ACCEL_XOUT_H);	  //读取X轴加速度
//	Angle_ax = (Accel_x - 1100) /16384;   //去除零点偏移,计算得到角度（弧度）
//	Angle_ax = Angle_ax*1.2*180/3.14;     //弧度转换为度,
    //-------角速度-------------------------
	//范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
 
//	 Angle_gy = Angle_gy + Gyro_y*0.01;  //角速度积分得到倾斜角度.		
	//-------卡尔曼滤波融合-----------------------
	
//	Kalman_Filter(Angle_ax,Gyro_y);       //卡尔曼滤波计算倾角
	
	/*//-------互补滤波-----------------------
	//补偿原理是取当前倾角和加速度获得倾角差值进行放大，然后与
    //陀螺仪角速度叠加后再积分，从而使倾角最跟踪为加速度获得的角度
	//0.5为放大倍数，可调节补偿度；0.01为系统周期10ms			
	Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/															  
} 
void Show_Angle(void) 
{
   
    /*
         OutData[0] =  (short)Angle_ax*100;  //加速度计推算所得角度
         OutData[1] =  (short)Gyro_y*100;    //角速度数值
         OutData[2] =  (short)Angle*100;     //卡尔曼滤波结果角度
    */
     /*
         角速度积分所得角度，仅用于验证陀螺仪量程     
         OutData[3] = (short) Angle_gy*100; 
         trans((int)Angle_gy,a) ;
         LCD_P8x16Str(5,6,a);
     */
         trans((int)Angle_ax,x);
         trans((int)Angle,y);
         trans((int)Gyro_y,z) ;
                           
         LCD_P8x16Str(5,0,x); //LED显示
         LCD_P8x16Str(45,0,y);
       //  LCD_P8x16Str(5,2,z);

   //      OutPut_Data(); //串口输出数据
}



/*
函数废弃区

void Get_Gravity(void)
{
    
   
   
    x1=GetData(ACCEL_XOUT_H); X轴输出值待转化为角度 

    y1=GetData(ACCEL_YOUT_H);  

    z1=GetData(ACCEL_ZOUT_H);  

    Accel_x=MPU6050_Get_Angle( x1, y1, z1,0);

    Accel_y=MPU6050_Get_Angle( x1, y1, z1,1);

    Accel_z=MPU6050_Get_Angle( x1, y1, z1,2);
    
    trans(Accel_x,x) ;
    trans(Accel_y,y) ;
    trans(Accel_z,z) ;
    
     LCD_P8x16Str(5,0,x);
     LCD_P8x16Str(5,2,y);
     LCD_P8x16Str(5,4,z);
}



void Get_Gyro(void) 
{
   
    
    Gyro_x=GetData(GYRO_XOUT_H); 显示X轴角速度 
       
    Gyro_y=GetData(GYRO_YOUT_H); 显示Y轴角速度 

    Gyro_z=GetData(GYRO_ZOUT_H); 显示Z轴角速度 
}

*/

