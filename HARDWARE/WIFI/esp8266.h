#ifndef __ESP8266_H
#define __ESP8266_H

#include "stdio.h"	
#include "sys.h" 
#include "usart.h"
#include "oled.h"	  
#include "delay.h"
#include "mpu6050-STM.h"

int Connect_master(void);
void Net_Init(void);
int Receive(void);
int Re(int seconds);

#endif
