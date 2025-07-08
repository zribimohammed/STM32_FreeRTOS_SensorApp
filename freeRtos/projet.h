#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

QueueHandle_t ADC_QUEUE;
QueueHandle_t Buffer_QUEUE;
SemaphoreHandle_t  sem,semcount,semcount1;
static char buffer[100] = "";

void vTask0(void *p);
void vTask1(void *p);
void vTask2(void *p);
void vTask3(void *p);
void vTask4(void *p);
void vTask5(void *p);

void Config_EXTI0(void);
void Config_WD(void);
void config_ADC(void);
void config_gpio(void);
void config_usart2(void);
void send_string_USART2( char *pt);
void delay(int count);
void Config_I2C1(void);
void I2C_start(void);
void I2C_adress(uint8_t adress);
void I2C_write(uint8_t data);
uint8_t I2C1_Read(uint8_t ack);
void I2C_stop(void);
void DS1621_discount(void);
void start_conversion (void);
int Read_Temperature(void);
void ADC_IRQHandler(void);
void  EXTI0_IRQHandler(void);
