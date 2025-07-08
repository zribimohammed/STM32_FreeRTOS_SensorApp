
#include "projet.h"



int main(void)
{
  	 //Configurations
	Config_EXTI0();
	Config_WD();
	config_gpio();
	config_usart2();
	config_ADC();
	Config_I2C1();
    DS1621_discount();
    start_conversion();

	send_string_USART2("Projet RTOS\r\n");
		delay(1000000);
		GPIOD->ODR= 0x1<<12;

	//PREPARATION QUEUE
	ADC_QUEUE = xQueueCreate(3, sizeof(int));
	Buffer_QUEUE = xQueueCreate(1, sizeof(buffer));
	sem = xSemaphoreCreateBinary();
   // semcount= xSemaphoreCreateCounting(3,0);
    semcount1= xSemaphoreCreateCounting(3,0);
    TIM2->CR1|=0x1;//enable tim2
    //PREPARATION TASKS
     xTaskCreate( vTask0,"vTask_EN_WD", configMINIMAL_STACK_SIZE, NULL, 4,NULL);
     xTaskCreate( vTask1, "vTask_SEND_DATA", configMINIMAL_STACK_SIZE, NULL, 3,NULL);
	 xTaskCreate(vTask2, "vTask_PROCESS_DATA", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	 xTaskCreate(vTask3, "vTask_READ_CH0", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	 xTaskCreate(vTask4, "vTask_READ_CH1", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
     xTaskCreate(vTask5, "vTask_READ_CH2", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    //LANCEMENT ORDONNANCEUR
    vTaskStartScheduler();

    while(1);
}

