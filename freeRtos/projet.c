#include "projet.h"

uint8_t temp2 = 0, t = 0;
uint8_t temp1 = 0;
char Temp[50] = "", data[100] = "", temp[50] = "";
uint32_t ADC_VALUE[3] = {0};
int j = 0, i = 0;
uint32_t CHx_VALUE, CHy_VALUE, CHz_VALUE;
char b1[10],b2[10],b3[10];

void Config_EXTI0()
{
 RCC->APB2ENR |=1<<14;//enable System clock configuration
 SYSCFG->EXTICR[0]|=0x0;//liaison du mux0 avec PA0
 EXTI->RTSR|=0x1;//realisation d'une interruption externe sur rising edge
 EXTI->IMR|=0x1;//Autorisation d'interruption de l'interruption externe 0
 NVIC_EnableIRQ(EXTI0_IRQn);//Enable interrupt request
  NVIC_SetPriority(EXTI0_IRQn,4);
}
void Config_WD()
{
RCC->APB1ENR|=0x1<<11;
WWDG->CR|=0x7F; // Charger la valeur maximale pour éviter une réinitialisation immédiate
WWDG->CFR|=0x7B;//Time (s)= (1/APB1 clock frequency)*4096*CR*CFR //T=4s
WWDG->SR = 0x00;
while (WWDG->SR&0x1);// Attendez que la configuration soit prise en compte
}

void config_ADC(void)
{
	//config of ADC1
	  RCC  ->APB2ENR |=1<<8;//ADC Clock Enable
	  ADC  ->CCR|=3<<16;// 8 dans prediviseur du adc
	  ADC1 ->CR1     |=1<<8;//ADC mode Scan
	  ADC1 ->CR1    |=1<<11;//ADC mode discontinous
	  ADC1 ->CR1    |=0x0<<13;//Discontinous number=1
	  ADC1 ->SQR1    |=0x2<<20;//Nombre de conversion
	  ADC1 ->SQR3    =0x1481;//CH1==>CH4==>CH5
	  ADC1 ->SMPR2   =0x000;//reglage du temps de conversion = 3 cycles
	  ADC1 ->CR2     =0x400; //EOCS=1
	  ADC1 ->CR1    |=1<<5; //interrupt enable of EOC
	  NVIC_EnableIRQ(ADC_IRQn);//Enable interrupt request
	  NVIC_SetPriority(ADC_IRQn,6);
	  //**********config_GPIO*************************************
	   RCC->AHB1ENR  |= 0x1;
	   GPIOA->MODER  |=0xF0C;//  A1  A4 et A5 analog mode

	   //**********config of TIM2**************************************
	   RCC ->APB1ENR|=0x1;//TIM2 Clock Enable
	   TIM2->PSC      = 41999; //DT=Ti*(1+ARR) // Fclock/PSC+1=Fi ==>1/Tclock*(PSC+1)=1/Ti
	   TIM2->ARR      =299;//T=1s
	   TIM2->CR2      |= 2<<4;

	   ADC1 ->CR2    |=0x1<<28;// Trigger detection on the rising edge
	   ADC1 ->CR2    |=0x6<<24;//using TIM2 to control ADC1
	   /************************************************************/
	  // TIM2->CR1|=0x1;//enable tim2
	   ADC1 ->CR2 |=0x1; //Enable ADC1

}
int ADC_read(ADC_TypeDef *ADCx)
{
	TIM2->CR1|=0x1;//enable tim2

   while(!(ADCx->SR & (ADC_SR_EOC)));
   // return data
	return (ADCx->DR);
}

void config_gpio(void)
{
		//  Enable GPIOD
		RCC->AHB1ENR|=0x8;
 	  GPIOD->MODER  |=0x55<<24; 	// D12 D13 D14 D15  en mode digital output

	 //configuration GPIO FOR USART
		RCC->AHB1ENR|=0x1;		//GPIO_A_EN
		GPIOA->MODER|=0x20;		//PA2=AF
	  GPIOA->AFR[0]|=0x7<<8;	//PA2(TX)->mux7
}
void config_usart2(void)
 {
	 RCC->APB1ENR|=0x1<<17;		//usart2_clk_EN
	 USART2->BRR|=0x1120;			//BRR=16Mhz/115200
	 USART2->CR1|=0x01<<13; 		//ENABLE USART2
	 USART2->CR1|=0x01<<3;		//ENABLE TE
 }

 void send_string_USART2( char *pt)
{
  while(*pt)
  {
    while (!(USART2->SR&0x80)) ; 	//test TXE
    USART2->DR=*pt;

    pt++;
  }
}

void delay(int count)
  {
 	 while(count--);
  }

void Config_I2C1()
{
	RCC->APB1ENR|=0x1<<21;

	//configuration GPIOB
	RCC ->AHB1ENR |=0x2;//Enable GPIOB
	GPIOB->MODER  |=0xA<<12;//PB6 et PB7 en mode alternate function
	GPIOB->OTYPER |= 3<<6;//config PB6 et PB7 en open drain
	GPIOB->OSPEEDR|=0xF<<12;//high speed pins
	GPIOB->PUPDR|=0x5<<12;//Pull up bit 6 et 7

	GPIOB->AFR[0]|=0x4<<24;//Lier SCL avec PB6
	GPIOB->AFR[0]|=0x4<<28;//Lier SDA avec PB7

	//reset the I2C
	//I2C1->CR1 |=(1<<15);
	//I2C1->CR1 &=~(1<<15);

	I2C1->CR2 |=0x8;	//Fi=8Mhz
	I2C1->CCR=40;//CCR=(8MHZ/100KHZ*2.2)=36         pr fi =8Mhz
	I2C1->TRISE=9;

	I2C1->CR1 |=0x1;// enable I2C1
}

void I2C_start(void)
{
  I2C1->CR1|=0x1<<8;		//start génération
  while(!(I2C1->SR1&(1<<0)));	//test start bit
}

void I2C_adress(uint8_t adress)
{
  I2C1->DR=adress;
  while(!(I2C1->SR1 & (1<<1)));		//test adresse envoyée
  //I2C1->SR1 ;
  I2C1->SR2;	//read SR 1/2 to clear add
}

void I2C_write(uint8_t data)
{
  while (!(I2C1->SR1 & (1<<7)));	//test TXE
  I2C1->DR=data;
//  while (!(I2C1->SR1 & (1<<2)));	//test BTF

}

uint8_t I2C1_Read(uint8_t ack)
{
    if (ack)
    {
    	I2C1->CR1 |= (1 << 10);                          // Générer un signal ACK après la réception
    }
    else
    {
    	I2C1->CR1 &= ~(0 << 10);                         // Générer un signal NACK après la réception
    }
    //while (!(I2C1->SR1 & (1<<6)));                   // Attendre que les données soient reçues
    return I2C1->DR;                                       // Lire les données reçues depuis le registre de données
}

void I2C_stop(void)
{
  I2C1->CR1|=(1<<9);
}

void DS1621_discount(void)
{
	I2C_start();
	I2C_adress(0x90);
	I2C_write(0xAC);	//pointage sur registre config ADC
	I2C_write(0x1);		// discontinue (1 shot)
	I2C_stop();
}

void start_conversion (void)
{
	I2C_start();
	I2C_adress(0x90);
	I2C_write(0xEE);
	I2C_stop();
}

int Read_Temperature(void)
{

	I2C_start();
	I2C_adress(0x90);
	I2C_write(0xAA);
	I2C_stop();
	I2C_start();
	I2C_adress(0x91);

	I2C1->CR1 |= (1 << 10);                          // Générer un signal ACK après la réception
	while (!(I2C1->SR1 & (1<<6)));                   // Attendre que les données soient reçues
	temp1=I2C1->DR;

	I2C_stop();

	return temp1;

}
void vTask3(void *p)
{
    while (1)
    {  /* send_string_USART2("task3\r\n");
    delay(100);*/
    	 if (xSemaphoreTake(semcount1, portMAX_DELAY))
        {
           // CHx_VALUE = ADC_read(ADC1);
    		xQueueSend(ADC_QUEUE, &CHx_VALUE, portMAX_DELAY);
            sprintf(b1, "CHx_VALUE:%d\r\n", CHx_VALUE);
            send_string_USART2(b1);
           // xSemaphoreGive(semcount);
        }
    }
}


void vTask4(void *p)
{
    while (1)
    {    /*send_string_USART2("task4\r\n");
    delay(100);*/
        if (xSemaphoreTake(semcount1, portMAX_DELAY))
        {
            //CHy_VALUE = ADC_read(ADC1);

            sprintf(b2, "CHy_VALUE:%d\r\n", CHy_VALUE);
            send_string_USART2(b2);
            xQueueSend(ADC_QUEUE, &CHy_VALUE, portMAX_DELAY);
            //xSemaphoreGive(semcount);
        }
    }
}

void vTask5(void *p)
{
    while (1)
    {
    	/*send_string_USART2("task5\r\n");
    	delay(100);*/
       if (xSemaphoreTake(semcount1, portMAX_DELAY))
        {
           // CHz_VALUE = ADC_read(ADC1);

            sprintf(b3, "CHz_VALUE:%d\r\n", CHz_VALUE);
            send_string_USART2(b3);
            xQueueSend(ADC_QUEUE, &CHz_VALUE,portMAX_DELAY);
            //xSemaphoreGive(semcount);
        }
    }
}
void vTask2(void * p)
{
	while(1)
	{
	//	send_string_USART2("task2\r\n");


      if ( xQueueReceive(ADC_QUEUE, &ADC_VALUE[0], portMAX_DELAY)&&
      xQueueReceive(ADC_QUEUE, &ADC_VALUE[1], portMAX_DELAY)&&
      xQueueReceive(ADC_QUEUE, &ADC_VALUE[2], portMAX_DELAY))
      {  t=Read_Temperature();

		    		I2C1->CR1 &= ~(1 << 10);                         // Générer un signal NACK après la réception
		    		//while (!(I2C1->SR1 & (1<<6)));                   // Attendre que les données soient reçues
		    		temp2=I2C1->DR;

		    		if (temp2 !=0)
		    		{
		    			sprintf(Temp, "T=%d,5 °C\r\n ", t);

		    		}
		    		else
		    		{
		    		   sprintf(Temp, "T=%d °C\r\n ", t);

		    		}
		    		send_string_USART2(Temp);

			//prep buffer
			sprintf(buffer,"CH1=%d,CH2=%d,CH3=%d,temp=%d\r\n",ADC_VALUE[0],ADC_VALUE[1],ADC_VALUE[2],t);

			 send_string_USART2(buffer);

			//Send buffer to buffer queue
			xQueueSendToFront(Buffer_QUEUE,buffer ,portMAX_DELAY);
			delay(10000);
      }
    }
}

void vTask1(void * p)
{
	while(1)
	{    //send_string_USART2("task1 \r\n");
		if( xQueueReceive(Buffer_QUEUE , &data ,portMAX_DELAY));
		 {
		 //SEND STRING

			 send_string_USART2(data);
            // delay(10000);

		 }
    }
}

void vTask0(void * p)
{
	while(1)
	{
		//send_string_USART2("task0\r\n");
		if(xSemaphoreTake(sem , portMAX_DELAY))
		{
		send_string_USART2("task0\r\n");

		WWDG->CR|=0x1<<7;
		}
    }
}

void ADC_IRQHandler(void)
{
    if (ADC1->SR & (1 << 1))
    {   BaseType_t ur = pdFALSE;
    	 send_string_USART2("adc\r\n");
    	        i++;
    	        if (i == 1)
    	        {
    	            CHx_VALUE=ADC1->DR;
    	            xSemaphoreGiveFromISR(semcount1, pdTRUE);
    	            portYIELD_FROM_ISR(ur);
    	        }
    	        if(i==2)
    	        {
				CHy_VALUE=ADC1->DR;
				xSemaphoreGiveFromISR(semcount1, pdTRUE);
				portYIELD_FROM_ISR(ur);
    	            	        }
    	        if (i==3)
    	        {
    	        	i=0;
    	           CHz_VALUE=ADC1->DR;
    	            xSemaphoreGiveFromISR(semcount1, pdTRUE);
    	             portYIELD_FROM_ISR(ur);
    	             i=0;

    	            	        }
    }
}

void  EXTI0_IRQHandler(void)
{
	if(EXTI->PR&0x1!=0)
	{   // BaseType_t ur = pdFALSE;
		send_string_USART2("exti\r\n");
		WWDG->CR|=0x1<<7;

        //xSemaphoreTakeFromISR(sem ,pdTRUE);
			// portYIELD_FROM_ISR(ur);
		EXTI->PR=0x1;


	}
}
