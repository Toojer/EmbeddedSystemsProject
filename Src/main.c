/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************

  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  *
  * @retval None
  */

void transmitString(char* character);
void transmitChar(char);
uint8_t transmitComplete(void);
void initiateTransaction(uint8_t Addr, uint8_t numBytes,uint8_t rd_wr);
uint8_t setupGyro(uint8_t Addr, uint8_t value);	
uint8_t readData(void);
int16_t read2byteData(void);
uint8_t transmitData(uint8_t data,uint8_t numBytes);
uint8_t transmitComplete(void);

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Enable the Peripheral Clocks*/
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOAEN;   //enable GPIOC and GPIOB GPIOA Clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN|RCC_APB1ENR_TIM3EN|RCC_APB1ENR_TIM2EN|RCC_APB1ENR_I2C2EN; //enable USART3 clock Enable Timer 2 (TIM2EN) and Timer 3 (TIM3EN) registers  and I2C2 6.4.8 Peripheral Manual
	 
	/* Set up USART3 */
	GPIOC->AFR[1] |= (GPIO_AF1_USART3 << GPIO_AFRH_AFSEL10_Pos)|(GPIO_AF1_USART3 << GPIO_AFRH_AFSEL11_Pos);  //Set PC10 and PC11 AFR registers to USART3
	GPIOC->MODER  |= (2 << GPIO_MODER_MODER10_Pos)|(2 << GPIO_MODER_MODER11_Pos); //PC10 USART3_TX and PC11 USART3_RX to Alternate function mode
	USART3->BRR  = HAL_RCC_GetHCLKFreq()/115200;
	USART3->CR1 |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE; //Enable UART,TX,RX and RXInterrupt
	  // Enable USART_Rx Interrupt //
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,4); //set interrupt priority or USART
	/* End USART3 setup */
	
	/* Setup PWM */
	  // Configure Timer2 //
	TIM2->PSC  = 8000-1;   /*Prescalar will divide by 8000 giving 1kHz clock and 1ms resolution 18.3.1 for explanation and 18.4.11 for register information Peripheral manual*/
	TIM2->ARR  = 500;      //Periph Man 18.4.3 TIMER2 count to 500ms 18.4.10 Peripheral Manual
	TIM2->DIER = 1;				 //Periph Man 18.4.4 Update interrupt enable
  TIM2->CR1  = 0x0081;   //Periph Man 18.4.1 Auto Reload not buffered; Edge-aligned; Downcounter; Counter not stop;Update enabled Counter enabled
	  // Configure Timer3 //
	TIM3->PSC  = 1000-1;    //Set clock to 800kHz  
	TIM3->ARR  = 10;     // changes every 1.25ms
	  // setup timer 1 and timer 2 //
	TIM3->CCMR1= 0x6868;  // Set CC1S CC2S to output mode  Peripheral Man 18.4.7  OC1M PWM Mode2 OC2M Mode 2
	TIM3->CCMR2= 0;     
	TIM3->CCR1 = 0;			 //Set to 0% of ARR
	TIM3->CCR2 = 0;      //Set CCR to 0% of ARR
	TIM3->CCER = 0x0011;  //18.4.9 Set Channel 1 and Chan2 to output enable
  TIM3->CR1  = 0x0081;  //enable timer and 18.4.1 periph man
	  // Set outputs to Tim3_ch1 and TIM3_CH2 //
	//the cancelled out function is to set up PC6,7 to be the PWM outputs
  //GPIOC->AFR[0] |= GPIO_AF0_TIM3 << GPIO_AFRL_AFSEL6_Pos| GPIO_AF0_TIM3 << GPIO_AFRL_AFSEL7_Pos; //set PC6,7 to Alt function 0
	//GPIOC->MODER  |= (2 << GPIO_MODER_MODER6_Pos)|(2<<GPIO_MODER_MODER7_Pos)|(1<<GPIO_MODER_MODER8_Pos)|(1<<GPIO_MODER_MODER9_Pos);//Set LEDs PC6,7 alternate function , Set LEDs to general purpose PC8,9
	GPIOB->AFR[0] |= GPIO_AF1_TIM3 << GPIO_AFRL_AFSEL4_Pos| GPIO_AF1_TIM3 << GPIO_AFRL_AFSEL5_Pos; //set PB4,5 to Alt function 1
	GPIOB->MODER  |= 2<<GPIO_MODER_MODER4_Pos | 2<<GPIO_MODER_MODER5_Pos; //set PB4,5 to Alternate function mode
    // Configure Interrupt //
	NVIC_EnableIRQ(TIM2_IRQn);  		//Enable TIM2 interrupt Section 4.2 Core Programming M0 Manual
	NVIC_SetPriority(TIM2_IRQn,6);  //Configure EXTI0 interrupt priority Section 4.2.1 Core Prog M0 Manual
  /* End PWM Setup */
  
	/* I2C2 Setup */
	GPIOB->MODER  |= 2 << GPIO_MODER_MODER11_Pos | 2 << GPIO_MODER_MODER13_Pos; //Set PB11 PB13 to Alternate function mode
	GPIOB->MODER  |= 1 << GPIO_MODER_MODER14_Pos;  //Set PB14 to output mode
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11|GPIO_OTYPER_OT_13; //PB11 and PB13 open-drain
	GPIOB->AFR[1] |= GPIO_AF1_I2C2 << GPIO_AFRH_AFSEL11_Pos | GPIO_AF5_I2C2 << GPIO_AFRH_AFSEL13_Pos; //PB11 and PB13 Alternate function I2C2
	GPIOC->MODER  |= 1|1 << GPIO_MODER_MODER6_Pos|1<<GPIO_MODER_MODER7_Pos|1<<GPIO_MODER_MODER8_Pos|1<<GPIO_MODER_MODER9_Pos; //PC0 to output mode LEDs PC6-9 to output
	GPIOC->ODR     = 1; //PC0 Set High for address on GYRO
	
	/* Configure I2C2 */
	I2C2->TIMINGR |= (1<<I2C_TIMINGR_PRESC_Pos)|(0x13 << I2C_TIMINGR_SCLL_Pos)|(0xF << I2C_TIMINGR_SCLH_Pos)|(0x2 << I2C_TIMINGR_SDADEL_Pos)|(0x4<<I2C_TIMINGR_SCLDEL_Pos); //Setting the timing according to Table 91
	I2C2->CR1      = I2C_CR1_PE;
	GPIOB->ODR     = GPIO_PIN_14; //configure address on Gyroscope
	
	/*Configure GPIOA Pins for Motor Direction*/
	 GPIOA->MODER |= 1 << GPIO_MODER_MODER1_Pos | 1 << GPIO_MODER_MODER2_Pos; //set PC1 and PC2 to general purp OUtput
   //GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_1|GPIO_OTYPER_OT_2);
   HAL_Delay(100);
	/* End I2C2 Setup */
	
	/* Enable the Gyroscope */
		initiateTransaction(0x6B,2,0);
	  
	  uint8_t TXComplt = setupGyro(0x20,0x0B);
	  
    HAL_Delay(500);
		
	/* --Lab 3 - PWM controller  */ //I have this setup for PC6 and PC7 getting Tim2 and Tim3 outputs.  Change to PBx GPIO pin
	/* --Lab 4 - USART for console control   Using PC10 and PC11*/
	/* --Lab 5 - I2C for Gyroscope    Using PB15 SDA    PB13 SCL  PC0 SPI/I2C mode   PB14 Slave addr of gyroscope*/
	/* Lab 8 Encoder Counter*/

	uint8_t temp = 0;
  while (1)
  {
   	 
	 // ****** Read Out_Y_L and Out_Y_H ***** */
	  HAL_Delay(10);
		initiateTransaction(0x6B,1,0); //write transaction
	  TXComplt = transmitData(0xAA,1);
		
		initiateTransaction(0x6B,2,1); //set read transaction
	  int16_t RxData = read2byteData();
		
		uint8_t TXComplete = transmitComplete();
		if(RxData > 1000)
			{
			GPIOC->ODR |= GPIO_PIN_6;
		  TIM3->CCR1 = 3; //set to 20% of CCR
			}
		else if(RxData < -1000)
			{
			GPIOC->ODR |= GPIO_PIN_7;
		  TIM3->CCR1 = 3;
			}
			if(RxData > 2000)
			{
			GPIOC->ODR |= GPIO_PIN_6;
		  TIM3->CCR1 = 4; //set to 20% of CCR
			}
		else if(RxData < -2000)
			{
			GPIOC->ODR |= GPIO_PIN_7;
		  TIM3->CCR1 = 4;
			}
		else
			GPIOC->ODR &= ~(GPIO_PIN_6|GPIO_PIN_7); //red blue
		
		HAL_Delay(10);  
 /* End Code for Gyroscope */			 
} //end while loop

}

/**
  * @brief System Clock Configuration
  * @retval None
  */

/* Code for I2C for Gyro */
void initiateTransaction(uint8_t Addr, uint8_t numBytes,uint8_t rd_wr)
{
	  I2C2->CR2 = 0; //Clear CR2
	  I2C2->CR2 |= Addr << 1; //write address
	  I2C2->CR2 |= numBytes << I2C_CR2_NBYTES_Pos; //num of bytes 
  	
	  if(rd_wr) //master read
			I2C2->CR2 |= I2C_CR2_RD_WRN; //assign a read
		else
	    I2C2->CR2 &= ~I2C_CR2_RD_WRN; //set to write transaction
		
		I2C2->CR2 |= I2C_CR2_START; //start transaction.
}

uint8_t setupGyro(uint8_t Addr, uint8_t value)
{
	 while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS)){}//wait for TXIS ready (TX Reg empty)
	 I2C2->TXDR = Addr; //write to TX register
	 while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS)){}//wait for TXIS ready (TX Reg empty)
	 I2C2->TXDR = value; 
	 while(((I2C2->ISR & I2C_ISR_TC) != I2C_ISR_TC)){} //wait for Transfer Complete
	 return 1;	
	
}
uint8_t readData(void)
{
    while(((I2C2->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE)){}
	  uint8_t RxData = I2C2->RXDR;
	  return RxData;
}
int16_t read2byteData(void)
{
	while(((I2C2->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE)){}
	uint8_t RxDataL = I2C2->RXDR;
	while(((I2C2->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE)){}
	uint8_t RxDataH = I2C2->RXDR;
		
	int16_t RxData = RxDataH << 8 | RxDataL;
	return RxData;
}

uint8_t transmitData(uint8_t data,uint8_t numBytes)
{
	for(;numBytes>0;numBytes-=1)
	{
   while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS)){}//wait for TXIS ready (TX Reg empty)
	 I2C2->TXDR = data; //write to TX register
	}
	while(((I2C2->ISR & I2C_ISR_TC) != I2C_ISR_TC)){} //wait for Transfer Complete
	return 1;	
}

uint8_t transmitComplete(void)
{
	while(((I2C2->ISR & I2C_ISR_TC) != I2C_ISR_TC)){} //wait for Transfer Complete
	I2C2->CR2 |= I2C_CR2_STOP;
	return 1;
}
/* End Code for I2C for Gyro */

/*Code for USART*/
void clearString(char* stringVal,int n)
{
	   for(;n>-1;n--)
			{
			  stringVal[n] = NULL;
			}
}


uint8_t matchString(char* str1, char* str2,uint8_t n)
{
	uint8_t pass = 0;
	
	while(n >= 1)
	{
		if(str1[n-1]==str2[n-1])
		{
			pass=1;
			n-=1;
		}
		else
		{
			 pass=0;
		   break;
		}
	}
	return pass;
}

void transmitChar(char character)
{
    int x = ((USART3->ISR)&(0x80)); 
	  if(x>0){
	    USART3->TDR = character;
		}			//Turn on the Red LED USART TX
}	

void transmitString(char* character)
{
	uint8_t x = 0; /* checks the bit 7 USART_ISR  TXE Register, 
	                                       Transmit register empty. Periph Man 27.8.8
	                                       **NOTE**This will return 0x80 if true			*/
	while(*character !='\0')
	 {
		x = ((USART3->ISR)&(0x80)); 
	  if(x>0)
		{
		  uint8_t sendChar = *(character);
	    USART3->TDR = sendChar;
			character+=1;
		}
	 }
}
/* End Code for USART*/



void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
