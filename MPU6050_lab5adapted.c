/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
void startWrite(uint8_t Addr, uint8_t numBytes)
{
	  I2C2->CR2 = 0; //Clear CR2
	  I2C2->CR2 |= Addr << 1; //write address
	  I2C2->CR2 |= numBytes << I2C_CR2_NBYTES_Pos; //num of bytes 
  
		I2C2->CR2 &= ~I2C_CR2_RD_WRN; //assign a write
		
		I2C2->CR2 |= I2C_CR2_START; //start transaction.
}

void startRead(uint8_t Addr, uint8_t numBytes)
{
	  I2C2->CR2 = 0; //Clear CR2
	  I2C2->CR2 |= Addr << 1; //write address
	  I2C2->CR2 |= numBytes << I2C_CR2_NBYTES_Pos; //num of bytes 
  
		I2C2->CR2 |= I2C_CR2_RD_WRN; //assign a read
		
		I2C2->CR2 |= I2C_CR2_START; //start transaction.
}

volatile int x = 0;
volatile int y = 0;

int main(void)
{
	HAL_Init();
  SystemClock_Config();

  //uint8_t bytes = 0;
	
  //uint16_t result = 0;
	int16_t result = 0;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOBEN;
	
	//Enable I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	//Setup LEDs
	GPIO_InitTypeDef initStr = {GPIO_PIN_6| GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	HAL_GPIO_Init(GPIOC, &initStr);
	
	//PB11 AF, open drain, output, I2C2_SDA (AF1)
	GPIOB->MODER |= 2<<GPIO_MODER_MODER11_Pos; //set moder 11 at bit [22:23] to AF mode
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11; //set bit 11 (1 open-drain, 0 push-pull)
	GPIOB->AFR[1] |= GPIO_AF1_I2C2 << GPIO_AFRH_AFSEL11_Pos; //set bits [15:12] to AF1
	//GPIO_AF1_I2C2 << GPIO_AFRH_AFSEL11_Pos
	//PB13 AF, open drain, output, I2C2_SCL (AF5)
	GPIOB->MODER |= 2<<GPIO_MODER_MODER13_Pos; //set moder 13 at bit [26:27] to AF mode
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13; //set bit 13 (1 open-drain, 0 push-pull)
	GPIOB->AFR[1] |= GPIO_AF5_I2C2 << GPIO_AFRH_AFSEL13_Pos; //set bits [23:20] to AF5
	//PB14 push-pull, output, initialize high
	GPIOB->MODER |= 1<<GPIO_MODER_MODER14_Pos; //set moder 14 at bit [28:29] to Output
	//GPIOB->OTYPER |= 0<<14; //set bit 14 (1 open-drain, 0 push-pull)
	GPIOB->ODR |= GPIO_PIN_14; //set bit 14 high for GPIOB
	
	//PC0 push-pull, output, initialize high
	GPIOC->MODER |= 1; //set bit [0:1] to 1 for Output
	//GPIOC->OTYPER |= 0; // set bit 0 to 0 for push-pull
	GPIOC->ODR |= 1; //set bit 0 high for GPIOC
	
	//Set Frequency from Fig. 5.4
	I2C2->TIMINGR |= 0x1<<I2C_TIMINGR_PRESC_Pos; //PRESC [31:28]
	I2C2->TIMINGR |= 0x13<<I2C_TIMINGR_SCLL_Pos; //SCLL [7:0]
	I2C2->TIMINGR |= 0xF<<I2C_TIMINGR_SCLH_Pos; //SCLH [15:8]
	I2C2->TIMINGR |= 0x2<<I2C_TIMINGR_SDADEL_Pos; //SDADEL [19:16]
	I2C2->TIMINGR |= 0x4<<I2C_TIMINGR_SCLDEL_Pos; //SCLDEL [23:20]
	
	//Finalize setup by setting PE bit in CR1 (locks CR1)
	I2C2->CR1 |= I2C_CR1_PE; //PE [0]
	//GPIOB->ODR = GPIO_PIN_14; //configure address on Gyro
	HAL_Delay(100);
	
	/*
	startWrite(0x6B, 2);
	//wait for TXIS (success) or NACKF (failure)
	//Trasmit Interrupt Status | Not Acknowledge Received Flag
	// [1]										 | [4]
	
	
	while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS))//(!I2C2->ISR&0b10 || !I2C2->ISR&0b10000) //wait for write or fail
	{
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	//if((I2C2->ISR&2)) //check bit 2 of I2C2 for 
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	//else
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	
	I2C2->TXDR = 0x20;//0x0F; //should contain 0xD4 (check if it should actually be 0x0F)
	
  while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS))
  {
  }
  
  I2C2->TXDR = 0x0F;
  
	while(((I2C2->ISR & I2C_ISR_TC) != I2C_ISR_TC))//(!I2C2->ISR&0b1000000) //check bit 7 for transfer complete flag
	{
	}
  */
  //if(((I2C2->ISR & I2C_ISR_TC) == I2C_ISR_TC))
  //  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	
	//Set up first read
	//I2C2->CR2 |= 0x6B<<1; //[7:1] set to address for SADD
	//I2C2->CR2 |= 0x1<<16; //[23:16] set NBYTES to 1
	//I2C2->CR2 |= 1<<10; //[10] set to 0 for read
	//I2C2->CR2 |= 1<<13; //[13] set to 1 for START
  
	
	//while(!I2C2->ISR&0b100 || !I2C2->ISR&0b10000) //wait for read or fail
	//{
	//}
	//if(I2C2->RXDR == 0xD4)
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
//	else
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	
//	while(!I2C2->ISR&0b1000000) //wait for TC
//	{
//	}
	
	//I2C2->CR2 |= 1<<5; //set stop flag
	
	//end first transfer
	
	
  while (1)
  {
		result = 0;
    //X AXIS
    //result = 0;
    startWrite(0x68,1);
    //transmit data 0xA8
    while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS))
    {
    }
    
    I2C2->TXDR = 0x3B;//0xA8; //write data
  
    while(((I2C2->ISR & I2C_ISR_TC) != I2C_ISR_TC)) {}
    
    startRead(0x68, 1);
    //read 2 bytes
    while(((I2C2->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE))
    {}
    
    result |= I2C2->RXDR;
    
		startWrite(0x68, 1);
		while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS))
    {
    }
		
		I2C2->TXDR = 0x3C;
		
		startRead(0x68, 1);
		
    while(((I2C2->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE))
    {}
    
    result |= I2C2->RXDR<<8;
		x = result;
    
    if(result > 4000)
			GPIOC->ODR |= GPIO_PIN_9;
    else if(result < -4000)
			GPIOC->ODR |= GPIO_PIN_8;  
    else
      GPIOC->ODR &= ~(GPIO_PIN_8|GPIO_PIN_9);
    
    //Y AXIS
    result = 0;
    startWrite(0x68,1);
    //transmit data 0xAA
    while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS))
    {
    }
    
    I2C2->TXDR = 0x3D;//0xAA; //write data
  
    while(((I2C2->ISR & I2C_ISR_TC) != I2C_ISR_TC)) {}
    
    startRead(0x68, 1);
    //read 2 bytes
		
    while(((I2C2->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE))
    {}
    
    result |= I2C2->RXDR;
    
		startWrite(0x68,1);
		while(((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS))
    {
    }
		
		I2C2->TXDR = 0x3E;
		
		while(((I2C2->ISR & I2C_ISR_TC) != I2C_ISR_TC)) {}
    
		
		startRead(0x68, 1);
			
    while(((I2C2->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE))
    {}
    
    result |= I2C2->RXDR<<8;
		y = result;
    
    if(result > 4000)
			GPIOC->ODR |= GPIO_PIN_6;  
    else if(result < -4000)
			GPIOC->ODR |= GPIO_PIN_7;  
		
    else
      GPIOC->ODR &= ~(GPIO_PIN_6|GPIO_PIN_7);
    
    /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
