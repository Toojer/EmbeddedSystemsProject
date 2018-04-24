
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
#define MPU6050_adr 				0x68
#define MPU6050_first 			0x3B
#define MPU6050_WHO_AM_I 		0x75

#define SMPLRT_DIV 				0x19
#define CONFIG 						0x1A
#define GYRO_CONFIG				0x1B
#define ACCEL_CONFIG			0x1C
#define FIFO_EN						0x23
#define SIGNAL_PATH_RESET 0x68
#define USER_CTRL					0x6A
#define	PWR_MGMT_1				0x6B

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void gyro_init(void);
void i2c_write(uint16_t address, uint16_t reg, uint16_t value);
void i2c_read(uint8_t reg, uint8_t num_bytes, uint8_t *buffer);



uint8_t buffer[14];
int16_t gyro_data[3];

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void config_reg(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint8_t)MPU6050_adr<<1, data, 2, 1000) != HAL_OK)
	{
	}
}

void read_gyro()
{
	uint8_t reg = MPU6050_first;
	uint8_t data[6];
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MPU6050_adr<<1, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)MPU6050_adr<<1, data, 6, 1000) != HAL_OK);
	
	gyro_data[0] = (int16_t)(data[0] << 8 | data[1]);
	gyro_data[1] = (int16_t)(data[2] << 8 | data[3]);
	gyro_data[2] = (int16_t)(data[4] << 8 | data[5]);
}

void i2c_read_gyro()
{
	uint8_t reg = MPU6050_first;
	uint8_t num_bytes = 6;
	uint8_t data[num_bytes];
	
	i2c_read(reg, 6, data);
	
	gyro_data[0] = (int16_t)(data[0] << 8 | data[1]);
	gyro_data[1] = (int16_t)(data[2] << 8 | data[3]);
	gyro_data[2] = (int16_t)(data[4] << 8 | data[5]);
}

void i2c_init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	GPIOB->MODER &= ~((GPIO_MODER_MODER0) << 13);
	GPIOB->MODER &= ~((GPIO_MODER_MODER0) << 15);
	GPIOB->MODER |= (1 << 13); 
	GPIOB->MODER |= (1 << 15);
	
	GPIOB->OTYPER &= ~((GPIO_OTYPER_OT_0) << 6); //Reset the OTYPER to 0
	GPIOB->OTYPER &= ~((GPIO_OTYPER_OT_0) << 7); 
	GPIOB->OTYPER |= (1 << 6); //built in pullup
	GPIOB->OTYPER |= (1 << 7);
	
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << 12);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << 14);
	GPIOB->PUPDR |= (1 << 12);
	GPIOB->PUPDR |= (1 << 14);
	
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << 12); //reset ospeedr
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << 14); //reset ospeedr
	GPIOB->OSPEEDR &= ~(1 << 12); //low speed
	GPIOB->OSPEEDR &= ~(1 << 14); //low speed
	
	GPIOB->AFR[0] &= ~(0xF << 24);
	GPIOB->AFR[0] &= ~(0xF << 28);
	GPIOB->AFR[0] |= (1 << 24);
	GPIOB->AFR[0] |= (1 << 28);
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	I2C1->CR1 &= ~(1);
	I2C1->CR1 &= ((uint32_t)0x00CFE0FF);
	
	//Timing
	I2C1->TIMINGR = 0x00310309;
	
	I2C1->CR1 |= 1; //Set the PE bit
	
	I2C1->OAR1 = 0;
	I2C1->OAR2 = 0;
	I2C1->OAR1 = 0;
	
	I2C1->OAR1 |= I2C_OAR1_OA1EN; //address 1 ack
	
	//set i2c mode
	I2C1->CR1 |= 0x0; //i2c mode
	
	I2C1->CR2 &= ~((uint32_t)0x07FF7FFF); //clear CR2
	I2C1->CR2 |= 0x0; //ack config
	I2C1->CR2 &= ~I2C_CR2_SADD; //reset slave
	
	I2C1->CR2 |= (uint32_t) 0x68; //set slave address
	I2C1->CR1 |= I2C_CR1_PE;
}

void gyro_init()
{
	i2c_write(MPU6050_adr, SMPLRT_DIV, 0x07);
	i2c_write(MPU6050_adr, CONFIG, 0x06);
	i2c_write(MPU6050_adr, GYRO_CONFIG, 0x0);
	i2c_write(MPU6050_adr, ACCEL_CONFIG, 0x0);
	i2c_write(MPU6050_adr, FIFO_EN, 0x0);
	i2c_write(MPU6050_adr, SIGNAL_PATH_RESET, 0x0);
	i2c_write(MPU6050_adr, USER_CTRL, 0x0);
	i2c_write(MPU6050_adr, PWR_MGMT_1, 0x0);
}

void i2c_write(uint16_t addr, uint16_t reg, uint16_t value)
{
	while(((I2C1->ISR >> 15) & 0x1) == 1);
	
	I2C1->CR2 = 0;
	I2C1->CR2 &= ~(1<<11);
	I2C1->CR2 |= (addr << 1);
	I2C1->CR2 |= 2<<16; //transmit 2 bytes
	I2C1->CR2 |= 1<<25; //enable autoend
	I2C1->CR2 &= ~(1 << 10); //write mode
	I2C1->CR2 |= (1<<13); //start transfer
	
	while(((I2C1->ISR >> 1) & 1) == 0); //wait for transfer to finish
	
	I2C1->TXDR = reg; //set data
	
	while(((I2C1->ISR >> 1) & 1) == 0); //loop until transfer complete
	
	I2C1->TXDR = value;
	
	while(((I2C1->ISR >> 5) & 0x1) == 0); //wait for stop bit to be set by autoend
	
	I2C1->ISR &= ~(1 << 5); //clear the stop bit
}

void i2c_read(uint8_t reg, uint8_t num_bytes, uint8_t *buffer)
{
	uint8_t counter = 0;
	while(((I2C1->ISR >> 15) & 0x1) == 1);
	
	I2C1->CR2 = 0; //clear cr2
	I2C1-> CR2 &= ~(1 << 11); //7 bit mode
	I2C1->CR2 |= (uint8_t)MPU6050_adr << 1; //set address
	I2C1->CR2 |=1 << 16; //read 1 byte
	I2C1->CR2 &= ~(1 << 25); //software end
	I2C1->CR2 |= ~(1 << 10); //write mode
	I2C1->CR2 |= 1 << 13; //set start
	
	while(((I2C1->ISR >> 1) & 0x1) == 0);
	
	I2C1->TXDR = reg; //register to read from
	
	while(((I2C1->ISR >> 6) & 0x1) == 0); //wait for TC
	
	I2C1->CR2 = 0; //clear CR2
	I2C1->CR2 &= ~(1 << 11); //7 bit addressing
	I2C1->CR2 |= (uint8_t)MPU6050_adr << 1; //set address
	
	I2C1->CR2 |= num_bytes << 16;  //set num bytes to read
	I2C1->CR2 |= 1 << 25; //autoend mode
	I2C1->CR2 |= 1 << 10; //read mode
	I2C1->CR2 |= 1 << 13; //start
	
	for(; counter < num_bytes; counter++)
	{
		while(((I2C1->ISR >> 2) & 1) == 0); //wait for RXNE bit to signal data ready
		buffer[counter] = I2C1->RXDR; //read data
	}
	
	while(((I2C1->ISR >> 5) & 1) == 0); //wait for stop bit
	
	I2C1->ISR &= ~(1 << 5); //clear stop.
}

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	/* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  __HAL_RCC_GPIOC_CLK_ENABLE();
	MX_GPIO_Init();
  //MX_I2C1_Init();
	i2c_init();
	gyro_init();
  /* USER CODE BEGIN 2 */
	//uint8_t WHO_AM_I = (uint8_t)0x0;//I2C_OAR1_OA1EN; (uint8_t)MPU6050_WHO_AM_I;
	//uint8_t temp;
	//uint8_t d[2];
	
	GPIO_InitTypeDef initStr = {GPIO_PIN_6| GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
  
	HAL_GPIO_Init(GPIOC, &initStr);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |GPIO_PIN_9, GPIO_PIN_SET);
	
	//GPIOC->ODR ^= GPIO_PIN_6;
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	//uint8_t address = MPU6050_adr<<1;
	
	//uint8_t dataBuffer[14];
	//uint8_t n_bytes = 14;
	
	
	
  while (1)
  {
		i2c_read_gyro();
		if(gyro_data[0] > 4000)
		{
			GPIOC->ODR &= ~(GPIO_PIN_7);
			GPIOC->ODR |= GPIO_PIN_6;  
		}
    else if(gyro_data[0] < -4000)
		{
			GPIOC->ODR &= ~(GPIO_PIN_6);
			GPIOC->ODR |= GPIO_PIN_7;
		}			
		
		if(gyro_data[1] > 4000)
		{
			GPIOC->ODR &= ~(GPIO_PIN_7);
			GPIOC->ODR |= GPIO_PIN_6;  
		}
    else if(gyro_data[1] < -4000)
		{
			GPIOC->ODR &= ~(GPIO_PIN_6);
			GPIOC->ODR |= GPIO_PIN_7;
		}			
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00310309;//0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
