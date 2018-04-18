/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"


/* I created this code for the Timer2 interrupt */
void TIM2_IRQHandler(void)   //This comes from startup_stm32f072xb.s  Lab2 explains it
{
	  //GPIOC->ODR  ^= GPIO_PIN_8; //toggle value of orange LED
	  //GPIOC->ODR  ^= GPIO_PIN_9; //toggle green LED
	  TIM2->SR     = 0;           //Clear flag
}

uint8_t GyroDataOutput(uint8_t GetSet, uint8_t valueToSet);
void transmitChar(char);
void transmitString(char*);
void clearString(char* stringVal,int n);
uint8_t matchString(char* str1, char* str2,uint8_t n);

void USART3_4_IRQHandler(void)
{
	char* message = "\nError has to be 'gyro','fwd', 'bck', 'lft' or 'rgt'\n";
	char* wrong   = "\nYou Ruined It!\n";
	char* command = "\nCMD?";
	char* CMDACK0 = "\nTurned OFF LED";
	char* CMDACK1 = "\nTurned ON LED";
	char* CMDACK2 = "\nToggled LED";
	char* gyroMSG = "Do you want Gyro Info On?";
	
	static uint16_t GPIOPin = 0;
	static uint8_t    state = 0;
	char             RxChar = NULL;
	static char StrRxChar[10];
	static uint16_t n=0;
	
	RxChar = USART3->RDR; 
	
	transmitChar(RxChar);
	
	StrRxChar[n]=RxChar;
	n+=1;
	
	if(RxChar == 13) //carriage return
	{
	
	switch(state)
		  { 
		    case 0:
	        if((matchString("fwd",StrRxChar,n-1)))
				  { 
					  GPIOPin = GPIO_PIN_6;
				    transmitString(command);
						transmitChar('\n');
					  state = 1;
						clearString(StrRxChar,n);		
						n=0;
				  }
				  else if(matchString("bck",StrRxChar,n-1))
				  {
					  GPIOPin = GPIO_PIN_9;
				    transmitString(command);
						transmitChar('\n');
						state = 1;
						clearString(StrRxChar,n);
            n=0;						
				  }
			    else if( matchString("rgt",StrRxChar,n-1))
				  {
					  GPIOPin = GPIO_PIN_7;
				    transmitString(command);
						transmitChar('\n');
					  state = 1;
						clearString(StrRxChar,n);
						n=0;
				  }
				  else if(matchString("lft",StrRxChar,n-1))
				  {
					  GPIOPin = GPIO_PIN_8;
				    transmitString(command);
						transmitChar('\n');
					  state = 1;
						clearString(StrRxChar,n);	
					  n=0;
				  }
					else if(matchString("gyro",StrRxChar,n-1))
					{
						state = 2;
					  GyroDataOutput(1,0); //set gyro to not output
						transmitChar('\n');
						transmitString(gyroMSG);
						transmitChar('\n');
						clearString(StrRxChar,n);
					
					}
				  else 
				  {
					  transmitString(message);
						transmitChar('\n');
  				  state = 0;
						clearString(StrRxChar,n);
						n=0;
					}
					
			    break;
		    case 1:
			    if(matchString("off",StrRxChar,n-1))
				  {
  					GPIOC->ODR &= ~GPIOPin;
	  		    transmitString(CMDACK0);
						transmitChar('\n');
						clearString(StrRxChar,n);
						state = 0;
					}
			  	else if(matchString("on",StrRxChar,n-1))
				  {
					  GPIOC->ODR |= GPIOPin;
  			    transmitString(CMDACK1);	
						transmitChar('\n');
						clearString(StrRxChar,n);
						state = 0;
	  			}
		  		else if(matchString("toggle",StrRxChar,n-1))
			  	{
				  	GPIOC->ODR ^= GPIOPin;
			      transmitString(CMDACK2);
						transmitChar('\n');
						clearString(StrRxChar,n);
						state = 0;
  				} 
	  			else
					{
		  		  transmitString(wrong);
						transmitChar('\n');
						clearString(StrRxChar,n);
						state = 0;
				  }
		      break;
				case 2:
					 if(matchString("on",StrRxChar,n-1))
					 {	
					  GyroDataOutput(1,1); //set gyro to output
				   }
					 else
					 {
						GyroDataOutput(1,0);
					 }						 //set gyro to not output
					 clearString(StrRxChar,n);
					 state = 0;
				  break;
	  }//end switch statement
	}//end if	
}//end function

/************************************************
 **** End of my USART3 Interrupt Function *******/


/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
