/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include "leds.h"  //Include LED driver
#include "timer.h"  // Include timer driver
#include "i2c.h"  //Include i2c driver
#include "lsm6dsl.h"  //Include accelerometer driver
#include "ble.h"
#include "../../Drivers/CMSIS/Include/core_cm4.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>  //Include library for printings
#include "../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l475xx.h"  //Include memory map of MCU

TIM_TypeDef* timer; //Declare timer variable for LEDs
volatile int messageIndex;  //Declare variable for counter for bit pair in message
volatile int secondsCounter; //Declare variable for counting seconds that PrivTag was "lost"
/**
 * First four elements are the preamble, remaining 8 are the message itself
 * last four digits of my id are 3782 -> 0b00 00 11 10 11 00 01 10 -> 8 pairs of bits that make up message
 * We are using 4 digits of the PID as the transmit message for P2, after consulting with the TA
 */
volatile uint8_t toTransmit[] = {0b10, 0b01, 0b10, 0b01, 0b00, 0b00, 0b11, 0b10, 0b11, 0b00, 0b01, 0b10};
const int messageLength = 12;  //number of chunks in the LED transmit message
const int g = 16393; //the value of g for the accelerometer sensor given by the spec
const int sensThresh = 8000; //the sensitivity threshold used to determine what qualifies as movement for the accelerometer
const int lostTime = 10; //sets the time in seconds to count to before PrivTag is considered "lost"

volatile int dataAvailable = 0;  ///used in ble.c
volatile int lostFlag = 0;  //flag triggered every 10 seconds after tag is lost
volatile int foundFlag = 0;

//these are the disconnect arrays that would have been used if the connection handle had been 0, ignore now
//uint8_t HCI_DISCONNECT[] = {0x01, 0x06, 0x04, 0x03, 0x00, 0x00, 0x15};    //first byte is # packets (1), next two are op-code little-endian, 0x03 is parameter byte length connection handle is 0x0000, reason is 0x15 (terminated connection due to power off)
//uint8_t HCI_DISCONNECT_COMPLETE[]= {0x04,0x05,0x04,0x00,0x00,0x00,0x16};  //4 packets? (0x04), 0x0e is event code, 0x04 is parameter total byte length, 0x01 for one packet, 0x06 and 0x04 for op-code, no return parameters so last byte is 0x00

SCB_Type* cortexM4; //Declare struct to access cpu registers

SPI_HandleTypeDef hspi3;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

/**
 * Enables SWV ITM Data Console for printf statements
 */
int _write(int file,char *ptr,int len){
	int i = 0;
	for (i = 0; i < len; i++){
		ITM_SendChar(*ptr++);
	}
	return len;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	messageIndex = 0;  //Index counter variable used to iterate through all the binary pairs that make up the message
	secondsCounter = 0; //Seconds counter variable used to count number of seconds that the PrivTag has been "lost"
	timer = TIM2;  //Define LED timer
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	MX_GPIO_Init();
	MX_SPI3_Init();

	//RESET BLE MODULE
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  	leds_init();   //Initialize LEDs
  	//timer_init(timer);  //Initialize timer
  	//timer_set_ms(timer,10000);  //Set the timer to count up to 50 ms, triggers interrupt every 50 ms
  	i2c_init();   //Initialize i2c
  	lsm6dsl_init();  //Initialize accelerometer

  	int16_t x = 0;    //Create variables to store accelerometer x, y, z readings
  	int16_t y = 0;
  	int16_t z = 0;

  	while (1)
  	{
  		lsm6dsl_read_xyz(&x, &y, &z);  //Reads x, y, z values from the accelerometer

  		/*
  		if(lostFlag){  //when lost flag triggered (occurs every 10 seconds)
			lostFlag = 0;   //reset lost flag
			char trans1[] = "32bMons lost for:";     //transmit a BLE packet indicating the amount of seconds tag has been lost
			char trans2[20];
			snprintf(trans2, sizeof(trans2), "%d secs.", (secondsCounter*10));
			int bufLength = strlen(trans2);                      //used to reduce the size of the second buffer containing the time
			updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(trans1)-1, trans1);
			updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, bufLength, trans2);
  		 }*/

  		if(foundFlag){
  			leds_set(0b11);
  			HAL_Delay(1000);
  			leds_set(0b00);
  			foundFlag = 0;
  			lsm6dsl_read_xyz(&x, &y, &z);  //Reads x, y, z values from the accelerometer
  		}
  		/*
  		lsm6dsl_read_xyz(&x, &y, &z);  //Reads x, y, z values from the accelerometer
  		if ((x > sensThresh || x < -sensThresh) || (y > sensThresh || y < -sensThresh) || (z > g+sensThresh || z < g-sensThresh)){   //Considers the PrivTag to be moving if x, y, or z exceed the sensitivity threshold
  		//if (foundFlag){
  			secondsCounter = 0;   //Reset both counters to 0 since the PrivTag is no longer lost
  			foundFlag = 0;
  			HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);   //Disconnects BLE because PrivTag is no longer lost
  			HAL_Delay(10);
  			HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  			leds_set(0b11);
  			HAL_Delay(100);
  			leds_set(0b00);
  		}*/
  			//cortexM4->SCR |= (0UL << SCB_SCR_SLEEPDEEP_Pos);
  			//__WFI(); // Might need to disable NVIC for this to work?
  		//}

  		/*
  		if (foundFlag > 500){
  			leds_set(0b11);
  		}*/
  }
}

/**
 * Handles timer update event - resets LEDs for blinking
 */
void TIM2_IRQHandler() {
	//Checks that interrupt flag is set to ensure correct interrupt
	if (TIM2->SR & TIM_SR_UIF) {
		//Reset interrupt flag
	    TIM2->SR &= ~(TIM_SR_UIF);
	    secondsCounter++;   //Increment the seconds

	    if (secondsCounter == lostTime/10){
	    	ble_init();  //initializes BLE on tag after dropped for one minute, starts advertising to begin connection process
	    }

	    if (secondsCounter >= lostTime/10){  //After tag has been "lost" for one minute, 1200/20 Hz = 60 seconds = 1 minute
	    	lostFlag = 1;
	    }
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == ACCL_INT_Pin){
		foundFlag = 1;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ACCL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
