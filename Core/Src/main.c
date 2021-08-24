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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__attribute__ ((section(".New_Firmware"))) char NewFirmware[5000] = {0};
__attribute__ ((section(".Vector_TB"))) char vector[1024] = {0};
//char NewFirmware[5000] = {0};
int size = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/*__attribute__ ((section(".code_in_ram"))) void My_UART_Init();
__attribute__ ((section(".code_in_ram"))) void My_DMA_Init();
__attribute__ ((section(".code_in_ram"))) void My_Send_Char(uint8_t data);
__attribute__ ((section(".code_in_ram"))) void My_Send_String(char* str);*/
void My_UART_Init();
void My_DMA_Init();
void My_Send_Char(uint8_t data);
void My_Send_String(char* str);
__attribute__ ((section(".code_in_ram"))) void UpdateFw_Func();
__attribute__ ((section(".code_in_ram"))) void Sys_Tick_inRam();
__attribute__ ((section(".code_in_ram"))) void FlashErase(uint32_t address, uint8_t numSector);
__attribute__ ((section(".code_in_ram"))) void FlashWrite(uint16_t* address, uint16_t data2Write);
__attribute__ ((section(".code_in_ram "))) void Custom_Software_Reset();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  My_UART_Init();
  My_DMA_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // receive new firmware using uart
	  if(size == 0)
	  {
		  //My_UART_Init();
		  //My_DMA_Init();
		 int sizeStart = DMA1_Channel5->CNDTR;
		  My_Send_String("Please send new firmware in 15s!");
		  HAL_Delay(15000);
		  size = sizeStart - DMA1_Channel5->CNDTR ;
		  UpdateFw_Func();

	  }


  }
  //Custom_Software_Reset();

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/**
  * @brief Uart1  Initialization Function with DMA
  * @param None
  * @retval None
  */
void My_UART_Init()
{
	__HAL_RCC_USART1_CLK_ENABLE();
	// GPIO Rx-> PA10-> INput floating, Tx PA9->outpush->push pull
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIOA->CRH = (GPIOA->CRH & 0xFFFFFF0F) | (11 << 4);
	GPIOA->CRH = (GPIOA->CRH & 0xFFFFF0FF)| (4 << 8);
	// set init uart
	USART1->CR1 = 0;	// watch manual: set data length (reset value is 0: 8 bit data)
	USART1->CR1 = USART1->CR1 | (1<<13);//USART1->CR1
	USART1->BRR = (52<<4)|1;//the desired baud rate: 9600
	USART1->CR1 = USART1->CR1 |(1 << 3)| (1 << 2);//Transmitter enable,  Receiver enable
	//can or not set Parity control enable
	USART1->CR3 |= (1 << 6); //: DMA enable receiver, channel 5 for  USART1_RX muốn khi nào nhận thì enable cái này. nên có thể bỏ ở ngoài

}
/**
  * @brief DMA1 channel 5  Initialization Function to transfer data from uart to ram
  * @param None
  * @retval None
  */
void My_DMA_Init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	//DMA1_Channel5->CPAR = 0x40013804; // channel 5 for  USART1_RX
	DMA1_Channel5->CPAR = &(USART1->DR);
	DMA1_Channel5->CMAR = NewFirmware;
	DMA1_Channel5->CNDTR = 0xffffffff;
	DMA1_Channel5->CCR = DMA1_Channel5->CCR | (1 << 7); //| ( 1<< 5); //// Memory& Peripheral   increment mode, Circular mode
	DMA1_Channel5->CCR = DMA1_Channel5->CCR | 1;//Channel enable
}

/**
  * @brief Sent a string by uart
  * @param pointer char
  * @retval None
  */
void My_Send_String(char* str)
{
	while(*str)
	{
		while(((USART1->SR >> 7)&1) !=1);			//wait TXE (Transmit data register empty)
		USART1->DR = *(str++);
		while(((USART1->SR>>6)&1) !=1);			//wait TC (Transmission complete)
		USART1->SR &= ~(uint32_t)(1<<6);
		//My_Send_Char(str[i]);
	}
}
/**
  * @brief trasfer VECTOR TABLE TO RAM, erase flash and write new firmware in to flash
  * @param None
  * @retval None
  */
void UpdateFw_Func()
{
	memcpy(vector, 0x08000000, 1024);
	uint32_t* VECTOR = 0xE000ED08;
	*VECTOR = vector;
	uint32_t* Sys_Tick = 0x2000003C;
	*Sys_Tick = (uint32_t)Sys_Tick_inRam;
	//Erase
	FlashErase((uint32_t)0x08000000, 15);
	//Write
	uint16_t* add = (uint16_t*)0x08000000;
	//size = 4576
	for(int i = 0; i < size ;i = i + 2)
	{
		FlashWrite(add + i/2, (NewFirmware[i + 1] << 8)|NewFirmware[i]);
	}
	Custom_Software_Reset();
}
/**
  * @brief system tick interupt
  * @param None
  * @retval None
  */
void Sys_Tick_inRam()
{
	int a =0;
}
/**
  * @brief reset
  * @param None
  * @retval None
  */
void Custom_Software_Reset()
{
	uint32_t* AIRCR = (uint32_t*)(0xE000ED0C);
	/*
	* To write to this register, you must write 0x5FA to the VECTKEY field,
	* otherwise the processor ignores the write.
	*/
	*AIRCR = ( 0x5FA <<16 ) | (1<<2) ;
}
/**
  * @brief Erase a number of pages flash memory
  * @param address: address of first page need erase
  * @param numSector: number of pages need erase
  * @retval None
  */
void FlashErase(uint32_t address, uint8_t numSector)
{
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
	for(int i =0; i < numSector ; i++)
	{
		FLASH->CR |= (1<<1);
		FLASH->AR = address + i*0x400;
		FLASH->CR |= (1<<6);
		while(((FLASH->SR)&(0x1))!=0);
		//FLASH->SR|=(1<<5);
	}

}
/**
  * @brief Write a haft-word to flash
  * @param address: address need write
  * @param data2Write: data write
  * @retval None
  */
void FlashWrite(uint16_t* address, uint16_t data2Write)
{
			FLASH->KEYR = 0x45670123;
			FLASH->KEYR = 0xCDEF89AB;
			while(((FLASH->SR)&(0x1))!=0);
			FLASH->CR = 0;
			FLASH->CR |= 1;
			uint16_t *Write_Data = address;
			*Write_Data = (uint16_t)data2Write;
			while(((FLASH->SR)&(0x1))!=0);
			FLASH->SR|=(1<<5);
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
