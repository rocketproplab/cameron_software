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
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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


void SPI1_send(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	// With the pin being put to "GPIO_PIN_RESET" or 0, the chip is selected
	// to be written to.
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);


	HAL_SPI_Transmit(&hspi1, (uint8_t*)(pData), Size, Timeout);
	// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

	// Sets the CS pin to high, which deselects it
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void SPI1_receive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	// With the pin being put to "GPIO_PIN_RESET" or 0, the chip is selected
	// to be written to.
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);


	HAL_SPI_Receive(&hspi1, pData, Size, Timeout);

	// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

	// Sets the CS pin to high, which deselects it
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

}

void SPI1_send_receive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
	// With the pin being put to "GPIO_PIN_RESET" or 0, the chip is selected
	// to be written to.
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);


	HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, Size, Timeout);

	// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

	// Sets the CS pin to high, which deselects it
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

}

void SPI1_send_XBee(uint8_t frame_Type, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	// With the pin being put to "GPIO_PIN_RESET" or 0, the chip is selected
	// to be written to.
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);


	HAL_SPI_Transmit(&hspi1, (uint8_t*)(pData), Size, Timeout);
	uint8_t  packetHeader[4];
	uint8_t checkSum_byte;


	// Calculates the checksum of the data sent
	for(uint16_t i = 0; i < Size; i++)
	{
		checkSum_byte += pData[i];
	}
	checkSum_byte = 0xFF - checkSum_byte;


	packetHeader[0] = 0x7E; 					  // Starting delimiter of packet
	packetHeader[1] = (((Size + 1) >> 8) & 0xFF); // Honestly don't know what these do, but its the length of MSB
	packetHeader[2] = ((Size + 1) & 0xFF); 		  // Honestly don't know what these do, but its the length of LSB
	packetHeader[3] = frame_Type;			 	  // API frame type


	// Write the header of the SPI packet
	HAL_SPI_Transmit(&hspi1, (uint8_t*)(packetHeader), Size, Timeout);
		// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);


	// Writes the actual data of the SPI packet
	HAL_SPI_Transmit(&hspi1, (uint8_t*)(pData), Size, Timeout);
	// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);


	// Writes the checksum byte of the SPI packet
	HAL_SPI_Transmit(&hspi1, (uint8_t*)(&checkSum_byte), Size, Timeout);
	// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);


	// Sets the CS pin to high, which deselects it
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}


void SPI1_send_XBee_Packet(uint8_t frame_Type, uint64_t xbee_Dest_Addr, uint8_t *pData, uint16_t data_Size, uint32_t Timeout)
{
	// With the pin being put to "GPIO_PIN_RESET" or 0, the chip is selected
	// to be written to.
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);


//	HAL_SPI_Transmit(&hspi1, (uint8_t*)(pData), Size, Timeout);

	const uint8_t PACKET_HEADER_SIZE = 17;

	uint8_t total_packet_size;
	uint8_t packetHeader[PACKET_HEADER_SIZE];
	uint8_t checkSum_byte;
	const uint8_t frameID = 0x01;
	const uint16_t xbee_Dest_Addr_16bit = 0xFFFE; // Send to 0xFFFE for unknown 16 bit addr
	const uint8_t broadcast_radius = 0x00;
	const uint8_t options = 0x00;

//	uint8_t * api_Packet;

	/*api_Packet = (uint8_t *) malloc(4 * sizeof(uint8_t) + sizeof(checkSum_byte) +
									sizeof(frameID) + sizeof(xbee_Dest_Addr_16bit) +
									sizeof(max_hops) + sizeof(options) + Size);*/

	total_packet_size = data_Size + PACKET_HEADER_SIZE; // Adds the API header size to the size of just the data to transmit


	// TODO change data size to reflect the MSB and LSB of the packet data between length and checksum


	packetHeader[0] = 0x7E; 					  		// Starting delimiter of packet for XBee API
	packetHeader[1] = (((data_Size + 1) >> 8) & 0xFF);  // Honestly don't know what these do, but its the length of MSB
	packetHeader[2] = ((data_Size + 1) & 0xFF); 		// Honestly don't know what these do, but its the length of LSB
	packetHeader[3] = frame_Type;			 	  		// API frame type

	packetHeader[4] = frameID;

	for(uint8_t i = 0; i < 8; i++)
	{
		// Converts 64 bit integer to 8, 8 bit integers
		packetHeader[i + 5] =  *((uint8_t *) (&xbee_Dest_Addr) + sizeof(uint8_t) * i);
	}

	packetHeader[13] = *((uint8_t *) (& xbee_Dest_Addr_16bit));
	packetHeader[14] = *((uint8_t *) (& xbee_Dest_Addr_16bit) + sizeof(uint8_t));
	packetHeader[15] = broadcast_radius;
	packetHeader[16] = options;

	// Calculates the checksum of the data sent
	for(uint16_t i = 3; i < (total_packet_size - 1); i++)
	{
		checkSum_byte += pData[i];
	}
	checkSum_byte = 0xFF - checkSum_byte;

	// Write the header of the SPI packet
	HAL_SPI_Transmit(&hspi1, (uint8_t*)(packetHeader), PACKET_HEADER_SIZE, Timeout);
		// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);


	// Writes the actual data of the SPI packet
	HAL_SPI_Transmit(&hspi1, (uint8_t*)(pData), data_Size, Timeout);
	// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);


	// Writes the checksum byte of the SPI packet
	HAL_SPI_Transmit(&hspi1, (uint8_t*)(&checkSum_byte), 1, Timeout);
	// Waits until the SPI bus is not busy
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);


	// Sets the CS pin to high, which deselects it
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

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
