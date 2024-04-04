/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#include "sta350bw_driver.h"
#include "testData.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STA350BW_DEV_ADDR               STA350BW_DEV_ADDR_1
#define STA350BW_HI2C_INSTANCE          &hi2c1
#define STA350BW_I2C_TIMEOUT            ((uint32_t)0xff)

#define AUDIO_BUFFER_SIZE               (8U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//uint32_t audioData[AUDIO_BUFFER_SIZE] = {0,1,2,3,4,5,6,7};
uint16_t audioBuffer[AUDIO_BUFFER_SIZE<<1] = {0};
uint32_t audioIdx = 0;
uint32_t cbCnt = 0;
char msg[50] = {0};

sta350bw_status_t sta350bw_write(
  uint8_t devAddr,
  uint8_t *pData,
  size_t nBytes);

sta350bw_status_t sta350bw_read(
  uint8_t devAddr,
  uint8_t *pData,
  size_t nBytes);

sta350bw_handle_t sta350bwH =
{
     .deviceAddr            = STA350BW_DEV_ADDR,
     .masterVolume          = -24.0f,
     .i2sCfg.sampleRate     = STA350BW_I2S_FS_48000,
     .i2sCfg.dataFirstBit   = STA350BW_I2S_MSB_FIRST,
     .i2sCfg.delayBitEn     = STA350BW_I2S_DELAY_BIT_ENABLE,
     .i2sCfg.bitsPerFrame   = STA350BW_I2S_64_BITS_PER_FRAME,
     .i2sCfg.format         = STA350BW_I2S_FMT_I2S,
     .i2sCfg.mclkSampRate   = STA350BW_I2S_MCLK_FS_X_256,
     .i2sCfg.dataBits       = STA350BW_I2S_DATA_BITS_24,
     .write                 = sta350bw_write,
     .read                  = sta350bw_read,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* i2c write that is used by the handle of the stm350bw */
sta350bw_status_t sta350bw_write(
  uint8_t devAddr,
  uint8_t *pData,
  size_t nBytes)
{
    if (HAL_I2C_Master_Transmit(STA350BW_HI2C_INSTANCE, devAddr, pData, nBytes, STA350BW_I2C_TIMEOUT) != HAL_OK)
    {
        return STA350BW_STATUS_I2C_ERROR;
    }

    return STA350BW_STATUS_OK;
}

/* i2c read that is used by the handle of the stm350bw */
sta350bw_status_t sta350bw_read(
  uint8_t devAddr,
  uint8_t *pData,
  size_t nBytes)
{
    // read from subaddress
    if (HAL_I2C_Master_Receive(STA350BW_HI2C_INSTANCE, devAddr, pData, nBytes, STA350BW_I2C_TIMEOUT) != HAL_OK)
    {
        return STA350BW_STATUS_I2C_ERROR;
    }

    return STA350BW_STATUS_OK;
}

/* starts the sysTick counter, returning current value of the counter */
uint32_t startTimer(void)
{
    uint32_t val;

    // get current value
    val = SysTick->VAL;

    // reset ms count
    uwTick = 0;

    return val;
}

/* stops the the sysTick counter and calculates the difference between the start and end time */
uint32_t endTimer(uint32_t startVal)
{
    // calculate difference
    int32_t diff = startVal - SysTick->VAL;

    // offset if the count has wrapped around
    if (diff < 0)
    {
        diff += SysTick->LOAD;
    }

    // return the total number of samples
    return (uint32_t)diff + uwTick * SysTick->LOAD;
}

/* i2s tx full callback interrupt handler */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    uint32_t i;
    uint32_t cnt = 0;

    // set up pointers to the buffer and data
	uint16_t *pAudioBuffer = audioBuffer;
	uint32_t *pAudioData = (uint32_t*)(testData + audioIdx);

    // start timer
    cnt = startTimer();

	// copy over new data to buffer
	for (i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		*pAudioBuffer = *pAudioData >> 8;
		pAudioBuffer++;

		*pAudioBuffer = *pAudioData << 8;
		pAudioBuffer++;

		pAudioData++;
	}

    // iterate the audio index
    audioIdx += AUDIO_BUFFER_SIZE;
    audioIdx %= N_SAMPLES;

    // end timer
    cnt = endTimer(cnt);
    cbCnt++;

    // print output
    if (cbCnt >= 10000)
    {
        // number of cycles per callback
        // sysClock / sampleRate * callbackLen  -->  84000000 / 48000 * 32 = 56000
        sprintf(msg, "Clk Count: %lu;\t Percentage: %.2f\n\r", cnt, (float)cnt/560.0f);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, 50, 0xFFFF);

        cbCnt = 0;
    }
}

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // set up I2S DMA transmission
  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)audioBuffer, AUDIO_BUFFER_SIZE);

  HAL_Delay(1000);

  // initialise amp
  if (sta350bw_init(&sta350bwH) != 0)
  {
	  return -1;
  }

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
