/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "MLX90640_API.h"
#include "string.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS16HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air

static uint16_t eeMLX90640[832];
static float mlx90640To[768];
uint16_t frame[834];
float emissivity=0.95;
int status;
float target[24][32];
int position_x;
int position_y;
int uart_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void coordinates();
void reverse();
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
 int ir=0,ic=0;
 int flag=0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);
  	MLX90640_SetChessMode(MLX90640_ADDR);
  	paramsMLX90640 mlx90640;
    status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
    if (status != 0) printf("\r\nload system parameters error with code:%d\r\n",status);
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0) printf("\r\nParameter extraction failed with error code:%d\r\n",status);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int status = MLX90640_GetFrameData(MLX90640_ADDR, frame);
	  		if (status < 0)
	  		{
	  			//printf("GetFrame Error: %d\r\n",status);
	  		}
	  		float vdd = MLX90640_GetVdd(frame, &mlx90640);
	  		float Ta = MLX90640_GetTa(frame, &mlx90640);

	  		float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
	  		//printf("vdd:  %f Tr: %f\r\n",vdd,tr);
	  		MLX90640_CalculateTo(frame, &mlx90640, emissivity , tr, mlx90640To);
	  		char buffer[]="\r\n==========================CAMERA IS INTERFACED==========================\r\n";
       HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);
       memset(buffer,0,sizeof(buffer));
	  		printf("\r\n==========================START OF IMAGE PIXEL PRINTING==========================\r\n");
	  		HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
	  		HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
	  		HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
		int imlx = 0;
		float data_ip;
		for (ir = 0; ir < 24; ir++) {
			for (ic = 0; ic < 32; ic++) {
				data_ip = mlx90640To[imlx++] - 31;
				if (data_ip > 10)

				{
					target[ir][ic] = data_ip;
				} else
					target[ir][ic] = 0;

			}
		}

		char d_buff[100];
		for (ir = 0; ir < 24; ir++) {
			for (ic = 0; ic < 32; ic++) {

				sprintf(d_buff, "%0.0f ", target[ir][ic]);
				HAL_UART_Transmit(&huart2, d_buff, strlen(d_buff), 100);
				memset(d_buff, 0, sizeof(d_buff));
			}
			HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
		}
		HAL_UART_Transmit(&huart2, "\r\n", 2, 100);


		float max = target[0][0];
		for (ir = 0; ir < 24; ir++) {
			for (ic = 0; ic < 32; ic++) {

				if (max < target[ir][ic]) {
					max = target[ir][ic];
					position_y = ir;
					position_x = ic;
				}

			}
			HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
		}
		//HAL_UART_Receive_IT(&huart2, &uart_data, 1);

		if (max != 0)
			{
				flag = 1;
			}
			if (flag == 1) {
				flag = 0;
				sprintf(d_buff, "%0.0f row=%d column=%d", max, position_y + 1,
						position_x + 1);

				HAL_UART_Transmit(&huart2, d_buff, strlen(d_buff), 100);
				coordinates();
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);

				//HAL_Delay(1000);
				reverse();
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);

			}
			HAL_Delay(3000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Motor_P1_Pin|Motor_P2_Pin|Motor1_P1_Pin|Motor1_P2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Motor_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_P1_Pin Motor_P2_Pin Motor1_P1_Pin Motor1_P2_Pin */
  GPIO_InitStruct.Pin = Motor_P1_Pin|Motor_P2_Pin|Motor1_P1_Pin|Motor1_P2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Motor_EN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Motor_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void coordinates()
{
	for(int i=0;i<position_y;i++)
	{
	HAL_GPIO_WritePin(GPIOC, Motor_P1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, Motor_P2_Pin, GPIO_PIN_RESET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOC, Motor_P1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Motor_P2_Pin, GPIO_PIN_RESET);
	HAL_Delay(300);
	}

	for(int j=0;j<position_x;j++)
		{
		HAL_GPIO_WritePin(GPIOC, Motor1_P1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Motor1_P2_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOC, Motor1_P1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Motor1_P2_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		}
}

void reverse()
{
	for(int i=position_y;i>0;i--)
	{
	HAL_GPIO_WritePin(GPIOC, Motor_P1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Motor_P2_Pin, GPIO_PIN_SET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOC, Motor_P1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Motor_P2_Pin, GPIO_PIN_RESET);
	HAL_Delay(300);
	}

	for(int j=position_x;j>0;j--)
		{
		HAL_GPIO_WritePin(GPIOC, Motor1_P1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Motor1_P2_Pin, GPIO_PIN_SET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOC, Motor1_P1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Motor1_P2_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		}
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
