/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "MY_LIS3DSH.h"
#include "gesture.h"
#include "gesture_data.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define KAL_ERROR_ESTIMATED			((float) 9800)
#define KAL_ERROR_MEASUREMENT			((float) 9800)
#define KAL_NOISE_PROCESS			((float) 10)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LIS3DSH_DataScaled myData;
LIS3DSH_DataRaw myDataRaw;
uint8_t drdyFlag=0;
int8_t sendData[8];
char text[23];

uint8_t MyString[13] = "Hello World\r\n";



float kalman_gain_x,err_est_x,err_mea_x, current_estimate_x,last_estimate_x, proc_noise_x;
float err_est_x = KAL_ERROR_ESTIMATED; float err_mea_x = KAL_ERROR_MEASUREMENT; float proc_noise_x = KAL_NOISE_PROCESS;

/* mea - data read from sensor
 * err_est, err_mea, proc_noise is set by user, mN/s^2 */


float KalmanFilter_X(float mea_x)
{
	kalman_gain_x = err_est_x/(err_est_x + err_mea_x) ;
	current_estimate_x = last_estimate_x + kalman_gain_x*(mea_x-last_estimate_x);
	err_est_x = (1.0-kalman_gain_x)*err_est_x + fabs(last_estimate_x-current_estimate_x)*proc_noise_x;
	last_estimate_x = current_estimate_x;
	return current_estimate_x;
}


float kalman_gain_y,err_est_y,err_mea_y, current_estimate_y,last_estimate_y, proc_noise_y;
float err_est_y = KAL_ERROR_ESTIMATED; float err_mea_y = KAL_ERROR_MEASUREMENT; float proc_noise_y = KAL_NOISE_PROCESS;

float KalmanFilter_Y(float mea_y)
{
	kalman_gain_y = err_est_y/(err_est_y + err_mea_y) ;
	current_estimate_y = last_estimate_y + kalman_gain_y*(mea_y-last_estimate_y);
	err_est_y = (1.0-kalman_gain_y)*err_est_y + fabs(last_estimate_y-current_estimate_y)*proc_noise_y;
	last_estimate_y = current_estimate_y;
	return current_estimate_y;
}

float kalman_gain_z,err_est_z,err_mea_z, current_estimate_z,last_estimate_z, proc_noise_z;
float err_est_z = KAL_ERROR_ESTIMATED; float err_mea_z = KAL_ERROR_MEASUREMENT; float proc_noise_z = KAL_NOISE_PROCESS;

float KalmanFilter_Z(float mea_z)
{
	kalman_gain_z = err_est_z/(err_est_z + err_mea_z) ;
	current_estimate_z = last_estimate_z + kalman_gain_z*(mea_z-last_estimate_z);
	err_est_z = (1.0-kalman_gain_z)*err_est_z + fabs(last_estimate_z-current_estimate_z)*proc_noise_z;
	last_estimate_z = current_estimate_z;
	return current_estimate_z;
}



float kal_a_z, kal_a_x, kal_a_y;

float out_data0;




float activations[AI_GESTURE_DATA_ACTIVATIONS_SIZE];
float in_data[AI_GESTURE_IN_1_SIZE_BYTES];
float out_data[AI_GESTURE_OUT_1_SIZE_BYTES];

ai_buffer *ai_input;
ai_buffer *ai_output;

ai_handle network = AI_HANDLE_NULL;
ai_error err;
ai_network_report report;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	LIS3DSH_InitTypeDef myAccConfigDef;
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
	myAccConfigDef.dataRate = LIS3DSH_DATARATE_25;
	myAccConfigDef.fullScale = LIS3DSH_FULLSCALE_4;
	myAccConfigDef.antiAliasingBW = LIS3DSH_FILTER_BW_50;
	myAccConfigDef.enableAxes = LIS3DSH_XYZ_ENABLE;
	myAccConfigDef.interruptEnable = true;
	LIS3DSH_Init(&hspi1, &myAccConfigDef);

	LIS3DSH_X_calibrate(-1000.0, 980.0);
	LIS3DSH_Y_calibrate(-1020.0, 1040.0);
	LIS3DSH_Z_calibrate(-920.0, 1040.0);

	//Yapay Sinir Ağı



	  /** @brief Initialize network */
	    const ai_handle acts[] = { activations };
	    err = ai_gesture_create_and_init(&network, acts, NULL);
	    if (err.type != AI_ERROR_NONE) {
	        printf("ai init_and_create error\n");
	        return -1;
	    }

	    /** @brief {optional} for debug/log purpose */
	    if (ai_gesture_get_report(network, &report) != true) {
	        printf("ai get report error\n");
	        return -1;
	    }

	    printf("Model name      : %s\n", report.model_name);
	    printf("Model signature : %s\n", report.model_signature);

	    ai_input = &report.inputs[0];
	    ai_output = &report.outputs[0];
	    printf("input[0] : (%d, %d, %d)\n", AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_HEIGHT),
	                                        AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_WIDTH),
	                                        AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_CHANNEL));
	    printf("output[0] : (%d, %d, %d)\n", AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_HEIGHT),
	                                         AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_WIDTH),
	                                         AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_CHANNEL));

	    /** @brief Fill input buffer with random values */
	    srand(1);
	    for (int i = 0; i < AI_GESTURE_IN_1_SIZE; i++) {
	        in_data[i] = rand() % 0xFFFF;
	    }

	    /** @brief Normalize, convert and/or quantize inputs if necessary... */

	    /** @brief Perform inference */
	    ai_i32 n_batch;

	    /** @brief Create the AI buffer IO handlers
	     *  @note  ai_inuput/ai_output are already initilaized after the
	     *         ai_network_get_report() call. This is just here to illustrate
	     *         the case where get_report() is not called.
	     */
	    ai_input = ai_gesture_inputs_get(network, NULL);
	    ai_output = ai_gesture_outputs_get(network, NULL);

	    /** @brief Set input/output buffer addresses */
	    ai_input[0].data = AI_HANDLE_PTR(in_data);
	    ai_output[0].data = AI_HANDLE_PTR(out_data);
int firstInData = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(drdyFlag == 1)
		{
			drdyFlag = 0;
			myData = LIS3DSH_GetDataScaled();
			myDataRaw = LIS3DSH_GetDataRaw();

			/*
	  			sendData[0] = myDataRaw.x & 0xff;
	  			sendData[1] = (myDataRaw.x >>8) & 0xff;
	  			sendData[2] = 5;
	  			sendData[3] = 5;
	  			sendData[4] = 5;
	  			sendData[5] = 5;
	  			sendData[6] = 5;
	  			sendData[7] = 5;

			 */
			kal_a_x=KalmanFilter_X(myDataRaw.x);
			kal_a_y=KalmanFilter_Y(myDataRaw.y);
			kal_a_z=KalmanFilter_Z(myDataRaw.z);
			//kalmansız veriler
			//sprintf(text,"x%06dy%06dz%06d\r\n",myDataRaw.x,myDataRaw.y,myDataRaw.z);
			//kalman filtreli veri
			sprintf(text,"x%06dy%06dz%06d\r\n",kal_a_x,kal_a_y,kal_a_z);
			HAL_UART_Transmit(&huart3,text, 23, 10);


			if(firstInData< AI_GESTURE_IN_1_SIZE_BYTES)
			{
				in_data[firstInData] = (kal_a_x+10000)/20000;
				in_data[firstInData+1] = (kal_a_y+10000)/20000;
				in_data[firstInData+2] = (kal_a_z+10000)/20000;
				firstInData = firstInData+3;
			}

			else{

				for(int i=0; i<AI_GESTURE_IN_1_SIZE_BYTES-3;i++)
				{

					in_data[i] = in_data[i+3];

				}

			in_data[AI_GESTURE_IN_1_SIZE_BYTES-3] = (kal_a_x+10000)/20000;
			in_data[AI_GESTURE_IN_1_SIZE_BYTES-2] = (kal_a_y+10000)/20000;
			in_data[AI_GESTURE_IN_1_SIZE_BYTES-1] = (kal_a_z+10000)/20000;



		    /** @brief Perform the inference */

		    n_batch = ai_gesture_run(network, &ai_input[0], &ai_output[0]);
		    if (n_batch != 1) {
		        err = ai_gesture_get_error(network);
		        printf("ai run error %d, %d\n", err.type, err.code);
		      return -1;
		    }


		    /** @brief Post-process the output results/predictions */
		    printf("Inference output..\n");
		    for (int i = 0; i < AI_GESTURE_OUT_1_SIZE; i++) {
		        printf("%d,", out_data[i]);

		    }





			}


		}

if(out_data[0]>0.9)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_SET);
}
else
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_RESET);
}
if(out_data[1]>0.9)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_SET);
}
else
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_RESET);
}
if(out_data[2]>0.9)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET);
}
else
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_RESET);
}


//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : MEMS_CS_Pin */
  GPIO_InitStruct.Pin = MEMS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MEMS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
	/* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
	 */

	drdyFlag = 1;
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
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

