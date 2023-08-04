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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ejection.h"
#include "string.h"
#include "stdio.h" // sprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	uint32_t header; // 4 bytes so accesses are aligned
	float accX;
	float accZ;
	float altitude;
	uint32_t trailer;
} telemetry_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_BUFFER_LENGTH 			20	// bytes
#define RX_TELEMETRY_LENGTH			20	// bytes: [header][AccX][AccZ][Altitude][trailer], 4 x 5 = 20 bytes total
#define CONTROL_RESPONSE_TEMPLATE	((const char *)"C,%d,E\r\n") // use for sprintf
#define CONTROL_RESPONSE_LENGTH		7

#define RX_TELEMETRY_HEADER			((uint32_t) 0xFFFF0000)
#define RX_TELEMETRY_TRAILER		((uint32_t) 0x0000FFFF)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// UART DMA buffers
uint8_t uart3_rx_dma_buffer[DMA_BUFFER_LENGTH];
uint8_t uart3_tx_buffer[DMA_BUFFER_LENGTH];

// flags
volatile uint8_t rx_telemetry_flag = 0;
volatile uint8_t button_pressed_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void config_timer_freq(TIM_TypeDef *TIMx, uint32_t freq);
void parse_telemetry(uint8_t *buffer, struct ej_data *data);
void send_flight_state(flight_state_t *fs);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void config_timer_freq(TIM_TypeDef *TIMx, uint32_t freq) {
	uint32_t psc = 0;
	if (freq < 50000) {
		psc = 100 - 1;
	}

	TIMx->PSC = psc;
	TIMx->ARR = (uint16_t) (45000000 / ((psc + 1) * freq) - 1);
	TIMx->EGR |= 0x1; // force update
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart3) {
		// check header and trailer
		uint32_t *buffer = (uint32_t *)(uart3_rx_dma_buffer);
		if (buffer[0] != RX_TELEMETRY_HEADER || buffer[RX_TELEMETRY_LENGTH / sizeof(buffer[0]) - 1] != RX_TELEMETRY_TRAILER) {
			// ignore this message since it is not in the right format, clear the buffer
			memset(uart3_rx_dma_buffer, 0, RX_TELEMETRY_LENGTH);
		}
		else {
			// message is good, let main loop parse it and pass to ejection algorithm
			rx_telemetry_flag = 1;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_Btn_Pin) {
		button_pressed_flag = 1;
	}
}

void parse_telemetry(uint8_t *buffer, struct ej_data *data) {
	// expected format: ['S'][AccX][AccZ][Altitude]['E'], 2 x char and 3 x floats
	// convert bytes to float by casting buffer to union telemetry_t
	telemetry_t *t = (telemetry_t *)(buffer);
	data->accX = t->accX;
	data->accZ = t->accZ;
	data->altitude = t->altitude;
	// add more checks if desired
}

void send_flight_state(flight_state_t *fs) {
	memset(uart3_tx_buffer, 0, DMA_BUFFER_LENGTH);
	sprintf(uart3_tx_buffer, CONTROL_RESPONSE_TEMPLATE, *fs);
	HAL_UART_Transmit(&huart3, uart3_tx_buffer, CONTROL_RESPONSE_LENGTH, 10);
}

void send_launch_status() {
	HAL_UART_Transmit(&huart3, "launch\n", CONTROL_RESPONSE_LENGTH, 10);
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
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // reset all debug GPIOs (ARD_D0-D7)
  HAL_GPIO_WritePin(ARD_D0_GPIO_Port, ARD_D0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ARD_D1_GPIO_Port, ARD_D1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ARD_D2_GPIO_Port, ARD_D2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ARD_D3_GPIO_Port, ARD_D3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ARD_D6_GPIO_Port, ARD_D6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, GPIO_PIN_RESET);

  // ejection init
  config_timer_freq(TIM2, EJ_ALGO_UPDATE_RATE_HZ);
  ej_init();

  // start timers and DMA
  HAL_UART_Receive_DMA(&huart3, uart3_rx_dma_buffer, RX_TELEMETRY_LENGTH);
//  HAL_TIM_Base_Start_IT(&htim2);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  flight_state_t fs;
	  if (button_pressed_flag) { // use button press to "launch"
		  ej_get_flight_state(&fs);
		  if (fs == PAD) {
			  send_launch_status();
			  is_launch_command_received = 1;
		  }
		  button_pressed_flag = 0;
	  }

	  if (rx_telemetry_flag) {
		  HAL_GPIO_WritePin(ARD_D0_GPIO_Port, ARD_D0_Pin, GPIO_PIN_SET); // entire block takes 650 us to run. limited by uart probably

		  struct ej_data datapoint;
		  parse_telemetry(uart3_rx_dma_buffer, &datapoint);

		  HAL_GPIO_WritePin(ARD_D1_GPIO_Port, ARD_D1_Pin, GPIO_PIN_SET);
		  ej_update_flight_state(&datapoint, &fs); // takes 25 us to run
		  HAL_GPIO_WritePin(ARD_D1_GPIO_Port, ARD_D1_Pin, GPIO_PIN_RESET);

		  send_flight_state(&fs);
		  rx_telemetry_flag = 0;
		  HAL_UART_Receive_DMA(&huart3, uart3_rx_dma_buffer, RX_TELEMETRY_LENGTH);

		  HAL_GPIO_WritePin(ARD_D0_GPIO_Port, ARD_D0_Pin, GPIO_PIN_RESET);
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
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
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

