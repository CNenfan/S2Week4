/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct
{
  int16_t rocker_l_;   // 左水平
  int16_t rocker_l1;   // 左竖直
  int16_t rocker_r_;   // 右水平
  int16_t rocker_r1;   // 右竖直

  int16_t SWA;         // 开关A
  int16_t SWB;         // 开关B
  int16_t SWC;         // 开关C
  int16_t SWD;         // 开关D

  int16_t VRA;         // 旋钮A
  int16_t VRB;         // 旋钮B

  uint8_t error_protect; // 故障保护激活位
  uint8_t byte_lose;     // 真丢失标志位
} RC_ctrl_t;

static RC_ctrl_t rc_data;       // 遥控器数据,初始化时返回

uint8_t buf[256];
int16_t CH[16];
uint8_t start_byte;
uint8_t flag_byte;
uint8_t stop_byte;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart == &huart3)
  {
    start_byte = buf[0];

    CH[0]  = (((int16_t)buf[1]  >> 0 | ((int16_t)buf[2]  << 8))  & 0x07FF) - 1024;
    CH[1]  = (((int16_t)buf[2]  >> 3 | ((int16_t)buf[3]  << 5))  & 0x07FF) - 1024;
    CH[2]  = (((int16_t)buf[3]  >> 6 | ((int16_t)buf[4]  << 2) | ((int16_t)buf[5] << 10)) & 0x07FF) - 1024;
    CH[3]  = (((int16_t)buf[5]  >> 1 | ((int16_t)buf[6]  << 7))  & 0x07FF) - 1024;
    CH[4]  = (((int16_t)buf[6]  >> 4 | ((int16_t)buf[7]  << 4))  & 0x07FF) - 1024;
    CH[5]  = (((int16_t)buf[7]  >> 7 | ((int16_t)buf[8]  << 1) | ((int16_t)buf[9] << 9))  & 0x07FF) - 1024;
    CH[6]  = (((int16_t)buf[9]  >> 2 | ((int16_t)buf[10] << 6))  & 0x07FF) - 1024;
    CH[7]  = (((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3))  & 0x07FF) - 1024;
    CH[8]  = (((int16_t)buf[12] >> 0 | ((int16_t)buf[13] << 8))  & 0x07FF) - 1024;
    CH[9]  = (((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5))  & 0x07FF) - 1024;
    CH[10] = (((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2) | ((int16_t)buf[16] << 10)) & 0x07FF) - 1024;
    CH[11] = (((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7))  & 0x07FF) - 1024;
    CH[12] = (((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4))  & 0x07FF) - 1024;
    CH[13] = (((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1) | ((int16_t)buf[20] << 9))  & 0x07FF) - 1024;
    CH[14] = (((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6))  & 0x07FF) - 1024;
    CH[15] = (((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3))  & 0x07FF) - 1024;

  flag_byte = buf[23];  // 标志位

    flag_byte = buf[23];
    stop_byte = buf[24];

    rc_data.rocker_r_     = CH[0];                           // 右水平
    rc_data.rocker_r1     = CH[1];                           // 右竖直
    rc_data.rocker_l1     = CH[2];                           // 左竖直
    rc_data.rocker_l_     = CH[3];                           // 左水平
    rc_data.SWA           = CH[4];                           // 开关A
    rc_data.SWB           = CH[5];                           // 开关B
    rc_data.SWC           = CH[6];                           // 开关C
    rc_data.SWD           = CH[7];                           // 开关D
    rc_data.VRA           = CH[8];                           // 旋钮A
    rc_data.VRB           = CH[9];                           // 旋钮B
    rc_data.error_protect = (flag_byte >> 3) & 0x01;         // 故障保护激活位
    rc_data.byte_lose     = (flag_byte >> 2) & 0x01;         // 真丢失标志位

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, 25);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart3)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, 25);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, 25); // Start UART reception in DMA mode with idle line detection
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);  // Disable half-transfer interrupt
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
