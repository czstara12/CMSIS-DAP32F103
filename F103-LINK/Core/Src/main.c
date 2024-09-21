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
#include "RTE_Components.h"
#ifdef RTE_Compiler_EventRecorder
	#include "EventRecorder.h"
#endif
#include "cmsis_os2.h" 
#include "rl_usb.h"
#include "osObjects.h"
#include "DAP_config.h"
#include "DAP.h"

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};

// Statically change USB Device 0 Device Descriptor
const uint8_t usbd0_device_descriptor[] = { 
  18U,         // bLength = 7 bytes
  1U,          // bDescriptorType = 1 = Device Descriptor Type
  0x00U,0x02U,       // bcdUSB = 2.00 (little-endian)
  0X00U,          // bDeviceClass = 0 = Defined in interface
  0U,          // bDeviceSubClass = 0 = Defined in interface
  0U,          // bDeviceProtocol = 0 = Defined in interface
  8U,         // bMaxPacketSize = 8
  0x09U,0x12U, // idVendor = 0x0D28 (little-endian)
  0x01U,0x00U, // idProduct = 0x0204 (little-endian)
  0U,1U,       // bcdDevice = 1.00 (little-endian)
  1U,          // iManufacturer = 0 = No Manufacturer string
  2U,          // iProduct = 0 = No Product string
  3U,          // iSerialNumber = 0 = No Serial Number string
  1U           // bNumConfigurations = 1 = 1 configuration
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Override default HAL_GetTick function */
uint32_t HAL_GetTick (void) { 
  static uint32_t ticks = 0U; 
  uint32_t i; 
  
  if (osKernelGetState () == osKernelRunning) { 
    return ((uint32_t)osKernelGetTickCount ()); 
  }
  
  /* If Kernel is not running wait approximately 1 ms then increment and return auxiliary tick counter value */ 
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) { 
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
  } 
  
  return ++ticks; 
} 

/* Override default HAL_InitTick function */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) { 
  UNUSED(TickPriority); 
  
  return HAL_OK; 
} 

// Application Main program
__NO_RETURN void app_main (void *argument) {
  (void)argument;

  DAP_Setup();                          // DAP Setup 

  USBD_Initialize(0U);                  // USB Device Initialization
  USBD_Connect(0U);                     // USB Device Connect

  while (!USBD_Configured(0U))
		osDelay(10);         // Wait for USB Device to configure

  LED_CONNECTED_OUT(1U);                // Turn on  Debugger Connected LED
  LED_RUNNING_OUT(1U);                  // Turn on  Target Running LED
  Delayms(500U);                        // Wait for 500ms
  LED_RUNNING_OUT(0U);                  // Turn off Target Running LED
  LED_CONNECTED_OUT(0U);                // Turn off Debugger Connected LED

  // Create DAP Thread
  DAP_ThreadId = osThreadNew(DAP_Thread, NULL, &DAP_ThreadAttr);

  // Create SWO Thread
//  SWO_ThreadId = osThreadNew(SWO_Thread, NULL, &SWO_ThreadAttr);

	osThreadExit();
//  osDelay(osWaitForever);
//  for (;;) {
//	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//		osDelay(200);
//	}
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	#ifdef RTE_Compiler_EventRecorder
		EventRecorderInitialize(EventRecordAll, 1);
	#endif  
	SystemCoreClockUpdate();
	osKernelInitialize();                 // Initialize CMSIS-RTOS
	osThreadNew(app_main, NULL, NULL);    // Create application main thread
  if (osKernelGetState() == osKernelReady) {
    osKernelStart();                    // Start thread execution
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nRESET_Pin|SWCLK_Pin|SWDIO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nRESET_Pin */
  GPIO_InitStruct.Pin = nRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SWCLK_Pin SWDIO_Pin */
  GPIO_InitStruct.Pin = SWCLK_Pin|SWDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
