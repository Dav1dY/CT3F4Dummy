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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Public.h"
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
IWDG_HandleTypeDef hiwdg;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for QueuePushTask */
osThreadId_t QueuePushTaskHandle;
const osThreadAttr_t QueuePushTask_attributes = {
  .name = "QueuePushTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CMD_RespTask */
osThreadId_t CMD_RespTaskHandle;
const osThreadAttr_t CMD_RespTask_attributes = {
  .name = "CMD_RespTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for IWDG_FeedTask */
osThreadId_t IWDG_FeedTaskHandle;
const osThreadAttr_t IWDG_FeedTask_attributes = {
  .name = "IWDG_FeedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RECV_Queue */
osMessageQueueId_t RECV_QueueHandle;
const osMessageQueueAttr_t RECV_Queue_attributes = {
  .name = "RECV_Queue"
};
/* Definitions for RESP_Queue */
osMessageQueueId_t RESP_QueueHandle;
const osMessageQueueAttr_t RESP_Queue_attributes = {
  .name = "RESP_Queue"
};
/* Definitions for DataInSem01 */
osSemaphoreId_t DataInSem01Handle;
const osSemaphoreAttr_t DataInSem01_attributes = {
  .name = "DataInSem01"
};
/* Definitions for USB_PlugIN_Sem */
osSemaphoreId_t USB_PlugIN_SemHandle;
const osSemaphoreAttr_t USB_PlugIN_Sem_attributes = {
  .name = "USB_PlugIN_Sem"
};
/* USER CODE BEGIN PV */
uint8_t R_buffer[32];
uint32_t len = 0;
volatile uint32_t IWDG_FEED = 0;
uint8_t CMD_ERROR_STATE = ERR_OK;

PinMap pin_map[] = {
    {PIN1_GPIO_Port, PIN1_Pin},
    {PIN2_GPIO_Port, PIN2_Pin},
	{PIN3_GPIO_Port, PIN3_Pin},
	{PIN4_GPIO_Port, PIN4_Pin},
	{PIN5_GPIO_Port, PIN5_Pin},
	{PIN6_GPIO_Port, PIN6_Pin},
	{PIN7_GPIO_Port, PIN7_Pin},
	{PIN8_GPIO_Port, PIN8_Pin},
	{PIN9_GPIO_Port, PIN9_Pin},
	{PIN10_GPIO_Port, PIN10_Pin},
	{PIN11_GPIO_Port, PIN11_Pin},
	{PIN12_GPIO_Port, PIN12_Pin},
	{PIN13_GPIO_Port, PIN13_Pin},
	{PIN14_GPIO_Port, PIN14_Pin},
	{PIN15_GPIO_Port, PIN15_Pin},
	{PIN16_GPIO_Port, PIN16_Pin},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void QueuePush(void *argument);
void CMD_Resp(void *argument);
void IWDG_Feed(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    static uint8_t rc = USBD_OK;
#ifdef USE_BLOCKING_USBVCOM
    do {
        rc = CDC_Transmit_FS((uint8_t *)ptr, len);
    } while (USBD_BUSY == rc);

    if (USBD_FAIL == rc) {
        // NOTE: Should never reach here.
        // TODO: Handle this error.
        return 0;
    }
#else
    rc = CDC_Transmit_FS((uint8_t *)ptr, len);
#endif
    return len;
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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of DataInSem01 */
  DataInSem01Handle = osSemaphoreNew(1, 0, &DataInSem01_attributes);

  /* creation of USB_PlugIN_Sem */
  USB_PlugIN_SemHandle = osSemaphoreNew(1, 0, &USB_PlugIN_Sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of RECV_Queue */
  RECV_QueueHandle = osMessageQueueNew (16, 32, &RECV_Queue_attributes);

  /* creation of RESP_Queue */
  RESP_QueueHandle = osMessageQueueNew (16, 32, &RESP_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of QueuePushTask */
  QueuePushTaskHandle = osThreadNew(QueuePush, NULL, &QueuePushTask_attributes);

  /* creation of CMD_RespTask */
  CMD_RespTaskHandle = osThreadNew(CMD_Resp, NULL, &CMD_RespTask_attributes);

  /* creation of IWDG_FeedTask */
  IWDG_FeedTaskHandle = osThreadNew(IWDG_Feed, NULL, &IWDG_FeedTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1600;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  /*Configure GPIO pin : PIN1_Pin */
  GPIO_InitStruct.Pin = PIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PIN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN2_Pin PIN3_Pin PIN4_Pin PIN5_Pin
                           PIN6_Pin PIN7_Pin PIN8_Pin */
  GPIO_InitStruct.Pin = PIN2_Pin|PIN3_Pin|PIN4_Pin|PIN5_Pin
                          |PIN6_Pin|PIN7_Pin|PIN8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN9_Pin PIN10_Pin PIN11_Pin PIN12_Pin
                           PIN13_Pin PIN14_Pin PIN15_Pin PIN16_Pin */
  GPIO_InitStruct.Pin = PIN9_Pin|PIN10_Pin|PIN11_Pin|PIN12_Pin
                          |PIN13_Pin|PIN14_Pin|PIN15_Pin|PIN16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void CMD_Split(uint8_t * cmd, uint8_t * cmd_sn, uint8_t * cmd_name, uint8_t * cmd_pin, uint8_t * cmd_value, uint8_t * resp_string)
{
    uint8_t * command = cmd;
    uint8_t resp_length = 0;
    uint8_t * underscore = NULL;
    size_t length = 0;
    //get sn
    underscore = memchr(command, '_', strlen((char *)command));
    if(underscore == NULL)
    {
        //invalid cmd fmt
        CMD_ERROR_STATE = ERR_INVALID_FMT;
        return;
    }
    if((length = underscore - command)!=SN_LENGTH)
    {
        //invalid sn
        CMD_ERROR_STATE = ERR_INVALID_SN;
        return;
    }
    memcpy(cmd_sn, command, length);
    memcpy(resp_string, command, length);
    resp_length = length;
    resp_string[resp_length++] = '_';
    command += length + 1;

    //get name
    underscore = memchr(command, '_', strlen((char *)command));
    if(underscore == NULL)
    {
        //invalid cmd fmt
        CMD_ERROR_STATE = ERR_INVALID_FMT;
        return;
    }
    if((length = underscore - command) == 0||length > CMD_NAME_LENGTH)
    {
        //invalid cmd name
        CMD_ERROR_STATE = ERR_INVALID_CMD;
        return;
    }
    memcpy(cmd_name, command, length);
    command += length + 1;

    //get pin
    underscore = memchr(command, '_', strlen((char *)command));
    if(underscore == NULL)
    {
        //case read
        uint8_t * carriage_return = memchr(command, '\r', strlen((char *)command));
        if(carriage_return == NULL)
        {
            //no carriage_return
            CMD_ERROR_STATE = ERR_INVALID_FMT;
            return;
        }
        if((length = carriage_return - command) == 0||length > PIN_LENGTH)
        {
            //invalid pin name
            CMD_ERROR_STATE = ERR_INVALID_PIN;
            return;
        }
        memcpy(cmd_pin, command, length);
        return;
    }
    else
    {
        //case write
        if((length = underscore - command) == 0||length > PIN_LENGTH)
        {
            //invalid pin name
            CMD_ERROR_STATE = ERR_INVALID_PIN;
            return;
        }
        memcpy(cmd_pin, command, length);
        command += length + 1;
    }
    //get pin value
    uint8_t * carriage_return = memchr(command, '\r', strlen((char *)command));
    if(carriage_return == NULL)
    {
        //no carriage_return
        CMD_ERROR_STATE = ERR_INVALID_FMT;
        return;
    }
    if((length = carriage_return - command) != PIN_VALUE_LENGTH)
    {
        // invalid pin value
        CMD_ERROR_STATE = ERR_INVALID_VALUE;
        return;
    }
    memcpy(cmd_value, command, length);
    return;
}

void Execute_CMD(uint8_t * cmd, uint8_t * resp_string)
{
    uint8_t cmd_sn[SN_LENGTH] = {0};
    uint8_t cmd_name[CMD_NAME_LENGTH] = {0};
    uint8_t cmd_pin[PIN_LENGTH] = {0};
    uint8_t cmd_value[PIN_VALUE_LENGTH] = {0};
    uint8_t pin_num = 0;
    CMD_ERROR_STATE = ERR_OK;
    CMD_Split(cmd, cmd_sn, cmd_name, cmd_pin, cmd_value, resp_string);
    if(CMD_ERROR_STATE != ERR_OK) return;
    //get pin number
    pin_num = atoi((char *)&cmd_pin[1]);
    if(cmd_pin[0]!='P'||pin_num == 0||pin_num > 16)
    {
        CMD_ERROR_STATE = ERR_INVALID_PIN;
        return;
        //command pin invalid error
    }

    if(memcmp(cmd_name, READ_CMD, strlen((char *)READ_CMD)) == osOK)
    {
    //Read function
        GPIO_PinState result = HAL_GPIO_ReadPin(pin_map[pin_num-1].GPIO_Port, pin_map[pin_num-1].GPIO_Pin);
        uint8_t resp_length = strlen((char *)resp_string);
        sprintf((char *)&resp_string[resp_length], "%s_P%d:%c", READ_CMD, pin_num, result == GPIO_PIN_SET ? 'H' : 'L');
    }
    else if(memcmp(cmd_name, WRITE_CMD, strlen((char *)WRITE_CMD)) == osOK)
    {
    //Write function
    }
    else
    {
		CMD_ERROR_STATE = ERR_INVALID_CMD;
		return;
    }
}

osStatus_t CMD_Error_Handler(uint8_t * resp_string)
{
    if(strlen((char *)resp_string) == 0) return osError;
    const uint8_t* error_message = NULL;
    switch(CMD_ERROR_STATE){
        case ERR_INVALID_SN:
            error_message = INVALID_SN;
            break;
        case ERR_INVALID_CMD:
            error_message = INVALID_CMD;
            break;
        case ERR_INVALID_PIN:
            error_message = INVALID_PIN;
            break;
        case ERR_INVALID_VALUE:
            error_message = INVALID_VALUE;
            break;
        case ERR_INVALID_FMT:
            error_message = INVALID_FMT;
            break;
        case ERR_UNKNOWN:
            error_message = UNKNOWN_ISSUE;
            break;
        default:
            break;
    }
    if(error_message != NULL) {
        strcat((char *)resp_string, (char *)error_message);
    }
    return osOK;
}

//void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
//{
//	osSemaphoreRelease(USB_PlugIN_SemHandle);
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN 5 */
  uint8_t msg[32] = {0};
  uint8_t resp_string[32];
  /* Infinite loop */
  for(;;)
  {
	IWDG_FEED |= TASK1_BIT;
	if(osMessageQueueGet(RECV_QueueHandle, msg, 0, 1000)==osOK)
	{
		memset(resp_string, 0, sizeof(resp_string));
		Execute_CMD(msg, resp_string);
		if(CMD_ERROR_STATE != ERR_OK) {
			if(CMD_Error_Handler(resp_string) == osError) continue;
		}
		size_t resp_length = strlen((char*)resp_string);
		resp_string[resp_length] = '\r';
		resp_string[resp_length+1] = '\n';
		resp_string[resp_length+2] = '\0';
		while(osMessageQueuePut(RESP_QueueHandle, resp_string, 0U, 0U)==osErrorResource)
		{
			osDelay(100);
			IWDG_FEED |= TASK1_BIT;
		}
		memset(msg, 0, sizeof(msg));
	}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_QueuePush */
/**
* @brief Function implementing the QueuePushTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_QueuePush */
void QueuePush(void *argument)
{
  /* USER CODE BEGIN QueuePush */
  /* Infinite loop */
  for(;;)
  {
	if(osSemaphoreAcquire(DataInSem01Handle, 1000)==osOK)
	{
		while(osMessageQueuePut(RECV_QueueHandle, R_buffer, 0U, 0U)==osErrorResource)
		{
			osDelay(100);
			IWDG_FEED |= TASK2_BIT;
		}
		memset(R_buffer, 0, 32);
	}
	IWDG_FEED |= TASK2_BIT;
  }
  /* USER CODE END QueuePush */
}

/* USER CODE BEGIN Header_CMD_Resp */
/**
* @brief Function implementing the CMD_RespTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CMD_Resp */
void CMD_Resp(void *argument)
{
  /* USER CODE BEGIN CMD_Resp */
  uint8_t msg[32] = {0};
  /* Infinite loop */
  for(;;)
  {
	if(osMessageQueueGet(RESP_QueueHandle, msg, 0, 1000)==osOK)
	{
	  printf("%s", msg);
	  memset(msg, 0, sizeof(msg));
	}
	IWDG_FEED |= TASK3_BIT;
  }
  /* USER CODE END CMD_Resp */
}

/* USER CODE BEGIN Header_IWDG_Feed */
/**
* @brief Function implementing the IWDG_FeedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IWDG_Feed */
void IWDG_Feed(void *argument)
{
  /* USER CODE BEGIN IWDG_Feed */
  /* Infinite loop */
  for(;;)
  {
    if(IWDG_FEED == ALL_TASKS)
	{
      if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
      {
	    Error_Handler();
      }
      IWDG_FEED = 0;
	}
	osDelay(100);
  }
  /* USER CODE END IWDG_Feed */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

