/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "i2c.h" // <-- ADD THIS
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
osThreadId_t laserTaskHandle;
const osThreadAttr_t laserTask_attributes = {
  .name = "laserTask",
  .stack_size = 512 * 4, // 2048 bytes
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 256 * 4, // 1024 bytes
  .priority = (osPriority_t) osPriorityHigh,
};

osSemaphoreId_t buttonSemHandle;
const osSemaphoreAttr_t buttonSem_attributes = {
  .name = "buttonSem"
};

osMutexId_t printfMutexHandle;
const osMutexAttr_t printfMutex_attributes = {
  .name = "printfMutex"
};

osThreadId_t gameLogicTaskHandle;
const osThreadAttr_t gameLogicTask_attributes = {
  .name = "gameLogicTask",
  .stack_size = 256 * 4, // 1024 bytes
  .priority = (osPriority_t) osPriorityNormal,
};

osMutexId_t gameMutexHandle;
const osMutexAttr_t gameMutex_attributes = {
  .name = "gameMutex"
};

osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .stack_size = 128 * 4, // 512 bytes
  .priority = (osPriority_t) osPriorityLow,
};

osSemaphoreId_t buzzerSemHandle;
const osSemaphoreAttr_t buzzerSem_attributes = {
  .name = "buzzerSem"
};

osThreadId_t dynamicLaserTaskHandle;
const osThreadAttr_t dynamicLaserTask_attributes = {
  .name = "dynamicLaserTask",
  .stack_size = 128 * 4, // 512 bytes
  .priority = (osPriority_t) osPriorityBelowNormal,
};

// Game State Variables
volatile int gameTimer = 60; // 60 seconds
volatile int gameRunning = 0; // 0=Idle, 1=Playing, 2=Lost, 3=Won
volatile int isLaser3Active = 1; // 1=On, 0=Off
volatile int lastButtonPressed = 0; // 1=Blue(Start/Reset), 2=External(Win)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void StartLaserTask(void *argument);
void StartButtonTask(void *argument);
void StartGameLogicTask(void *argument);
void StartBuzzerTask(void *argument);
void StartDynamicLaserTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Retargets printf to use UART2
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
    // --- NOW WE CREATE TASKS AND OBJECTS ---
    buttonSemHandle = osSemaphoreNew(1, 0, &buttonSem_attributes); // Max count 1, initial count 0
    printfMutexHandle = osMutexNew(&printfMutex_attributes);
    gameMutexHandle = osMutexNew(&gameMutex_attributes);
    buzzerSemHandle = osSemaphoreNew(1, 0, &buzzerSem_attributes);

    // Create the tasks
    laserTaskHandle = osThreadNew(StartLaserTask, NULL, &laserTask_attributes);
    buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);
    gameLogicTaskHandle = osThreadNew(StartGameLogicTask, NULL, &gameLogicTask_attributes);
    dynamicLaserTaskHandle = osThreadNew(StartDynamicLaserTask, NULL, &dynamicLaserTask_attributes); // <-- ADD THIS
      /* Start scheduler */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  External Line Interrupt Callback. This is the ISR.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // 1. Check Blue Button (PC13) -> Start / Reset
  if(GPIO_Pin == GPIO_PIN_13)
  {
    lastButtonPressed = 1; // 1 = Blue Button
    osSemaphoreRelease(buttonSemHandle);
  }
  // 2. Check External Button (SystemClock_Config) -> Win
  else if(GPIO_Pin == GPIO_PIN_0)
  {
    lastButtonPressed = 2; // 2 = External Button
    osSemaphoreRelease(buttonSemHandle);
  }
}

/**
  * @brief Function implementing the buttonTask thread.
  */
/**
  * @brief Function implementing the buttonTask thread.
  */
/**
  * @brief Function implementing the buttonTask thread.
  */
/**
  * @brief Function implementing the buttonTask thread.
  */
/**
  * @brief Function implementing the buttonTask thread.
  */
/**
  * @brief Function implementing the buttonTask thread.
  */
/**
  * @brief Function implementing the buttonTask thread.
  */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  for(;;)
  {
    // Wait for ANY button press (Semaphore released by ISR)
    if (osSemaphoreAcquire(buttonSemHandle, osWaitForever) == osOK)
    {
      // Simple mechanical debounce (waits for the spring inside the button to settle)
      // We keep this because all physical buttons bounce a little.
      osDelay(50);

      // --- NOISE FILTER REMOVED ---
      // We assume if the semaphore was released, it was a real press.

      // Lock the game state
      if (osMutexAcquire(gameMutexHandle, osWaitForever) == osOK)
      {
        // ------------------------------------------
        // LOGIC FOR BLUE BUTTON (START / RESET)
        // ------------------------------------------
        if (lastButtonPressed == 1)
        {
          // If Game is Idle (0) -> START IT
          if (gameRunning == 0)
          {
            gameRunning = 1;
            gameTimer = 60;

            // Turn ON Lasers
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

            // Silence Buzzer
            HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);

            if (osMutexAcquire(printfMutexHandle, osWaitForever) == osOK) {
              printf("\r\n--- GAME START! --- \r\n");
              osMutexRelease(printfMutexHandle);
            }
          }
          // If Game is Running -> RESET IT
          else
          {
            gameRunning = 0;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);

            if (osMutexAcquire(printfMutexHandle, osWaitForever) == osOK) {
              printf("\r\n--- GAME RESET. ---\r\n");
              osMutexRelease(printfMutexHandle);
            }
          }
        }

        // ------------------------------------------
        // LOGIC FOR EXTERNAL BUTTON (WIN)
        // ------------------------------------------
        else if (lastButtonPressed == 2)
        {
          // Only win if game is running
          if (gameRunning == 1)
          {
            gameRunning = 3; // 3 = Win

            // Turn OFF Lasers
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

            // WAKE UP BUZZER
            osSemaphoreRelease(buzzerSemHandle);

            if (osMutexAcquire(printfMutexHandle, osWaitForever) == osOK) {
              printf("\r\n--- YOU WIN! ---\r\n");
              osMutexRelease(printfMutexHandle);
            }
          }
        }

        osMutexRelease(gameMutexHandle);
      }

      // Long delay to prevent double-clicks
      osDelay(300);
    }
  }
  /* USER CODE END StartButtonTask */
}
/**
  * @brief Function implementing the gameLogicTask thread.
  */
void StartGameLogicTask(void *argument)
{
  /* USER CODE BEGIN StartGameLogicTask */
  for(;;)
  {
    osDelay(1000);

    if (osMutexAcquire(gameMutexHandle, osWaitForever) == osOK)
    {
      if (gameRunning == 1)
      {
        gameTimer--;

        if (osMutexAcquire(printfMutexHandle, osWaitForever) == osOK) {
          printf("Time remaining: %d\r\n", gameTimer);
          osMutexRelease(printfMutexHandle);
        }

        // Audible Cues
        if (gameTimer == 50 || gameTimer == 40 || gameTimer == 30 || gameTimer == 20) {
          HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_SET);
          osDelay(200);
          HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);
        }
        else if (gameTimer <= 10 && gameTimer > 0) {
          HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_SET);
          osDelay(50);
          HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);
        }

        if (gameTimer <= 0)
        {
          gameRunning = 2; // Lost
          osSemaphoreRelease(buzzerSemHandle);

          if (osMutexAcquire(printfMutexHandle, osWaitForever) == osOK) {
            printf("\r\n--- TIME'S UP! YOU LOSE! --- \r\n");
            osMutexRelease(printfMutexHandle);
          }
        }
      }
      osMutexRelease(gameMutexHandle);
    }
  }
  /* USER CODE END StartGameLogicTask */
}

// I2C Address for PCF8591
#define PCF_MODULE_ADDRESS (0x48 << 1)
#define PCF_READ_AIN1 (0x04 | 1)

/**
 * @brief Helper function to read AIN1 from a specific I2C bus
 */
int read_ldr_from_bus(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t i2c_data_buffer[2];

    i2c_data_buffer[0] = PCF_READ_AIN1;
    status = HAL_I2C_Master_Transmit(hi2c, PCF_MODULE_ADDRESS, i2c_data_buffer, 1, 100);

    if (status != HAL_OK) {
    	HAL_I2C_DeInit(hi2c);
    	HAL_I2C_Init(hi2c);
    	return -1;
    }

    status = HAL_I2C_Master_Receive(hi2c, PCF_MODULE_ADDRESS, i2c_data_buffer, 2, 100);
    if (status != HAL_OK) {
    	HAL_I2C_DeInit(hi2c);
    	HAL_I2C_Init(hi2c);
    	return -1;
    }

    return i2c_data_buffer[1];
}

void StartLaserTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  int ldr1_val, ldr2_val, ldr3_val;
  uint8_t beam_broken = 0;

  for(;;)
  {
    if (gameRunning == 1)
    {
      beam_broken = 0;

      ldr1_val = read_ldr_from_bus(&hi2c1);
      ldr2_val = read_ldr_from_bus(&hi2c2);

      if (isLaser3Active == 1) {
        ldr3_val = read_ldr_from_bus(&hi2c3);
      } else {
    	ldr3_val = 0;
      }

      if ( (ldr1_val == -1 || ldr1_val > 40) ||
           (ldr2_val == -1 || ldr2_val > 40) ||
           (isLaser3Active == 1 && (ldr3_val == -1 || ldr3_val > 40)) )
      {
        beam_broken = 1;
        HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_SET);
      }
      else {
        beam_broken = 0;
        HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);
      }

      if (beam_broken == 1)
      {
          if (osMutexAcquire(gameMutexHandle, osWaitForever) == osOK)
          {
            if (gameRunning == 1) {
              gameTimer -= 10;
              if (gameTimer < 0) gameTimer = 0;
              if (osMutexAcquire(printfMutexHandle, osWaitForever) == osOK) {
                printf("!!! BEAM BROKEN! (LDRs: %d, %d, %d) -10 SECONDS !!!\r\n", ldr1_val, ldr2_val, ldr3_val);
                osMutexRelease(printfMutexHandle);
              }
            }
            osMutexRelease(gameMutexHandle);
          }
          osDelay(1000);
      }
    }
    else {
      HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);
    }
    osDelay(200);
  }
  /* USER CODE END 5 */
}

/**
  * @brief Function implementing the buzzerTask thread.
  */
/**
  * @brief Function implementing the buzzerTask thread.
  */
void StartBuzzerTask(void *argument)
{
  /* USER CODE BEGIN StartBuzzerTask */
  /* Infinite loop */
  for(;;)
  {
    // Wait for signal (Win or Lose)
    if (osSemaphoreAcquire(buzzerSemHandle, osWaitForever) == osOK)
    {
      // -------------------------------------------------
      // SCENARIO A: PLAYER LOST (Slow Beeping)
      // -------------------------------------------------
      if (gameRunning == 2)
      {
        for(int i = 0; i < 10; i++)
        {
          if (gameRunning != 2) break; // Stop if reset

          HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_SET);
          osDelay(150);
          HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);
          osDelay(150);
        }
      }

      // -------------------------------------------------
      // SCENARIO B: PLAYER WON (Victory Tune!)
      // -------------------------------------------------
      else if (gameRunning == 3)
      {
        // Pattern: Beep-Beep-Beep-Hooooold
        // We loop this pattern 3 times
        for(int i = 0; i < 3; i++)
        {
           if (gameRunning != 3) break; // Stop if reset

           // 3 Short Fast Beeps
           for(int j=0; j<3; j++) {
             HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_SET);
             osDelay(80);
             HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);
             osDelay(80);
           }

           // 1 Long Happy Beep
           HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_SET);
           osDelay(400);
           HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);

           // Pause before repeating
           osDelay(300);
        }
      }
    }
  }
  /* USER CODE END StartBuzzerTask */
}
/**
  * @brief Function implementing the dynamicLaserTask thread.
  */
void StartDynamicLaserTask(void *argument)
{
  /* USER CODE BEGIN StartDynamicLaserTask */
  for(;;)
  {
    isLaser3Active = 1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    osDelay(3000);

    isLaser3Active = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    osDelay(3000);
  }
  /* USER CODE END StartDynamicLaserTask */
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
#ifdef USE_FULL_ASSERT
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
