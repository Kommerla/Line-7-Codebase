/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body -- LINE 7 peptide synthesizer
  *
  *  Boots the peripherals, initialises the RS485 field-bus devices, then hands
  *  the CPU to FreeRTOS. The peptide application (FR5 six-axis arm + rail + the
  *  flow-SPPS station sequence) runs in the appMain task -- see freertos.c and
  *  Application/Peptide/peptide_app.c.
  *
  *  All customisations live inside USER CODE blocks so STM32CubeMX preserves
  *  them when you regenerate after enabling FreeRTOS + LWIP (already set in the
  *  .ioc). See INTEGRATION_GUIDE.md.
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
#include "cmsis_os.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "uart.h"
#include "Load.h"
#include "timer_app.h"   /* timerAppInit() -- starts TIM2 for delay_us() */
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* PE9 = load-cell feedback INPUT. Changing PE9's mode to input in CubeMX
 * cleared its user label, so main.h no longer generates these two defines.
 * Re-declare them here (USER CODE = regen-safe). Matches gpio.c, which now
 * configures PE9/GPIOE as GPIO_MODE_INPUT.
 * (To make these come from main.h again, just re-enter the User Label
 *  "Load_cell_feedback" on PE9 in CubeMX and regenerate.) */
#ifndef Load_cell_feedback_Pin
#define Load_cell_feedback_Pin        GPIO_PIN_9
#define Load_cell_feedback_GPIO_Port  GPIOE
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Servo bus handle: shared RS485/Modbus bus (USART2). Addresses:
 *   MIXER_ID (0x01) = St5 stir servo, MicroID (0x02) = St2 amino carousel,
 *   ROBOT_LINEAR_ID (0x03) = arm linear rail. */
static ServoHandle ServoObject;
ServoHandle *servo = &ServoObject;

/* Load cell / vibratory feeder controller (USART6). */
static LoadCellHandle LoadCellObject;
LoadCellHandle *loadcell = &LoadCellObject;

#define LOADCELL_SLAVE_ID   0x01

/* MachineMenu.c declares this as `extern uint8_t SelectedID;` (transient
 * scratch var for whichever servo the numbered menu is currently driving,
 * e.g. MIXER_ID / MicroID) -- it must be DEFINED somewhere. Restored here
 * to match the original menu-based main.c. */
uint8_t SelectedID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void stepperTimerTick(void);    /* stepper.c -- motor 1, one TIM7 step sub-state  */
void stepperTimerTick2(void);   /* stepper.c -- motor 2, one TIM10 step sub-state */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Retarget printf() to the USART3 console so the ported robot module's
 * logging works alongside UART_Print(). syscalls.c's _write() calls this. */
int __io_putchar(int ch)
{
	uint8_t c = (uint8_t)ch;
	HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
	return ch;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  /* Start TIM2 so delay_us()/delay_ms() in timer_app.c actually count -- the
   * new_stepper.c driver blocks in delay_us() forever without this. Inside
   * USER CODE so CubeMX keeps it on regenerate. */
  timerAppInit();

  /* RS485 field-bus devices (must be up before the RTOS tasks use them):
       USART2 -> MAX485 -> servo drives (St5 mixer, St2 carousel, arm rail)
       USART6 -> MAX485 -> load cell / vibratory feeder controller */
  ServoInit(servo, &huart6);
  //ServoPositionConfig(servo,MicroID);
  //ServoPositionConfig(servo, MIXER_ID);

  LoadCell_Init(loadcell, &huart2,
                Load_cell_start_GPIO_Port, Load_cell_start_Pin,
                Load_cell_stop_GPIO_Port, Load_cell_stop_Pin,
                Load_cell_feedback_GPIO_Port, Load_cell_feedback_Pin,
                LOADCELL_SLAVE_ID);

  UART_Print("\r\nLINE 7 boot: peripherals up, starting RTOS...\r\n");
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7)
  {
    stepperTimerTick();     /* motor 1 */
  }
  if (htim->Instance == TIM10)
  {
    stepperTimerTick2();    /* motor 2 */
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Visible fault indicator: fast-blink LD3 (red, PB14) at ~5 Hz so a HAL
   * init failure is immediately obvious without a debugger attached.
   * Interrupts stay disabled so nothing else can run. */
  __disable_irq();
  /* Ensure GPIOB clock is on and PB14 is output (may already be done by
   * MX_GPIO_Init, but Error_Handler can fire before that). */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gi = {0};
  gi.Pin   = LD3_Pin;  /* PB14 */
  gi.Mode  = GPIO_MODE_OUTPUT_PP;
  gi.Pull  = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gi);
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
    for (volatile uint32_t d = 0; d < 800000; d++) { }
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
