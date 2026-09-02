/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "Stepper.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DC_Motor_pin_Pin GPIO_PIN_3
#define DC_Motor_pin_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define PUL2B_Pin_Pin GPIO_PIN_2
#define PUL2B_Pin_GPIO_Port GPIOF
#define Robot_gripper_Pin GPIO_PIN_4
#define Robot_gripper_GPIO_Port GPIOF
#define X_pump_Pin GPIO_PIN_7
#define X_pump_GPIO_Port GPIOF
#define Load_cell_start_Pin GPIO_PIN_8
#define Load_cell_start_GPIO_Port GPIOF
#define Liquid_peristalic_pump_Pin GPIO_PIN_9
#define Liquid_peristalic_pump_GPIO_Port GPIOF
#define PULB_Pin_Pin GPIO_PIN_10
#define PULB_Pin_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define DIR2B_Pin_Pin GPIO_PIN_3
#define DIR2B_Pin_GPIO_Port GPIOC
#define PULA_Pin_Pin GPIO_PIN_0
#define PULA_Pin_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define Clevage_solvent_motor_1_Pin GPIO_PIN_4
#define Clevage_solvent_motor_1_GPIO_Port GPIOA
#define Clevage_solvent_motor_2_Pin GPIO_PIN_5
#define Clevage_solvent_motor_2_GPIO_Port GPIOA
#define Pnuematic_1_Pin GPIO_PIN_6
#define Pnuematic_1_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define DIRA_Pin_Pin GPIO_PIN_1
#define DIRA_Pin_GPIO_Port GPIOB
#define Pnuematic_3_Pin GPIO_PIN_14
#define Pnuematic_3_GPIO_Port GPIOF
#define Pnuematic_4_Pin GPIO_PIN_15
#define Pnuematic_4_GPIO_Port GPIOF
#define Load_cell_feedback_Pin GPIO_PIN_0
#define Load_cell_feedback_GPIO_Port GPIOG
#define Drain_up_down_limit_Pin GPIO_PIN_1
#define Drain_up_down_limit_GPIO_Port GPIOG
#define Load_cell_stop_Pin GPIO_PIN_10
#define Load_cell_stop_GPIO_Port GPIOE
#define PUL1A_Pin_Pin GPIO_PIN_11
#define PUL1A_Pin_GPIO_Port GPIOE
#define PUL1B_Pin_Pin GPIO_PIN_13
#define PUL1B_Pin_GPIO_Port GPIOE
#define DIR1A_Pin_Pin GPIO_PIN_14
#define DIR1A_Pin_GPIO_Port GPIOE
#define DIR2A_Pin_Pin GPIO_PIN_15
#define DIR2A_Pin_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define PUL2A_Pin_Pin GPIO_PIN_10
#define PUL2A_Pin_GPIO_Port GPIOD
#define Y_Pump_Pin GPIO_PIN_12
#define Y_Pump_GPIO_Port GPIOD
#define Z_Pump_Pin GPIO_PIN_13
#define Z_Pump_GPIO_Port GPIOD
#define Pnuematic_5_Pin GPIO_PIN_5
#define Pnuematic_5_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define DIR1B_Pin_Pin GPIO_PIN_6
#define DIR1B_Pin_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Clevage_solvent_motor_3_Pin GPIO_PIN_15
#define Clevage_solvent_motor_3_GPIO_Port GPIOA
#define Clevage_limit_1_Pin GPIO_PIN_1
#define Clevage_limit_1_GPIO_Port GPIOD
#define DIRB_Pin_Pin GPIO_PIN_2
#define DIRB_Pin_GPIO_Port GPIOD
#define Clevage_solvent_motor_4_Pin GPIO_PIN_3
#define Clevage_solvent_motor_4_GPIO_Port GPIOD
#define Load_cell_Tx_Pin GPIO_PIN_5
#define Load_cell_Tx_GPIO_Port GPIOD
#define Load_cell_Rx_Pin GPIO_PIN_6
#define Load_cell_Rx_GPIO_Port GPIOD
#define Servo_UART_Pin GPIO_PIN_9
#define Servo_UART_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define Servo_UARTG14_Pin GPIO_PIN_14
#define Servo_UARTG14_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Pnuematic_2_Pin GPIO_PIN_5
#define Pnuematic_2_GPIO_Port GPIOB
#define Clevage_limit_2_Pin GPIO_PIN_6
#define Clevage_limit_2_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define Proximity_sensing_Pin GPIO_PIN_9
#define Proximity_sensing_GPIO_Port GPIOB
#define Drain_valve_pin_Pin_Pin GPIO_PIN_0
#define Drain_valve_pin_Pin_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
