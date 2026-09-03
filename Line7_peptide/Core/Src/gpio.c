/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DC_Motor_pin_Pin|Load_cell_stop_Pin|PUL1A_Pin_Pin|PUL1B_Pin_Pin
                          |DIR1A_Pin_Pin|DIR2A_Pin_Pin|Drain_valve_pin_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, PUL2B_Pin_Pin|Robot_gripper_Pin|X_pump_Pin|Load_cell_start_Pin
                          |Liquid_peristalic_pump_Pin|PULB_Pin_Pin|Pnuematic_3_Pin|Pnuematic_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR2B_Pin_Pin|DIR1B_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PULA_Pin_Pin|Clevage_solvent_motor_1_Pin|Clevage_solvent_motor_2_Pin|Pnuematic_1_Pin
                          |Clevage_solvent_motor_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|DIRA_Pin_Pin|LD3_Pin|Pnuematic_2_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Drain_up_down_limit_Pin|Pnuematic_5_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PUL2A_Pin_Pin|Y_Pump_Pin|Z_Pump_Pin|Clevage_limit_1_Pin
                          |DIRB_Pin_Pin|Clevage_solvent_motor_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DC_Motor_pin_Pin Load_cell_stop_Pin PUL1A_Pin_Pin PUL1B_Pin_Pin
                           DIR1A_Pin_Pin DIR2A_Pin_Pin Drain_valve_pin_Pin_Pin */
  GPIO_InitStruct.Pin = DC_Motor_pin_Pin|Load_cell_stop_Pin|PUL1A_Pin_Pin|PUL1B_Pin_Pin
                          |DIR1A_Pin_Pin|DIR2A_Pin_Pin|Drain_valve_pin_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL2B_Pin_Pin Robot_gripper_Pin X_pump_Pin Load_cell_start_Pin
                           Liquid_peristalic_pump_Pin PULB_Pin_Pin Pnuematic_3_Pin Pnuematic_4_Pin */
  GPIO_InitStruct.Pin = PUL2B_Pin_Pin|Robot_gripper_Pin|X_pump_Pin|Load_cell_start_Pin
                          |Liquid_peristalic_pump_Pin|PULB_Pin_Pin|Pnuematic_3_Pin|Pnuematic_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR2B_Pin_Pin DIR1B_Pin_Pin */
  GPIO_InitStruct.Pin = DIR2B_Pin_Pin|DIR1B_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PULA_Pin_Pin Clevage_solvent_motor_1_Pin Clevage_solvent_motor_2_Pin Pnuematic_1_Pin
                           Clevage_solvent_motor_3_Pin */
  GPIO_InitStruct.Pin = PULA_Pin_Pin|Clevage_solvent_motor_1_Pin|Clevage_solvent_motor_2_Pin|Pnuematic_1_Pin
                          |Clevage_solvent_motor_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin DIRA_Pin_Pin LD3_Pin Pnuematic_2_Pin
                           LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|DIRA_Pin_Pin|LD3_Pin|Pnuematic_2_Pin
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Load_cell_feedback_Pin USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = Load_cell_feedback_Pin|USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : Drain_up_down_limit_Pin Pnuematic_5_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = Drain_up_down_limit_Pin|Pnuematic_5_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL2A_Pin_Pin Y_Pump_Pin Z_Pump_Pin Clevage_limit_1_Pin
                           DIRB_Pin_Pin Clevage_solvent_motor_4_Pin */
  GPIO_InitStruct.Pin = PUL2A_Pin_Pin|Y_Pump_Pin|Z_Pump_Pin|Clevage_limit_1_Pin
                          |DIRB_Pin_Pin|Clevage_solvent_motor_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Clevage_limit_2_Pin */
  GPIO_InitStruct.Pin = Clevage_limit_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Clevage_limit_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Proximity_sensing_Pin */
  GPIO_InitStruct.Pin = Proximity_sensing_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Proximity_sensing_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
