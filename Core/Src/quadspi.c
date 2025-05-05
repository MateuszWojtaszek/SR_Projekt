/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.c
  * @brief   This file provides code for the configuration
  *          of the QUADSPI instances.
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
#include "quadspi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

QSPI_HandleTypeDef hqspi;

/* QUADSPI init function */
void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 0;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspInit 0 */

  /* USER CODE END QUADSPI_MspInit 0 */
    /* QUADSPI clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**QUADSPI GPIO Configuration
    PE10     ------> QUADSPI_CLK
    PE11     ------> QUADSPI_NCS
    PE12     ------> QUADSPI_BK1_IO0
    PE13     ------> QUADSPI_BK1_IO1
    PE14     ------> QUADSPI_BK1_IO2
    PE15     ------> QUADSPI_BK1_IO3
    */
    GPIO_InitStruct.Pin = FLASH_CLK_Pin|FLASH_CS_Pin|FLASH_D0_Pin|FLASH_D1_Pin
                          |FLASH_D2_Pin|FLASH_D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN QUADSPI_MspInit 1 */

  /* USER CODE END QUADSPI_MspInit 1 */
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{

  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

  /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI GPIO Configuration
    PE10     ------> QUADSPI_CLK
    PE11     ------> QUADSPI_NCS
    PE12     ------> QUADSPI_BK1_IO0
    PE13     ------> QUADSPI_BK1_IO1
    PE14     ------> QUADSPI_BK1_IO2
    PE15     ------> QUADSPI_BK1_IO3
    */
    HAL_GPIO_DeInit(GPIOE, FLASH_CLK_Pin|FLASH_CS_Pin|FLASH_D0_Pin|FLASH_D1_Pin
                          |FLASH_D2_Pin|FLASH_D3_Pin);

  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

  /* USER CODE END QUADSPI_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#define CMD_READ_ID 0x9F

uint8_t Flash_ReadID(void) {
  QSPI_CommandTypeDef sCommand;
  uint8_t flash_id[3];

  sCommand.Instruction = CMD_READ_ID;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.NbData = 3;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return 0;
  }
  if (HAL_QSPI_Receive(&hqspi, flash_id, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return 0;
  }

  return (flash_id[0] << 16) | (flash_id[1] << 8) | flash_id[2];
}

#define CMD_WRITE_ENABLE 0x06

void Flash_WriteEnable(void) {
  QSPI_CommandTypeDef sCommand;
  sCommand.Instruction = CMD_WRITE_ENABLE;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;

  HAL_QSPI_Command(&hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}

#define CMD_SECTOR_ERASE 0xD8

void Flash_EraseSector(uint32_t address) {
  QSPI_CommandTypeDef sCommand;

  Flash_WriteEnable();

  sCommand.Instruction = CMD_SECTOR_ERASE;
  sCommand.Address = address;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;

  HAL_QSPI_Command(&hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}
/* USER CODE END 1 */
