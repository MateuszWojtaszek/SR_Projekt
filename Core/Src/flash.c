#include "flash.h"
#include "n25q128a.h"
#include <stdio.h>
#include <string.h>

#define QSPI_DEFAULT_TIMEOUT 5000

uint8_t Flash_Init(void) {
  if (hqspi.State == HAL_QSPI_STATE_RESET) {
    // QSPI nie został zainicjowany przez HAL (MX_QUADSPI_Init)
    return QSPI_ERROR;
  }

  // reset pamięci przy starcie
  if (QSPI_ResetMemory(&hqspi) != QSPI_OK) {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

uint8_t Flash_Erase_SubSector(uint32_t Address) {
  QSPI_CommandTypeDef s_command;

  // Adres musi być w zakresie pamięci
  if (Address >= N25Q128A_FLASH_SIZE) {
    return QSPI_ERROR;
  }

  // Inicjalizacja komendy kasowania subsektora
  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction = SUBSECTOR_ERASE_CMD;
  s_command.AddressMode = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize = QSPI_ADDRESS_24_BITS;
  s_command.Address = Address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode = QSPI_DATA_NONE;
  s_command.DummyCycles = 0;
  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (QSPI_WriteEnable(&hqspi) != QSPI_OK) {
    return QSPI_ERROR;
  }
  if (HAL_QSPI_Command(&hqspi, &s_command, QSPI_DEFAULT_TIMEOUT) != HAL_OK) {
    return QSPI_ERROR;
  }
  if (QSPI_AutoPollingMemReady(&hqspi, N25Q128A_SUBSECTOR_ERASE_MAX_TIME) !=
      QSPI_OK) {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

uint8_t Flash_Erase_Chip(void) {
  QSPI_CommandTypeDef s_command;

  // Inicjalizacja komendy kasowania całej pamięci
  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction = BULK_ERASE_CMD;
  s_command.AddressMode = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode = QSPI_DATA_NONE;
  s_command.DummyCycles = 0;
  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (QSPI_WriteEnable(&hqspi) != QSPI_OK) {
    return QSPI_ERROR;
  }
  if (HAL_QSPI_Command(&hqspi, &s_command, QSPI_DEFAULT_TIMEOUT) != HAL_OK) {
    return QSPI_ERROR;
  }
  if (QSPI_AutoPollingMemReady(&hqspi, N25Q128A_BULK_ERASE_MAX_TIME) !=
      QSPI_OK) {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

uint8_t Flash_Write(uint8_t *pData, uint32_t Address, uint32_t Size) {
  QSPI_CommandTypeDef s_command;
  uint32_t end_addr, current_size, current_addr;

  // Sprawdzenie zakresu adresu i rozmiaru
  if ((Address + Size) > N25Q128A_FLASH_SIZE) {
    return QSPI_ERROR;
  }

  // rozmiar pozostały na stronie
  current_size = N25Q128A_PAGE_SIZE - (Address % N25Q128A_PAGE_SIZE);

  if (current_size > Size) {
    current_size = Size;
  }

  current_addr = Address;
  end_addr = Address + Size;

  // Inicjalizuj komendę programowania strony (standardowa 0x02)
  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction = PAGE_PROG_CMD;
  s_command.AddressMode = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode = QSPI_DATA_1_LINE;
  s_command.DummyCycles = 0;
  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  do {
    s_command.Address = current_addr;
    s_command.NbData = current_size;

    if (QSPI_WriteEnable(&hqspi) != QSPI_OK) {
      return QSPI_ERROR;
    }
    if (HAL_QSPI_Command(&hqspi, &s_command, QSPI_DEFAULT_TIMEOUT) != HAL_OK) {
      return QSPI_ERROR;
    }
    if (HAL_QSPI_Transmit(&hqspi, pData, QSPI_DEFAULT_TIMEOUT) != HAL_OK) {
      return QSPI_ERROR;
    }
    if (QSPI_AutoPollingMemReady(&hqspi, QSPI_DEFAULT_TIMEOUT) != QSPI_OK) {
      return QSPI_ERROR;
    }

    // Zaktualizuj adres i rozmiar dla następnej strony
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + N25Q128A_PAGE_SIZE) > end_addr)
                       ? (end_addr - current_addr)
                       : N25Q128A_PAGE_SIZE;

  } while (current_addr < end_addr);

  return QSPI_OK;
}

uint8_t Flash_Read(uint8_t *pData, uint32_t Address, uint32_t Size) {
  QSPI_CommandTypeDef s_command;

  // Sprawdzenie zakresu adresu i rozmiaru
  if ((Address + Size) > N25Q128A_FLASH_SIZE) {
    printf("BŁĄD Flash_Read: Adres+Rozmiar (0x%lX) poza zakresem (0x%lX)!\n",
           Address + Size, N25Q128A_FLASH_SIZE);
    return QSPI_ERROR;
  }
  if (pData == NULL || Size == 0) {
    printf("BŁĄD Flash_Read: Nieprawidłowy bufor lub rozmiar = 0!\n");
    return QSPI_ERROR;
  }

  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction = READ_CMD;
  s_command.AddressMode = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize = QSPI_ADDRESS_24_BITS;
  s_command.Address = Address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode = QSPI_DATA_1_LINE;
  s_command.DummyCycles = 0;

  s_command.NbData = Size;
  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &s_command, QSPI_DEFAULT_TIMEOUT) != HAL_OK) {
    printf("BŁĄD Flash_Read: HAL_QSPI_Command nie powiódł się!\n");
    return QSPI_ERROR;
  }
  if (HAL_QSPI_Receive(&hqspi, pData, QSPI_DEFAULT_TIMEOUT) != HAL_OK) {
    printf("BŁĄD Flash_Read: HAL_QSPI_Receive nie powiódł się!\n");
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

uint8_t Flash_GetInfo(FlashInfo_t *pInfo) {
  if (pInfo == NULL) {
    return QSPI_ERROR;
  }
  pInfo->FlashSize = N25Q128A_FLASH_SIZE;
  pInfo->EraseSectorSize = N25Q128A_SUBSECTOR_SIZE;
  pInfo->EraseSectorsNumber = (N25Q128A_FLASH_SIZE / N25Q128A_SUBSECTOR_SIZE);
  pInfo->ProgPageSize = N25Q128A_PAGE_SIZE;
  pInfo->ProgPagesNumber = (N25Q128A_FLASH_SIZE / N25Q128A_PAGE_SIZE);

  return QSPI_OK;
}

static uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi_internal) {
  QSPI_CommandTypeDef s_command;
  QSPI_AutoPollingTypeDef s_config;

  // Włącz operacje zapisu (komenda 0x06)
  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction = WRITE_ENABLE_CMD;
  s_command.AddressMode = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode = QSPI_DATA_NONE;
  s_command.DummyCycles = 0;
  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi_internal, &s_command, QSPI_DEFAULT_TIMEOUT) !=
      HAL_OK) {
    return QSPI_ERROR;
  }

  s_config.Match = N25Q128A_SR_WREN;
  s_config.Mask = N25Q128A_SR_WREN;
  s_config.MatchMode = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval = 0x10;
  s_config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.Instruction = READ_STATUS_REG_CMD;
  s_command.DataMode = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(hqspi_internal, &s_command, &s_config,
                           QSPI_DEFAULT_TIMEOUT) != HAL_OK) {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

static uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi_internal,
                                        uint32_t Timeout) {
  QSPI_CommandTypeDef s_command;
  QSPI_AutoPollingTypeDef s_config;

  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction = READ_STATUS_REG_CMD; // 0x05
  s_command.AddressMode = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode = QSPI_DATA_1_LINE;
  s_command.DummyCycles = 0;
  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  s_config.Match = 0x00;
  s_config.Mask = N25Q128A_SR_WIP;
  s_config.MatchMode = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval = 0x10;
  s_config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(hqspi_internal, &s_command, &s_config, Timeout) !=
      HAL_OK) {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

static uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi_internal) {
  QSPI_CommandTypeDef s_command;

  // Wyślij komendę Reset Enable (0x66)
  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction = RESET_ENABLE_CMD;
  s_command.AddressMode = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode = QSPI_DATA_NONE;
  s_command.DummyCycles = 0;
  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi_internal, &s_command, QSPI_DEFAULT_TIMEOUT) !=
      HAL_OK) {
    return QSPI_ERROR;
  }
  s_command.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(hqspi_internal, &s_command, QSPI_DEFAULT_TIMEOUT) !=
      HAL_OK) {
    return QSPI_ERROR;
  }
  if (QSPI_AutoPollingMemReady(hqspi_internal, QSPI_DEFAULT_TIMEOUT) !=
      QSPI_OK) {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}
