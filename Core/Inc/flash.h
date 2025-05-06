#ifndef FLASH_H
#define FLASH_H

#include "quadspi.h"
#include <stdint.h>

// Definicje statusów QSPI
#define QSPI_OK 0x00
#define QSPI_ERROR 0x01
#define QSPI_BUSY 0x02
#define QSPI_NOT_SUPPORTED 0x04
#define QSPI_SUSPENDED 0x08

typedef struct {
  uint32_t FlashSize;
  uint32_t EraseSectorSize;

  uint32_t EraseSectorsNumber;
  uint32_t ProgPageSize;
  uint32_t ProgPagesNumber;
} FlashInfo_t;

static uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi_internal);
static uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi_internal,
                                        uint32_t Timeout);
static uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi_internal);

uint8_t Flash_Init(void);

uint8_t Flash_Erase_SubSector(uint32_t Address);

uint8_t Flash_Erase_Chip(void);

uint8_t Flash_Write(uint8_t *pData, uint32_t Address, uint32_t Size);
uint8_t Flash_Read(uint8_t *pData, uint32_t Address, uint32_t Size);
uint8_t Flash_GetInfo(FlashInfo_t *pInfo);

#endif // FLASH_H
