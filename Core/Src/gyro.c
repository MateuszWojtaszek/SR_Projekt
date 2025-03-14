#include <gyro.h>
#include <spi.h>
#include <stdint.h>
#include <stm32l4xx_hal_def.h>

void gyro_init(void) {}

void gyro_write(uint8_t reg, uint8_t data) {
  uint8_t tx[2] = {reg, data};
  HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY);
}
