

#include <gyro.h>


#include <stdlib.h>

float bias_x = 0.0f;
float bias_y = 0.0f;
float bias_z = 0.0f;

void gyro_write(uint8_t reg, uint8_t data) {
  uint8_t tx[2] = {reg & 0x7F, data};
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, tx, 2, 1000);
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

void gyro_init(void) {
  gyro_write(CTRL_REG1, CTRL_REG1_DATA);
  gyro_write(CTRL_REG2, CTRL_REG2_DATA);
  gyro_write(CTRL_REG3, CTRL_REG3_DATA);
  gyro_write(CTRL_REG4, CTRL_REG4_DATA);
  gyro_write(CTRL_REG5, CTRL_REG5_DATA);
}

uint8_t gyro_read(uint8_t reg) {
  uint8_t tx = reg | 0x80;
  uint8_t rx = 0;
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &tx, 1, 1000);
  HAL_SPI_Receive(&hspi2, &rx, 1, 1000);
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
  return rx;
}

void gyro_calibrate(void) {
  int32_t sum_x = 0, sum_y = 0, sum_z = 0;
  GyroFullProcessedData raw_data;

  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    gyro_read_data(&raw_data);
    sum_x += raw_data.x_dps;
    sum_y += raw_data.y_dps;
    sum_z += raw_data.z_dps;
    HAL_Delay(2); // Delay to prevent overload
  }

  bias_x = sum_x / (float)GYRO_CALIBRATION_SAMPLES;
  bias_y = sum_y / (float)GYRO_CALIBRATION_SAMPLES;
  bias_z = sum_z / (float)GYRO_CALIBRATION_SAMPLES;
}

void gyro_read_data(GyroFullProcessedData *gyro_full_data) {
  const uint8_t tx = OUT_X_L | 0xC0;
  uint8_t rx[6];
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &tx, 1, 1000);
  HAL_SPI_Receive(&hspi2, rx, 6, 1000);
  uint8_t temp = gyro_read(OUT_TEMP);
  gyro_full_data->x_dps = ((float)((int16_t)(rx[1] << 8) | rx[0])/17.5f) - bias_x;
  gyro_full_data->y_dps = ((float)((int16_t)(rx[3] << 8) | rx[2])/17.5f) - bias_y;
  gyro_full_data->z_dps = ((float)((int16_t)(rx[5] << 8) | rx[4])/17.5f) - bias_z;
  gyro_full_data->temperature_c = (int8_t)temp;
}
