#ifndef GYRO_H_
#define GYRO_H_

#include <stdint.h>

#include <spi.h>
#include <stdint.h>
#include <stm32l4xx_hal_def.h>
#include <math.h>
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_spi.h"

#define WHO_AM_I 0x0F

#define CTRL_REG1 0x20
#define CTRL_REG1_DATA 0b10101111

#define CTRL_REG2 0x21
#define CTRL_REG2_DATA 0b00000010

#define CTRL_REG3 0x22
#define CTRL_REG3_DATA 0x00

#define CTRL_REG4 0x23
#define CTRL_REG4_DATA 0b00010001

#define CTRL_REG5 0x24
#define CTRL_REG5_DATA 0b00010000

#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define OUT_TEMP 0x26

#define GYRO_CALIBRATION_SAMPLES 1000


typedef struct {
  float x_dps;
  float y_dps;
  float z_dps;

  float temperature_c;     // Converted temperature in Celsius (optional)

} GyroFullProcessedData;

void gyro_init(void);
void gyro_calibrate(void);
void gyro_write(uint8_t reg, uint8_t data);
uint8_t gyro_read(uint8_t reg);
void gyro_read_data(GyroFullProcessedData *gyro_full_data);
#endif /* GYRO_H_ */
