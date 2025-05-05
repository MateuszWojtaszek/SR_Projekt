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

#/**
 * @brief Control register 1 (CTRL_REG1) address.
 *
 * - DR1-DR0: Output Data Rate (ODR) selection (00=100Hz, 01=200Hz, 10=400Hz, 11=800Hz)
 * - BW1-BW0: Bandwidth selection depending on ODR
 * - PD: Power down enable (0 = power-down, 1 = normal/sleep)
 * - Zen, Yen, Xen: Axis enable bits (0 = disable, 1 = enable)
 */
#define CTRL_REG1 0x20
#/**
 * @brief Default configuration value for CTRL_REG1.
 *
 * - Normal mode, all axes enabled
 * - 200 Hz Output Data Rate, 50 Hz bandwidth
 */
#define CTRL_REG1_DATA 0b01101111

#/**
 * @brief Control register 2 (CTRL_REG2) address.
 *
 * - HPM1-HPM0: High-pass filter mode selection
 * - HPCF3-HPCF0: High-pass filter cutoff frequency
 */
#define CTRL_REG2 0x21
#/**
 * @brief Default configuration value for CTRL_REG2.
 *
 * - High-pass filter enabled
 * - Custom cutoff frequency setting
 */
#define CTRL_REG2_DATA 0b00110101

#/**
 * @brief Control register 3 (CTRL_REG3) address.
 *
 * - I1_Int1: Interrupt enable on INT1 pin
 * - I1_Boot: Boot status on INT1 pin
 * - H_Lactive: Interrupt polarity configuration
 * - PP_OD: Push-Pull/Open-Drain configuration
 * - I2_DRDY: Data Ready interrupt on INT2 pin
 * - I2_WTM: FIFO Watermark interrupt on INT2 pin
 * - I2_ORun: FIFO Overrun interrupt on INT2 pin
 * - I2_Empty: FIFO Empty interrupt on INT2 pin
 */
#define CTRL_REG3 0x22
#/**
 * @brief Default configuration value for CTRL_REG3.
 *
 * - Data Ready interrupt enabled on INT2 pin
 */
#define CTRL_REG3_DATA 0b00001000

#/**
 * @brief Control register 4 (CTRL_REG4) address.
 *
 * - BDU: Block Data Update
 * - BLE: Big/Little Endian selection
 * - FS1-FS0: Full scale selection (00=250dps, 01=500dps, 10=2000dps, 11=2000dps)
 * - ST1-ST0: Self-test mode
 * - SIM: SPI mode selection (4-wire/3-wire)
 */
#define CTRL_REG4 0x23
#/**
 * @brief Default configuration value for CTRL_REG4.
 *
 * - Block Data Update enabled
 * - 250 dps full scale
 * - 3-wire SPI mode enabled
 */
#define CTRL_REG4_DATA 0b10000001

#/**
 * @brief Control register 5 (CTRL_REG5) address.
 *
 * - BOOT: Memory reboot command
 * - FIFO_EN: FIFO enable
 * - HPen: High-pass filter enable
 * - INT1_Sel1-0: Interrupt generator configuration
 * - Out_Sel1-0: Data output configuration
 */
#define CTRL_REG5 0x24
#/**
 * @brief Default configuration value for CTRL_REG5.
 *
 * - High-pass filter enabled
 */
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

typedef struct {
  float offset_x;
  float offset_y;
  float offset_z;
} GyroOffsets;

void gyro_init(void);
void gyro_write(uint8_t reg, uint8_t data);
uint8_t gyro_read(uint8_t reg);
void gyro_read_data(GyroFullProcessedData *gyro_full_data);
void gyro_selftest_calibrate(void);
#endif /* GYRO_H_ */
