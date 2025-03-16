//
// Created by Mateusz Wojtaszek on 15/03/2025.
//
#include "LSM303C.h"
#include <spi.h>
#include <stm32l4xx_hal_gpio.h>
#include <tgmath.h>
/************************ACCELEROMETER******************************/

void acc_write(uint8_t reg, uint8_t data) {
  uint8_t tx[2] = {reg, data};
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, tx, 2, 1000);
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);
}

void acc_init(void) {
  acc_write(CTRL_REG1_A, CTRL_REG1_A_DATA);
  acc_write(CTRL_REG2_A, CTRL_REG2_A_DATA);
  acc_write(CTRL_REG3_A, CTRL_REG3_A_DATA);
  acc_write(CTRL_REG4_A, CTRL_REG4_A_DATA);
  acc_write(CTRL_REG5_A, CTRL_REG5_A_DATA);
  acc_write(CTRL_REG6_A, CTRL_REG6_A_DATA);
  acc_write(CTRL_REG7_A, CTRL_REG7_A_DATA);
}
uint8_t acc_read(uint8_t reg) {
  uint8_t tx = reg | 0x80; // Read bit
  uint8_t rx = 0;
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &tx, 1, 1000);  // Wyślij rejestr do odczytu
  HAL_SPI_Receive(&hspi2, &rx, 1, 1000);   // Odczytaj wartość
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);

  return rx;
}

void acc_read_data(FullProcessedData *acc_full_data) {
  uint8_t rx[6]= {0};
  rx[0] = acc_read(OUT_X_L_A);
  rx[1] = acc_read(OUT_X_H_A);
  rx[2] = acc_read(OUT_Y_L_A);
  rx[3] = acc_read(OUT_Y_H_A);
  rx[4] = acc_read(OUT_Z_L_A);
  rx[5] = acc_read(OUT_Z_H_A);
  acc_full_data->x = ((float)((int16_t)(rx[1] << 8) | rx[0]) * 0.122f);
  acc_full_data->y = ((float)((int16_t)(rx[3] << 8) | rx[2]) * 0.122f);
  acc_full_data->z = ((float)((int16_t)(rx[5] << 8) | rx[4]) * 0.122f);
}




void acc_read_raw_data(RawData *acc_raw_data) {
  uint8_t rx[6] = {0};
  rx[0] = acc_read(OUT_X_L_A);
  rx[1] = acc_read(OUT_X_H_A);
  rx[2] = acc_read(OUT_Y_L_A);
  rx[3] = acc_read(OUT_Y_H_A);
  rx[4] = acc_read(OUT_Z_L_A);
  rx[5] = acc_read(OUT_Z_H_A);
  // Przetwarzanie surowych danych
  acc_raw_data->x = (int16_t)((rx[1] << 8) | rx[0]);
  acc_raw_data->y = (int16_t)((rx[3] << 8) | rx[2]);
  acc_raw_data->z = (int16_t)((rx[5] << 8) | rx[4]);
}

/***********************MAGNETOMETER*****************************/
void mag_write(uint8_t reg, uint8_t data) {
  uint8_t tx[2] = {reg, data};
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, tx, 2, 1000);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
}

void mag_init(void) {
  mag_write(CTRL_REG1_M, CTRL_REG1_M_DATA);
  mag_write(CTRL_REG2_M, CTRL_REG2_M_DATA);
  mag_write(CTRL_REG3_M, CTRL_REG3_M_DATA);
  mag_write(CTRL_REG4_M, CTRL_REG4_M_DATA);
  mag_write(CTRL_REG5_M, CTRL_REG5_M_DATA);
}

uint8_t mag_read(uint8_t reg) {
  uint8_t tx = reg | 0x80; // Ustawienie bitu odczytu
  uint8_t rx = 0;
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
  // Transmisja adresu rejestru
  HAL_SPI_Transmit(&hspi2, &tx, 1, 1000);
  // Odbiór wartości
  HAL_SPI_Receive(&hspi2, &rx, 1, 1000);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
  return rx;
}

void mag_read_data(FullProcessedData *mag_full_data) {
  uint8_t rx[6];
  rx[0] = mag_read(OUT_X_L_M);
  rx[1] = mag_read(OUT_X_H_M);
  rx[2] = mag_read(OUT_Y_L_M);
  rx[3] = mag_read(OUT_Y_H_M);
  rx[4] = mag_read(OUT_Z_L_M);
  rx[5] = mag_read(OUT_Z_H_M);
  // Przetwarzanie danych
  mag_full_data->x = ((float)((int16_t)(rx[1] << 8) | rx[0]) * .58f);
  mag_full_data->y = ((float)((int16_t)(rx[3] << 8) | rx[2]) * .58f);
  mag_full_data->z = ((float)((int16_t)(rx[5] << 8) | rx[4]) * .58f);
}

// Funkcja do obliczania kierunku północy
float calculate_heading(float M_X, float M_Y) {
  float heading = atan2(M_Y, M_X) * 180.0 / M_PI;
  if (heading < 0) {
    heading += 360;  // Normalizacja do zakresu 0-360°
  }
  return heading;
}