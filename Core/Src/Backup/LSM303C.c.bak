//
// Created by Mateusz Wojtaszek on 15/03/2025.
//
#include "LSM303C.h"
#include <spi.h>
#include <stdio.h>
#include <stm32l4xx_hal_gpio.h>
#include <tgmath.h>
/************************ACCELEROMETER******************************/

Offsets acc_offsets = {0};

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
  uint8_t id = acc_read(WHO_AM_I_A);
  if (id != 0x41) {
    printf("ACC: Wrong WHO_AM_I ID: 0x%02X\r\n", id);
  } else {
    printf("ACC: Device OK, WHO_AM_I = 0x%02X\r\n", id);
  }
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
  acc_full_data->x = ((float)((int16_t)(rx[1] << 8) | rx[0]) * 0.122f) - acc_offsets.offset_x;
  acc_full_data->y = ((float)((int16_t)(rx[3] << 8) | rx[2]) * 0.122f) - acc_offsets.offset_y;
  acc_full_data->z = ((float)((int16_t)(rx[5] << 8) | rx[4]) * 0.122f) - acc_offsets.offset_z;
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

void accel_selftest_calibrate(void)
{
    FullProcessedData normal_data = {0};
    FullProcessedData selftest_data = {0};
    int samples = CALIBRATION_SAMPLES;

    float sum_normal_x = 0, sum_normal_y = 0, sum_normal_z = 0;
    float sum_selftest_x = 0, sum_selftest_y = 0, sum_selftest_z = 0;

    printf("Rozpoczynanie Self-Test kalibracji...\r\n");

    // 1. Pomiar w normalnym trybie (spoczynkowym)
    for (int i = 0; i < samples; i++)
    {
        acc_read_data(&normal_data);
        sum_normal_x += normal_data.x;
        sum_normal_y += normal_data.y;
        sum_normal_z += normal_data.z;
        HAL_Delay(2);
    }

    float mean_normal_x = sum_normal_x / samples;
    float mean_normal_y = sum_normal_y / samples;
    float mean_normal_z = sum_normal_z / samples;

    // 2. Włącz Self-Test
    uint8_t reg5 = 0b00000100; // ST0 = 1
    acc_write(CTRL_REG5_A, reg5);

    HAL_Delay(200); // Stabilizacja Self-Testu

    // 3. Pomiar w trybie Self-Test
    for (int i = 0; i < samples; i++)
    {
        acc_read_data(&selftest_data);
        sum_selftest_x += selftest_data.x;
        sum_selftest_y += selftest_data.y;
        sum_selftest_z += selftest_data.z;
        HAL_Delay(20);
    }

    float mean_selftest_x = sum_selftest_x / samples;
    float mean_selftest_y = sum_selftest_y / samples;
    float mean_selftest_z = sum_selftest_z / samples;

    // 4. Wyłącz Self-Test
    reg5 = 0b00000000; // ST0 = 0
    acc_write(CTRL_REG5_A, reg5);

    HAL_Delay(200); // Stabilizacja normalnego trybu

    // 5. Oblicz rzeczywiste zmiany z Self-Testu
    float delta_x = mean_selftest_x - mean_normal_x;
    float delta_y = mean_selftest_y - mean_normal_y;
    float delta_z = mean_selftest_z - mean_normal_z;

  // 6. Oblicz bias (przesunięcie zerowe w stanie spoczynku)
  acc_offsets.offset_x = mean_normal_x;
  acc_offsets.offset_y = mean_normal_y;
  acc_offsets.offset_z = mean_normal_z - 1000.0f; // Korekta na grawitację!

  // 7. Wyświetl wyniki
  printf("Self-Test zakończony.\r\n");
  printf("Bias X: %.4f mg\r\n", acc_offsets.offset_x);
  printf("Bias Y: %.4f mg\r\n", acc_offsets.offset_y);
  printf("Bias Z: %.4f mg (po odjęciu 1g)\r\n", acc_offsets.offset_z);

  printf("Self-Test zmiana X: %.2f mg (oczekiwane ok. 70 mg)\r\n", delta_x);
  printf("Self-Test zmiana Y: %.2f mg (oczekiwane ok. 70 mg)\r\n", delta_y);
  printf("Self-Test zmiana Z: %.2f mg (oczekiwane ok. 70 mg)\r\n", delta_z);
}