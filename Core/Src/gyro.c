

#include <gyro.h>
#include <stdio.h>


#include <stdlib.h>


GyroOffsets gyro_offsets = {0};

void gyro_write(uint8_t reg, uint8_t data) {
  uint8_t tx[2] = {reg & 0x7F, data};
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
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

void gyro_selftest_calibrate(void)
{
    GyroFullProcessedData normal_data = {0};
    GyroFullProcessedData selftest_data = {0};
    int samples = GYRO_CALIBRATION_SAMPLES;

    float sum_normal_x = 0, sum_normal_y = 0, sum_normal_z = 0;
    float sum_selftest_x = 0, sum_selftest_y = 0, sum_selftest_z = 0;

    printf("Rozpoczynanie Self-Test kalibracji...\r\n");

    // 1. Pomiar w normalnym trybie (spoczynkowym)
    for (int i = 0; i < samples; i++)
    {
        gyro_read_data(&normal_data);
        sum_normal_x += normal_data.x_dps;
        sum_normal_y += normal_data.y_dps;
        sum_normal_z += normal_data.z_dps;
        HAL_Delay(2);
    }

    float mean_normal_x = sum_normal_x / samples;
    float mean_normal_y = sum_normal_y / samples;
    float mean_normal_z = sum_normal_z / samples;

    // 2. Włącz Self-Test
    uint8_t reg4 = 0b10000001;
    reg4 = 0b10000011; // ST0 = 1
    gyro_write(CTRL_REG4, reg4);

    HAL_Delay(20); // Stabilizacja Self-Testu

    // 3. Pomiar w trybie Self-Test
    for (int i = 0; i < samples; i++)
    {
        gyro_read_data(&selftest_data);
        sum_selftest_x += selftest_data.x_dps;
        sum_selftest_y += selftest_data.y_dps;
        sum_selftest_z += selftest_data.z_dps;
        HAL_Delay(2);
    }

    float mean_selftest_x = sum_selftest_x / samples;
    float mean_selftest_y = sum_selftest_y / samples;
    float mean_selftest_z = sum_selftest_z / samples;

    // 4. Wyłącz Self-Test
    reg4 &= 0b10000001; // ST0 = 0
    gyro_write(CTRL_REG4, reg4);

    HAL_Delay(20); // Stabilizacja normalnego trybu

    // 5. Oblicz rzeczywiste zmiany z Self-Testu
    float delta_x = mean_selftest_x - mean_normal_x;
    float delta_y = mean_selftest_y - mean_normal_y;
    float delta_z = mean_selftest_z - mean_normal_z;

    // 6. Oblicz bias (przesunięcie zerowe w stanie spoczynku)
    gyro_offsets.offset_x = mean_normal_x;
    gyro_offsets.offset_y = mean_normal_y;
    gyro_offsets.offset_z = mean_normal_z;

    // 7. Wyświetl wyniki
    printf("Self-Test zakończony.\r\n");
    printf("Bias X: %.4f dps\r\n", gyro_offsets.offset_x);
    printf("Bias Y: %.4f dps\r\n", gyro_offsets.offset_y);
    printf("Bias Z: %.4f dps\r\n", gyro_offsets.offset_z);

    printf("Self-Test zmiana X: %.2f dps (oczekiwane ~130)\r\n", delta_x);
    printf("Self-Test zmiana Y: %.2f dps (oczekiwane ~130)\r\n", delta_y);
    printf("Self-Test zmiana Z: %.2f dps (oczekiwane ~130)\r\n", delta_z);
}

void gyro_read_data(GyroFullProcessedData *gyro_full_data) {
  uint8_t rx[6];
  rx[0] = gyro_read(OUT_X_L);
  rx[1] = gyro_read(OUT_X_H);
  rx[2] = gyro_read(OUT_Y_L);
  rx[3] = gyro_read(OUT_Y_H);
  rx[4] = gyro_read(OUT_Z_L);
  rx[5] = gyro_read(OUT_Z_H);
  uint8_t temp = gyro_read(OUT_TEMP);
  gyro_full_data->x_dps = ((float)((int16_t)((rx[1] << 8) | rx[0])) * 0.00875f) - gyro_offsets.offset_x;
  gyro_full_data->y_dps = ((float)((int16_t)((rx[3] << 8) | rx[2])) * 0.00875f) - gyro_offsets.offset_y;
  gyro_full_data->z_dps = ((float)((int16_t)((rx[5] << 8) | rx[4])) * 0.00875f) - gyro_offsets.offset_z;
  gyro_full_data->temperature_c = (int8_t)temp;
}
