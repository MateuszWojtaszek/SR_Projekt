//
// Created by Mateusz Wojtaszek on 15/03/2025.
//

#ifndef LSM303C_H
#define LSM303C_H
#include <stdint.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define CALIBRATION_SAMPLES 1000;
#define WHO_AM_I_A 0b00001111
#define WHO_AM_I_M 0x0F
/*``````````````ACCELEROMETER``````````````*/
#define ACT_THIS_A 0x1E
#define ACT_DUR_A 0x1F
/**
 * @brief CTRL_REG1_A (0x20) - Accelerometer Control Register 1
 *
 * - HR (bit 7): High Resolution enable
 *     - 1: High resolution mode (zalecane dla dokładnych pomiarów)
 * - ODR[2:0] (bits 6-4): Output Data Rate selection
 *     - 011: 100 Hz (optymalne dla ręcznych ruchów)
 * - BDU (bit 3): Block Data Update
 *     - 1: Rejestry nie są aktualizowane do pełnego odczytu MSB i LSB (chroni przed niespójnością danych)
 * - ZEN, YEN, XEN (bits 2-0): Enable all axes
 */
#define CTRL_REG1_A 0x20
#define CTRL_REG1_A_DATA 0b10111111 // HR=1, ODR=100Hz, BDU=1, XYZ enable

/**
 * @brief CTRL_REG2_A (0x21) - Accelerometer Control Register 2
 *
 * - DFC[1:0] (bits 7-6): High-pass filter cutoff frequency selection
 *     - 00: Low-pass cutoff ODR/50 (~2 Hz dla 100 Hz) – kompromis między responsywnością a tłumieniem
 * - HPM[1:0] (bits 5-4): High-pass filter mode
 *     - 00: Normal mode
 * - FDS (bit 3): Filtered data selection
 *     - 0: Wyłączone (czytamy dane bez dodatkowego filtrowania HPF)
 */
#define CTRL_REG2_A 0x21
#define CTRL_REG2_A_DATA 0b00010000 // DFC=00 (ODR/50), HPM=00 (normal), FDS=0

/**
 * @brief CTRL_REG3_A (0x22) - Accelerometer Control Register 3
 *
 * - FIFO_EN, STOP_FTH: 0 (nie korzystamy z FIFO)
 * - INT_XL_DRDY (bit 0): Data-ready interrupt enable
 *     - 1: Generuje przerwanie po nowym pomiarze (opcjonalne jeśli korzystasz z przerwań)
 */
#define CTRL_REG3_A 0x22
#define CTRL_REG3_A_DATA 0b00000001 // Tylko INT_XL_DRDY włączone

/**
 * @brief CTRL_REG4_A (0x23) - Accelerometer Control Register 4
 *
 * - BW[2:1] (bits 7-6): Anti-aliasing filter bandwidth
 *     - 10: 100 Hz (dopasowany do ODR)
 * - FS[1:0] (bits 5-4): Full Scale selection
 *     - 10: ±4g (zalecane dla ruchu ręką)
 * - BW_SCALE_ODR (bit 3): 0 (nie skalujemy BW automatycznie)
 * - IF_ADD_INC (bit 2): 1 (automatyczny inkrement adresu przy wielobajtowych operacjach – przydatne)
 * - I2C_DISABLE, SIM: 0 (domyślnie, chyba że masz tylko SPI)
 */
#define CTRL_REG4_A 0x23
#define CTRL_REG4_A_DATA 0b10100011 // BW=100Hz, FS=±4g, IF_ADD_INC=1

/**
 * @brief CTRL_REG5_A (0x24) - Accelerometer Control Register 5
 *
 * - Wszystkie funkcje debugowania, resetu, decymacji, self-test wyłączone
 */
#define CTRL_REG5_A 0x24
#define CTRL_REG5_A_DATA 0x00

/**
 * @brief CTRL_REG6_A (0x25) - Accelerometer Control Register 6
 *
 * - BOOT (bit 7): 0 (normalna praca)
 */
#define CTRL_REG6_A 0x25
#define CTRL_REG6_A_DATA 0x00

/**
 * @brief CTRL_REG7_A (0x26) - Accelerometer Control Register 7
 *
 * - Wszystkie opcje przerwań pozostawione domyślnie (0)
 */
#define CTRL_REG7_A 0x26
#define CTRL_REG7_A_DATA 0x00

#define STATUS_REG_A 0x27
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

/*``````````````MAGNETOMETER``````````````*/
#define CTRL_REG1_M 0x20
#define CTRL_REG1_M_DATA 0b11010100


#define CTRL_REG2_M 0x21
#define CTRL_REG2_M_DATA 0b01100000


#define CTRL_REG3_M 0x22
#define CTRL_REG3_M_DATA 0b10000100


#define CTRL_REG4_M 0x23
#define CTRL_REG4_M_DATA 0b00001000


#define CTRL_REG5_M 0x24
#define CTRL_REG5_M_DATA 0x00


#define STATUS_REG_M 0x27
#define OUT_X_L_M 0x28
#define OUT_X_H_M 0x29
#define OUT_Y_L_M 0x2A
#define OUT_Y_H_M 0x2B
#define OUT_Z_L_M 0x2C
#define OUT_Z_H_M 0x2D
#define TEMP_L_M 0x2E
#define TEMP_H_M 0x2F


typedef struct {
    float x;
    float y;
    float z;

    float temperature_c;
} FullProcessedData;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} RawData;
typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
} Offsets;

void acc_init(void);
void acc_calibrate(void);
void acc_write(uint8_t reg, uint8_t data);
uint8_t acc_read(uint8_t reg);
void acc_read_data(FullProcessedData *acc_full_data);
void acc_read_raw_data(RawData *acc_raw_data);
void mag_init(void);
void mag_calibrate(void);
void mag_write(uint8_t reg, uint8_t data);
uint8_t mag_read(uint8_t reg);
void mag_read_data(FullProcessedData *mag_full_data);
float calculate_heading(float M_X, float M_Y);
void accel_selftest_calibrate(void);

#endif //LSM303C_H
