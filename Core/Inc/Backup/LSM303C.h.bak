//
// Created by Mateusz Wojtaszek on 15/03/2025.
//

#ifndef LSM303C_H
#define LSM303C_H
#include <stdint.h>
#include <math.h>
# define M_PI  3.14159265358979323846;
#define WHO_AM_I_A 0b00001111
#define WHO_AM_I_M 0x0F
/*``````````````ACCELEROMETER``````````````*/
#define ACT_THIS_A 0x1E
#define ACT_DUR_A 0x1F

#define CTRL_REG1_A 0x20
#define CTRL_REG1_A_DATA 0b11000111

#define CTRL_REG2_A 0x21
#define CTRL_REG2_A_DATA 0x00

#define CTRL_REG3_A 0x22
#define CTRL_REG3_A_DATA 0x00

#define CTRL_REG4_A 0x23
#define CTRL_REG4_A_DATA 0b01100011

#define CTRL_REG5_A 0x24
#define CTRL_REG5_A_DATA 0x00

#define CTRL_REG6_A 0x25
#define CTRL_REG6_A_DATA 0x00


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
#define CTRL_REG1_M_DATA 0b11010000


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
#endif //LSM303C_H
