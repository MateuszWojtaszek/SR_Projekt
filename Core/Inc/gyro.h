#ifndef GYRO_H_
#define GYRO_H_

#include <stdint.h>
#define WHO_AM_I 0xFF

struct data {
  float x;
  float y;
  float z;
};

void gyro_init(void);
void gyro_write(uint8_t reg, uint8_t data);
#endif /* GYRO_H_ */
