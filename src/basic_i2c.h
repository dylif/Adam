#include <stdint.h>

#ifndef BASIC_I2C_H
#define BASIC_I2C_H

/* define path to i2c device */
#ifndef DEVICE_I2C
#define DEVICE_I2C "/dev/i2c-1"
#endif

extern int i2c_read_reg8(int fd, int8_t reg, int8_t *buf);
extern int i2c_write_reg8(int fd, int8_t reg, int8_t data);
extern int i2c_read_reg16(int fd, int8_t reg, int16_t *buf);
extern int i2c_write_reg16(int fd, int8_t reg, int16_t data);

#endif