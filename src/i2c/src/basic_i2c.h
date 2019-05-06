#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#ifndef BASIC_I2C_H
#define BASIC_I2C_H

/* define path to i2c device */
#ifndef DEVICE_I2C
#define DEVICE_I2C "/dev/i2c-1"
#endif

#define delay(ms) usleep(ms * 1000)

extern int i2c_read_reg8(int fd, int addr, uint8_t reg, uint8_t *buf);
extern int i2c_write_reg8(int fd, int addr, uint8_t reg, uint8_t data);
extern int i2c_read_reg16(int fd, int addr, uint8_t reg, uint16_t *buf);
extern int i2c_write_reg16(int fd, int addr, uint8_t reg, uint16_t data);
extern int i2c_read(int fd, int addr, uint8_t base_reg, uint8_t *buf, size_t buf_sz);
extern int i2c_write(int fd, int addr, uint8_t base_reg, uint8_t *data, size_t data_sz);

#endif
