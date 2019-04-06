#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "basic_i2c.h"

/* i2c_read_reg8: read a byte from a 8-bit device register */
int i2c_read_reg8(int fd, uint8_t reg, uint8_t *buf)
{
	int reg_sz = sizeof(reg);
	int buf_sz = sizeof(*buf);

	if (fd < 0)
		return fd;

	if (write(fd, &reg, reg_sz) != reg_sz)
		return -1;

	if (read(fd, buf, buf_sz) != buf_sz)
		return -1;

	return buf_sz;
}

/* i2c_write_reg8: write a byte to a 8-bit device register */
int i2c_write_reg8(int fd, uint8_t reg, uint8_t data)
{
	uint8_t buf[2];
	int sz = sizeof(reg) + sizeof(data);

	if (fd < 0)
		return fd;

	buf[0] = reg;
	buf[1] = data;

	if (write(fd, buf, sz) != sz)
		return -1;

	return sizeof(data);
}

/* i2c_read_reg16: read a 16-bit int from a 8-bit device register (auto increment must be enabled) */
int i2c_read_reg16(int fd, uint8_t reg, uint16_t *buf)
{
	int reg_sz = sizeof(reg);
	int buf_sz = sizeof(*buf);

	if (fd < 0)
		return fd;

	if (write(fd, &reg, reg_sz) != reg_sz)
		return -1;

	if (read(fd, buf, buf_sz) != buf_sz)
		return -1;

	return buf_sz;
}

/* i2c_write_reg16: write a 16-bit int to a 8-bit device register (auto increment must be enabled) */
int i2c_write_reg16(int fd, uint8_t reg, uint16_t data)
{
	uint8_t buf[3];
	int sz = sizeof(reg) + sizeof(data);

	if (fd < 0)
		return fd;

	buf[0] = reg;
	buf[1] = (uint8_t) (data >> 8);
	buf[2] = (uint8_t) (data & 0xFF);

	if (write(fd, buf, sz) != sz)
		return -1;

	return sizeof(data);
}