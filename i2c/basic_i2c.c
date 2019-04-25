#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "basic_i2c.h"

/* i2c_read_reg8: read a byte from a 8-bit device register */
int i2c_read_reg8(int fd, int addr, uint8_t reg, uint8_t *buf)
{
	int reg_sz = sizeof(reg);
	int buf_sz = sizeof(*buf);

	if (fd < 0)
		return fd;

	if (write(fd, &reg, reg_sz) != reg_sz)
		return errno * -1;

	if (read(fd, buf, buf_sz) != buf_sz)
		return errno * -1;

	return buf_sz;
}

/* i2c_write_reg8: write a byte to a 8-bit device register */
int i2c_write_reg8(int fd, int addr, uint8_t reg, uint8_t data)
{
	uint8_t buf[2];
	int sz = sizeof(buf) / sizeof(buf[0]);

	if (fd < 0)
		return fd;

	buf[0] = reg;
	buf[1] = data;

	if (write(fd, buf, sz) != sz)
		return errno * -1;

	return sz;
}

/* i2c_read_reg16: read a 16-bit int from a 8-bit device register */
int i2c_read_reg16(int fd, int addr, uint8_t reg, uint16_t *buf)
{
	/* 8 bit register, 16 bit buffer */
	int reg_sz = 1;
	int buf_sz = 2;
	
	uint8_t cur_reg;

	if (fd < 0)
		return fd;

	int i;
	for (i = 0; i < 2; ++i) {
		cur_reg = reg + i;
		if (write(fd, &cur_reg, reg_sz) != reg_sz)
			return  errno * -1;
		if (read(fd, buf, buf_sz) != buf_sz)
			return errno * -1;
	}
	
	return buf_sz;
}

/* i2c_write_reg16: write a 16-bit int to a 8-bit device register */
int i2c_write_reg16(int fd, int addr, uint8_t reg, uint16_t data)
{
	uint8_t buf[4];
	int sz = sizeof(buf) / sizeof(buf[0]);

	if (fd < 0)
		return fd;

	buf[0] = reg;
	buf[1] = (uint8_t) (data & 0xFF);
	buf[2] = reg + 1;
	buf[3] = (uint8_t) (data >> 8);
	

	if (write(fd, buf, sz) != sz)
		return errno * -1;

	return sz;
}
