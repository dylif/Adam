#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "basic_i2c.h"

/* i2c_read_reg8: read a byte from a 8-bit device register */
int i2c_read_reg8(int fd, int addr, uint8_t reg, uint8_t *buf)
{
	int status; 
	
	int reg_sz = sizeof(reg);
	int buf_sz = sizeof(*buf);

	if (fd < 0)
		return fd;
		
	if ((status = ioctl(fd, I2C_SLAVE, addr)) < 0)
		return status;

	if (write(fd, &reg, reg_sz) != reg_sz)
		return -1;

	if (read(fd, buf, buf_sz) != buf_sz)
		return -1;

	return buf_sz;
}

/* i2c_write_reg8: write a byte to a 8-bit device register */
int i2c_write_reg8(int fd, int addr, uint8_t reg, uint8_t data)
{
	int status;
	
	uint8_t buf[2];
	int buf_sz = sizeof(buf) / sizeof(buf[0]);

	if (fd < 0)
		return fd;
		
	if ((status = ioctl(fd, I2C_SLAVE, addr)) < 0)
		return status;

	buf[0] = reg;
	buf[1] = data;

	if (write(fd, buf, buf_sz) != buf_sz)
		return -1;

	return buf_sz;
}

/* i2c_read_reg16: read a 16-bit int from a 8-bit device register */
int i2c_read_reg16(int fd, int addr, uint8_t reg, uint16_t *buf)
{
	int status;
	
	uint8_t byte_buf[2];
	int byte_buf_sz = sizeof(byte_buf) / sizeof(byte_buf[0]);
	
	if ((status = i2c_read(fd, addr, reg, byte_buf, byte_buf_sz)) < 0)
		return status;

	*buf = (((uint16_t) byte_buf[0]) << 8) | ((uint16_t) byte_buf[1]);
	
	return byte_buf_sz;
}

/* i2c_write_reg16: write a 16-bit int to a 8-bit device register */
int i2c_write_reg16(int fd, int addr, uint8_t reg, uint16_t data)
{
	int status; 
	
	uint8_t buf[2];
	int buf_sz = sizeof(buf) / sizeof(buf[0]);
	
	buf[0] = (uint8_t) (data & 0xFF);
	buf[1] = (uint8_t) (data >> 8);

	if ((status = i2c_write(fd, addr, reg, buf, buf_sz)) < 0)
		return status;

	return buf_sz;
}

/* i2c_read: Read multiple consecutive bytes from a i2c device. Reads no more than buf_sz bytes */
int i2c_read(int fd, int addr, uint8_t base_reg, uint8_t *buf, size_t buf_sz)
{
	int status;
	
	int i;
	for (i = 0; i < buf_sz; ++i) {
		if ((status = i2c_read_reg8(fd, addr, base_reg + i, buf + i)) < 0)
			return status;
	}
	
	return i;
}

/* i2c_write: Write multiple consecutive bytes to a i2c device. Writes no more than data_sz bytes */
int i2c_write(int fd, int addr, uint8_t base_reg, uint8_t *data, size_t data_sz)
{
	int status;
	
	int i;
	for (i = 0; i < data_sz; ++i) {
		if ((status = i2c_write_reg8(fd, addr, base_reg + i, data[i])) < 0)
			return status;
	}
	
	return i;
}
