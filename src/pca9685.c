#include <stdint.h>
#include <stdlib.h>

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "pca9685.h"
#include "basic_i2c.h"

/* declare static functions */
static uint8_t pca9685_get_base_reg(int pin);
static int pca9685_update_settings(struct pca9685 *pca, int tf);
static int pca9685_pwm_check(struct pca9685 *pca);
static void pca9685_enable_autoinc(struct pca9685 *pca);

/* pca9685_new: setup a new pca9685 struct (select the address on the i2c device) */
int pca9685_new(struct pca9685 *pca, int fd, unsigned int addr)
{
	/* check struct */
	if (pca == NULL) {
		errno = EINVAL;
		return -1;
	}

	/* check i2c file */
	if (fd < 0) {
		errno = EBADFD;
		return -1;
	}	

	/* choose i2c slave addr */
	if (ioctl(fd, I2C_SLAVE, addr) < 0)
		return -1;

	pca->fd = fd;
	pca->addr = addr;

	/* set defaults */
	pca->freq = 0;
	pca->autoinc = 0;
	pca->settings = 0;
	pca->sleep = 0;
	pca->wake = 0;
	pca->restart = 0;

	return fd;
}

/* pca9685_pwm_init: set a frequency for pwm signals and enable auto increment of device registers */
int pca9685_pwm_init(struct pca9685 *pca, float freq)
{
	uint8_t prescale;

	/* check struct */
	if (pca == NULL) {
		errno = EINVAL;
		return -1;
	}

	/* cap frequency at min and max */ 
	if (freq < PCA9685_PWM_MIN || freq > PCA9685_PWM_MAX) {
		errno  = EINVAL;
		return -1;
	}

	/* To set pwm frequency we have to set the prescale register. The formula is:
	 * prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
	 * Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
	 */
	prescale = (uint8_t) (25000000.0f / (PCA9685_PWM_TICK_MAX * freq) - 0.5f);

	/* update settings and calculate states */
	if (pca9685_update_settings(pca, 1) < 0)
		return -1;

	/* sleep, set prescale and wake */
	if (i2c_write_reg8(pca->fd, PCA9685_MODE1, pca->sleep) < 0)
		return -1;
	if (i2c_write_reg8(pca->fd, PCA9685_PRESCALE, prescale) < 0)
		return -1;
	if (i2c_write_reg8(pca->fd, PCA9685_MODE1, pca->wake) < 0)
		return -1;

	/* wait a millisecond for oscillator to finish stabilizing and restart pwm */
	delay(1);
	if (i2c_write_reg8(pca->fd, PCA9685_MODE1, pca->restart) < 0)
		return -1;

	pca->freq = freq;

	/* enable auto increment */
	pca9685_enable_autoinc(pca);

	return pca->freq;
}

/* pca9685_pwm_reset: reset all pwm output */
int pca9685_pwm_reset(struct pca9685 *pca)
{
	/* sanity check */
	if (pca9685_pwm_check(pca) < 0)
		return -1;

	if (i2c_write_reg16(pca->fd, PCA9685_LEDALL_ON_L, 0x0) < 0)
		return -1;
	if (i2c_write_reg16(pca->fd, PCA9685_LEDALL_ON_L + 2, 0x1000) < 0)
		return -1;

	return 0;
}

/* pca9685_pwm_write: write on and off ticks to a pin, disabling any full on or off */
int pca9685_pwm_write(struct pca9685 *pca, int pin, uint16_t on, uint16_t off)
{
	uint8_t reg = pca9685_get_base_reg(pin);

	/* sanity check */
	if (pca9685_pwm_check(pca) < 0)
		return -1;

	/* write to on and off registers and mask the 12 lowest bits of data to overwrite full on and off */
	if (i2c_write_reg16(pca->fd, reg, on & 0x0FFF) < 0)
		return -1; 
	if (i2c_write_reg16(pca->fd, reg + 2, off & 0x0FFF) < 0)
		return -1;

	return on + off;
}

/* pca9685_pwm_read: read both on and off registers as 16-bit data
 * PWM: mask each value with 0xFFF
 * full-on or off bit: mask with 0x1000
 * note: PCA9685_PIN_ALL will always return 0
 */
int pca9685_pwm_read(struct pca9685 *pca, int pin, uint16_t *on, uint16_t *off)
{
	uint8_t reg = pca9685_get_base_reg(pin);

	/* sanity check */
	if (pca9685_pwm_check(pca) < 0)
		return -1;

	if (on != NULL) {
		if (i2c_read_reg16(pca->fd, reg, on) < 0)
			return -1;
	}
	if (off != NULL) {
		if (i2c_read_reg16(pca->fd, reg + 2, off) < 0)
			return -1;
	}

	return (*on) + (*off);
}

/* pca9685_pwm_full_on: enables or dsiables full-on
 * tf = true: full-on
 * tf = false: according to PWM
 */
int pca9685_pwm_full_on(struct pca9685 *pca, int pin, int tf)
{
	uint8_t reg = pca9685_get_base_reg(pin) + 1; /* LEDX_ON_H */
	uint8_t state;

	/* sanity check */
	if (pca9685_pwm_check(pca) < 0)
		return -1;
	
	if (i2c_read_reg8(pca->fd, reg, &state) < 0)
		return -1;

	/* set bit 4 to 1 or 0 according to tf */
	state = tf ? (state | 0x10) : (state & 0xEF);

	if (i2c_write_reg8(pca->fd, reg, state) < 0)
		return -1;

	/* set full-off to 0 because it has priority over full-on */
	if (tf) {
		if (pca9685_pwm_full_off(pca, pin, 0) != pin) {
			return -1;
		}
	}

	return pin;
}

/* pca9685_pwm_full_off: enables or dsiables full-off
 * tf = true: full-off
 * tf = false: according to PWM or full-on
 */
int pca9685_pwm_full_off(struct pca9685 *pca, int pin, int tf)
{
	uint8_t reg = pca9685_get_base_reg(pin) + 3; /* LEDX_OFF_H */
	uint8_t state;
	
	/* sanity check */
	if (pca9685_pwm_check(pca) < 0)
		return -1;
	
	if (i2c_read_reg8(pca->fd, reg, &state) < 0)
		return -1;

	/* set bit 4 to 1 or 0 according to tf */
	state = tf ? (state | 0x10) : (state & 0xEF);

	if (i2c_write_reg8(pca->fd, reg, state) < 0)
		return -1;
	
	return pin;
}

/* static functions */

/* pca9685_get_base_reg: calculate the register for LED on pin */
static uint8_t pca9685_get_base_reg(int pin)
{
	return (pin >= PCA9685_PIN_ALL) ? PCA9685_LEDALL_ON_L : PCA9685_LED0_ON_L + 4 * pin;
}

/* pca9685_update_settings: update settings member of pca9685 struct. if tf then calculate states */
static int pca9685_update_settings(struct pca9685 *pca, int tf)
{
	if (i2c_read_reg8(pca->fd, PCA9685_MODE1, &(pca->settings)) < 0)
		return -1;

	pca->settings &= 0x7F; /* set restart bit to 0 */

	if (tf) {
		pca->sleep = pca->settings | PCA9685_SLEEP; /* set sleep bit to 1 */
		pca->wake = pca->settings & PCA9685_WAKE; /* set sleep bit to 0 */
		pca->restart = pca->wake | PCA9685_RESTART; /* set restart bit to 1 */
	}

	return 0;
}

/* pca9685_pwm_check: check if pwm settings are valid */
static int pca9685_pwm_check(struct pca9685 *pca)
{
	/* check struct */
	if (pca == NULL) {
		errno = EINVAL;
		return -1;
	}
	
	/* must have register auto increment enabled for writing more than one byte to a register */
	if (!pca->autoinc)
		return -1;

	if (pca->freq < PCA9685_PWM_MIN || pca->freq > PCA9685_PWM_MAX)
		return pca->freq * -1;

	return 0;
}

/* pca9685_enable_autoinc: enable autoinc setting. sets autoinc member accordingly */
static void pca9685_enable_autoinc(struct pca9685 *pca)
{
	uint8_t autoinc;

	/* get settings */
	if (pca9685_update_settings(pca, 0) < 0) {
		pca->autoinc = 0;
		return;
	}

	/* enable auto increment of device registers */
	autoinc = pca->settings | PCA9685_AUTOINC;
	if (i2c_write_reg8(pca->fd, PCA9685_MODE1, autoinc) < 0)
		pca->autoinc = 0;
	else
		pca->autoinc = 1;
}
