#include <stdint.h>
#include <stdlib.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "pca9685.h"
#include "basic_i2c.h"

/* declare static functions */
static int pca9685_get_base_reg(int pin);
static int pca9685_update_settings(struct pca9685 *pca, int tf);
static int pca9685_pwm_check(struct pca9685 *pca);
static void pca9685_enable_autoinc(struct pca9685 *pca);

/* pca9685_new: create and setup a new pca9685 struct */
struct pca9685 *pca9685_new(int addr, float freq)
{
	struct pca9685 *pca;
	int fd;

	pca = malloc(sizeof(*pca));
	if (pca == NULL)
		return NULL;

	/* open i2c device */
	fd = open(DEVICE_I2C, O_RDWR);
	if (fd < 0)
		goto error_free_pca;
	pca->fd = fd;

	/* choose slave referenced by addr */
	if (ioctl(pca->fd, I2C_SLAVE, addr) < 0)
		goto error_free_pca;
	pca->addr = addr;

	pca9685_enable_autoinc(pca);

	/* set pwm frequency and end sleep mode */
	if (freq > 0) {
		if (pca9685_pwm_freq(pca, freq) < 0)
			pca->freq = 0; /* error value */
		else 
			pca->freq = freq;	
	} else {
		pca->freq = 0;
	}

	return pca;

error_free_pca:
	if (pca)
		free(pca);
	return NULL;
}

/* pca9685_pwm_freq: set a frequency for pwm signals */
int pca9685_pwm_freq(struct pca9685 *pca, float freq)
{
	int prescale;

	/* cap frequency at min and max */ 
	freq = (freq > PCA9685_PWM_MAX ? PCA9685_PWM_MAX : (freq < PCA9685_PWM_MIN ? PCA9685_PWM_MIN : freq));

	/* To set pwm frequency we have to set the prescale register. The formula is:
	 * prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
	 * Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
	 */
	prescale = (int)(25000000.0f / (PCA9685_PWM_TICK_MAX * freq) - 0.5f);

	/* calculate states */
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
int pca9685_pwm_write(struct pca9685 *pca, int pin, int on, int off)
{
	int reg = pca9685_get_base_reg(pin);

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
int pca9685_pwm_read(struct pca9685 *pca, int pin, int16_t *on, int16_t *off)
{
	int reg = pca9685_get_base_reg(pin);

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
	int reg = pca9685_get_base_reg(pin) + 1; /* LEDX_ON_H */
	int8_t state;

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
	int reg = pca9685_get_base_reg(pin) + 3; /* LEDX_OFF_H */
	int8_t state;

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
static int pca9685_get_base_reg(int pin)
{
	return (pin >= PCA9685_PIN_ALL) ? PCA9685_LEDALL_ON_L : PCA9685_LED0_ON_L + 4 * pin;
}

/* pca9685_update_settings: update settings member of pca9685 struct. if tf then calculate states */
static int pca9685_update_settings(struct pca9685 *pca, int tf)
{
	if (i2c_read_reg8(pca->fd, PCA9685_MODE1, (int8_t*) &(pca->settings)) < 0)
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
	int autoinc;

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
