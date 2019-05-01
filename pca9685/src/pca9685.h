#include <stdint.h>
#include <unistd.h>

#ifndef PCA9685_H
#define PCA9685_H

/* define pwm constants */
#define PCA9685_PWM_MIN 40
#define PCA9685_PWM_MAX 1000
#define PCA9685_PWM_TICK_MAX 4096
#define PCA9685_PWM_OSC_CLK 25000000.0

/* define registers */
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

/* define register for first LED (pwm device) and all LEDs */
#define PCA9685_LED0_ON_L 0x6
#define PCA9685_LEDALL_ON_L 0xFA
#define PCA9685_PIN_MIN 0
#define PCA9685_PIN_MAX 15
#define PCA9685_PIN_ALL (PCA9685_PIN_MAX + 1)

/* define constants used for calculating states */
#define PCA9685_AUTOINC 0x20
#define PCA9685_SLEEP 0x10
#define PCA9685_WAKE 0xEF
#define PCA9685_RESTART 0x80

/* define pca9685 struct */
struct pca9685 {
	int fd;
	unsigned int addr;
	unsigned int freq;
	uint8_t settings;
	uint8_t sleep;
	uint8_t wake;
	uint8_t restart;
};

/* declare functions and useful macros */
#define delay(ms) usleep(ms * 1000)

extern int pca9685_new(struct pca9685 *pca, int fd, unsigned int addr);
extern int pca9685_pwm_init(struct pca9685 *pca, unsigned int freq);
extern int pca9685_pwm_reset(struct pca9685 *pca);
extern int pca9685_pwm_write(struct pca9685 *pca, int pin, uint16_t on, uint16_t off);
extern int pca9685_pwm_read(struct pca9685 *pca, int pin, uint16_t *on, uint16_t *off);
extern int pca9685_pwm_full_on(struct pca9685 *pca, int pin, int tf);
extern int pca9685_pwm_full_off(struct pca9685 *pca, int pin, int tf);
extern int pca9685_pwm_check(struct pca9685 *pca);

#endif
