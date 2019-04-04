#include <stdint.h>
#include <unistd.h>

#ifndef PCA9685_H
#define PCA9685_H

/* define pwm constants */
#define PCA9685_PWM_MIN 40
#define PCA9685_PWM_MAX 1000
#define PCA9685_PWM_TICK_MAX 4096

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
	int addr;
	float freq;
	int settings;
	int autoinc : 1;
	int sleep;
	int wake;
	int restart;
};

/* declare functions and useful macros */
#define delay(ms) usleep(ms * 1000)

extern struct pca9685 *pca9685_new(int addr, float freq);
extern int pca9685_pwm_freq(struct pca9685 *pca, float freq);
extern int pca9685_pwm_reset(struct pca9685 *pca);
extern int pca9685_pwm_write(struct pca9685 *pca, int pin, int on, int off);
extern int pca9685_pwm_read(struct pca9685 *pca, int pin, int16_t *on, int16_t *off);
extern int pca9685_pwm_full_on(struct pca9685 *pca, int pin, int tf);
extern int pca9685_pwm_full_off(struct pca9685 *pca, int pin, int tf);

#endif
