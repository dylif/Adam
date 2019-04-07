#include "pca9685.h"

#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

#define PCA9685_SERVO_MIN_PULSE_WIDTH 544
#define PCA9685_SERVO_MAX_PULSE_WIDTH 2400

struct pca9685_servo {
        struct pca9685 *pca;
        int us_min;
        int us_max;
	int pin;
	int us;
};

extern int pca9685_servo_new(struct pca9685_servo *servo, struct pca9685 *pca, int pin, int us_min, int us_max);
extern int pca9685_servo_write_us(struct pca9685_servo *servo, int us);
extern int pca9685_servo_deg_to_us(struct pca9685_servo *servo, int deg);

#endif