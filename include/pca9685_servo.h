#include "pca9685.h"

#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

struct pca9685_servo {
        struct pca9685 *pca;
        unsigned int us_min;
        unsigned int us_max;
	int pin;
	int us;
};

extern int pca9685_servo_new(struct pca9685_servo *servo, struct pca9685 *pca, unsigned int pin, unsigned int us_min, unsigned int us_max);
extern int pca9685_servo_write_us(struct pca9685_servo *servo, unsigned int us);
extern int pca9685_servo_deg_to_us(struct pca9685_servo *servo, unsigned int deg);

#endif
