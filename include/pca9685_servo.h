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

int pca9685_servo_new(struct pca9685_servo *servo, struct pca9685 *pca, unsigned int pin, unsigned int us_min, unsigned int us_max);
int pca9685_servo_read_us(struct pca9685_servo *servo);
int pca9685_servo_write_us(struct pca9685_servo *servo, unsigned int us);
int pca9685_servo_read_deg(struct pca9685_servo *servo);
int pca9685_servo_write_deg(struct pca9685_servo *servo, unsigned int deg);
int pca9685_servo_deg_to_us(struct pca9685_servo *servo, unsigned int deg);
int pca9685_servo_us_to_deg(struct pca9685_servo *servo, unsigned int us);

#endif
