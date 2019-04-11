#include <stdlib.h>
#include <math.h>
#include <errno.h>

#include "pca9685.h"
#include "pca9685_servo.h"

/* declare static functions */
static long map(long x, long in_min, long in_max, long out_min, long out_max); 
static int us_to_tick(int us, int hz);

/* pca9685_servo_new: create and initalize a servo struct */
int pca9685_servo_new(struct pca9685_servo *servo, struct pca9685 *pca, int pin, int us_min, int us_max)
{
	int check;

	/* check struct */
	if (servo == NULL) {
		errno = EINVAL;
		return -1;
	}

	/* set defaults */
	servo->pca = NULL;
        servo->us_min = -1;
        servo->us_max = -1;
	servo->pin = -1;
	servo->us = -1;

	/* check pca struct */
	if (pca == NULL) {
		errno = EINVAL;
		return -1;
	}

	/* check if pca struct is setup for pwm */
	if ((check = pca9685_pwm_check(pca)) < 0)
		return check;	

	/* validate value of pin and cap at min and max */
	if (pin > PCA9685_PIN_MAX)
		pin = PCA9685_PIN_MAX;
	if (pin < 0)
		pin = 0;

	/* check servo min and max */
	if (us_min < 0 || us_max < 0 || us_max == us_min)
		return -1;
	
	/* write validated values to members */
	servo->pca = pca;
	servo->pin = pin;
	servo->us_min = us_min;
	servo->us_max = us_max;

	return pin;
}

/* pca9685_servo_write_us: write a microsecond value to a servo struct. returns value written to servo */
int pca9685_servo_write_us(struct pca9685_servo *servo, int us)
{
	/* sanity check */
	if (servo == NULL || servo->pca == NULL || us < servo->us_min || us > servo->us_max)
		return (servo->us = -1); /* ensure that us does get set */
	
	if (pca9685_pwm_write(servo->pca, servo->pin, 0, us_to_tick(us, servo->pca->freq)) < 0)
		return (servo->us = -1); /* ditto */

	servo->us = us;

	return servo->us;
}

/* pca9685_servo_deg_to_us: convert degree value to a mircosecond value depending on the servo's min and max range */
int pca9685_servo_deg_to_us(struct pca9685_servo *servo, int deg)
{
	return map(deg, 0, 180, servo->us_min, servo->us_max);
}

/* static functions */

/* map: maps x from one range to another */
static long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* us_to_tick: converts mircroseconds to number of ticks the signal should be high for */
static int us_to_tick(int us, int hz)
{       
        return (int) round((float) us / ((1000000.0f / hz) / PCA9685_PWM_TICK_MAX));
}
