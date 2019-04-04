#include <stdlib.h>
#include <math.h>

#include "pca9685.h"
#include "pca9685_servo.h"

/* declare static functions */
static long map(long x, long in_min, long in_max, long out_min, long out_max); 
static int us_to_tick(int us, int hz);

/* servo_new: create and initalize a servo struct */
struct pca9685_servo *pca9685_servo_new(struct pca9685 *pca, int pin, int us_min, int us_max, int us)
{
	struct pca9685_servo *servo;

	servo = malloc(sizeof(*servo));
	if (servo == NULL)
		return NULL;

	/* ensure that the pca is valid */
	if (pca == NULL)
		goto error_free_servo;
	servo->pca = pca;

	/* validate value of pin, cap at min and max */
	if (pin > PCA9685_PIN_MAX)
		pin = PCA9685_PIN_MAX;
	if (pin < 0)
		pin = 0;
	servo->pin = pin;

	/* set servo min and max */
	if (us_min < 0)
		us_min = PCA9685_SERVO_MIN_PULSE_WIDTH;
	if (us_max < 0 || us_max == us_min)
		us_max = PCA9685_SERVO_MAX_PULSE_WIDTH;
	servo->us_min = us_min;
	servo->us_max = us_max;
	
	/* initalize the servo */
	if (pca9685_servo_write_us(servo, us) != us)
		goto error_free_servo;

	return servo;

error_free_servo:
	if (servo)
		free(servo);
	return NULL;
}

/* servo_write_us: write a microsecond value to a servo struct. returns value written to servo */
int pca9685_servo_write_us(struct pca9685_servo *servo, int us)
{
	/* sanity check */
	if (us < servo->us_min || us > servo->us_max || servo == NULL)
		return (servo->us = -1); /* ensure that us does get set */
	
	if (pca9685_pwm_write(servo->pca, servo->pin, 0, us_to_tick(us, servo->pca->freq)) < 0)
		return (servo->us = -1); /* ditto */

	servo->us = us;

	return servo->us;
}

/* pca9685_servo_deg_to_us: convert degrees to a mircoseconds value depending on the servo's min and max range */
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