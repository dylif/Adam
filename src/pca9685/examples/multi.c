#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "../pca9685.h"
#include "../basic_i2c.h"
#include "../pca9685_servo.h"

#define SERVO_MIN 700
#define SERVO_MAX 2500
#define SERVO_NEU 1600

#define ADDR 0x40
#define HZ 50

int us_to_tick(int us, int hz)
{       
        return (int) round((float) us / ((1000000.0f / hz) / PCA9685_PWM_TICK_MAX));
}

int main(void)
{
	struct pca9685 pca;
	struct pca9685_servo servo0;
	struct pca9685_servo servo1;
	
	int servo_count = 2;

	int fd = open(DEVICE_I2C, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "error opening i2c\n");
		return -1;
	}

	if (pca9685_new(&pca, fd, ADDR) < 0) {
		fprintf(stderr, "error creating pca: %s\n", strerror(errno));
		return -1;
	}
	
	if (pca9685_pwm_init(&pca, HZ) < 0) {
		fprintf(stderr, "error pwm init: %s\n", strerror(errno));
		return -1;
	}

	if (pca9685_servo_new(&servo0, &pca, 0, SERVO_MIN, SERVO_MAX) < 0) {
		fprintf(stderr, "error servo0 init: %s\n", strerror(errno));
		return -1;
	}
	if (pca9685_servo_new(&servo1, &pca, 1, SERVO_MIN, SERVO_MAX) < 0) {
		fprintf(stderr, "error servo1 init: %s\n", strerror(errno));
		return -1;
	}
	
	/* centre all servos */
	pca9685_pwm_write(&pca, 16, 0, us_to_tick(SERVO_NEU, HZ));
	
	for (;;) {
		int angle0 = 0;
		int angle1 = 0; 
		printf("Please enter servo0 angle:\n");
		scanf("%d", &angle0);
		printf("Please enter servo1 angle:\n");
		scanf("%d", &angle1);
		
		pca9685_servo_write_us(&servo0, pca9685_servo_deg_to_us(&servo0, angle0));
		pca9685_servo_write_us(&servo1, pca9685_servo_deg_to_us(&servo1, angle1));
	}
	pca9685_pwm_reset(&pca);

	return 0;
}
