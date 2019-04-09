#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "pca9685.h"
#include "basic_i2c.h"
#include "pca9685_servo.h"

#define SERVO_MIN 644
#define SERVO_MAX 2400

int main(void)
{
	struct pca9685 pca;
	struct pca9685_servo servo0;

	int fd = open(DEVICE_I2C, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Error opening i2c\n");
		return -1;
	}

	if (pca9685_new(&pca, fd, 0x40) < 0) {
		fprintf(stderr, "error creating pca: %s\n", strerror(errno));
		return -1;
	}
	
	if (pca9685_pwm_init(&pca, 50) < 0) {
		fprintf(stderr, "error pwm init: %s\n", strerror(errno));
		return -1;
	}

	if (pca9685_servo_new(&servo0, &pca, 0, 490, 2400) < 0) {
		fprintf(stderr, "error servo init: %s\n", strerror(errno));
		return -1;
	}

	printf("center servo0 in 5 sec: %d\n", pca9685_servo_write_us(&servo0, 1500));
	delay(5000);
	printf("servo0: %d\n", pca9685_servo_write_us(&servo0, 634));
	delay(3000);
	printf("servo0: %d\n", pca9685_servo_write_us(&servo0, 2400));
	delay(3000);

	pca9685_pwm_reset(&pca);

	return 0;
}
