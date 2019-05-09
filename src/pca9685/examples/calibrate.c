#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <pca9685.h>
#include <basic_i2c.h>
#include <pca9685_servo.h>

#define ADDR 0x40
#define HZ 50

/* these are accepted vals for the HS-755HB model servo
#define SERVO_MIN 650
#define SERVO_MAX 2300
 */

/* see above but relative to 700us
#define SERVO_MIN 700
#define SERVO_MAX 2500
#define SERVO_NEU 1600
 */
 
int main(void)
{
	struct pca9685 pca;
	struct pca9685_servo servo;
	
	printf("PCA9685 servo calibration\n");
	printf("Use this to test out the min and max microsecond vals of your servo\n");


	int fd = open(DEVICE_I2C, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "error opening i2c device: %s\n", strerror(errno));
		return -1;
	}

	if (pca9685_new(&pca, fd, ADDR) < 0) {
		fprintf(stderr, "error in pca init: %s\n", strerror(errno));
		return -1;
	}
	
	if (pca9685_pwm_init(&pca, HZ) < 0) {
		fprintf(stderr, "error in pca pwm init: %s\n", strerror(errno));
		return -1;
	}
	
	for (;;) {
		int pin;
		printf("Please enter a servo channel from 0-15:\n");
		scanf("%d", &pin); 
		if (pca9685_servo_new(&servo, &pca, pin, 100, 3000) < 0) {
			fprintf(stderr, "error in servo init: %s\n", strerror(errno));
			continue;
		}
		
		int us = 1500;
		while (us > 0) {
			if (pca9685_servo_write_us(&servo, us) < 0)
				break;

			printf("Servo %d is currently at %d us\n", pin, us);
			delay(1000);
			
			printf("Please enter a microsecond value from 100-3000:\n");	
			scanf("%d", &us);
		}
	}


	return 0;
}
