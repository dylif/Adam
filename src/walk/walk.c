#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <basic_i2c.h>
#include <lsm9ds0.h>
#include <pca9685.h>
#include <pca9685_servo.h>

#define PCA_ADDR 0x40
#define LSM_G_ADDR 0x6b
#define LSM_AM_ADDR 0x1d
#define PCA_HERTZ 50

#define SERVO_MIN 700
#define SERVO_MAX 2500
#define SERVO_NEU 1600

#define SERVO_NUM 6

#define SERVO_R_HIP 0
#define SERVO_L_HIP 1

#define SERVO_R_KNEE 2
#define SERVO_L_KNEE 3

#define SERVO_R_ANK 4
#define SERVO_L_ANK 5

int centre_all(struct pca9685_servo **servos)
{
	int i;
	for (i = 0; i < SERVO_NUM; ++i) {
		pca9685_servo_write_us(servos[i], SERVO_NEU);
	}	
	
	return 0;
}	

int right_step(struct pca9685_servo **servos)
{
	int deg;
	
	deg = pca9685_servo_deg_to_us(servos[SERVO_L_ANK], 120);
	pca9685_servo_write_us(servos[SERVO_L_ANK], deg);
	
	deg = pca9685_servo_deg_to_us(servos[SERVO_R_KNEE], 0);
	pca9685_servo_write_us(servos[SERVO_R_KNEE], deg);
	
	return 0;
}	

int main()
{
	int status, i;
	
	struct pca9685 pca;
	struct pca9685_servo *servos[SERVO_NUM];
	
	struct pca9685_servo right_knee;
	struct pca9685_servo left_knee;
	
	struct pca9685_servo right_hip;
	struct pca9685_servo left_hip;
	
	struct pca9685_servo right_ankle;
	struct pca9685_servo left_ankle;
	
	servos[SERVO_R_HIP] = &right_hip;
	servos[SERVO_L_HIP] = &left_hip;
	servos[SERVO_R_KNEE] = &right_knee;
	servos[SERVO_L_KNEE] = &left_knee;
	servos[SERVO_R_ANK] = &right_ankle;
	servos[SERVO_L_ANK] = &left_ankle;
	
	/* open i2c file */
	int fd = open(DEVICE_I2C, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Error in opening i2c file: %s\n", strerror(errno));
		return fd;
	}
	
	if ((status = pca9685_new(&pca, fd, PCA_ADDR)) < 0) {
		fprintf(stderr, "Error in pca_new %d: %s\n", status, strerror(status * -1));
		return status;
	}
	
	for (i = 0; i < SERVO_NUM; ++i) {
		if ((status = pca9685_servo_new(servos[i], &pca, i, SERVO_MIN, SERVO_MAX)) < 0) {
			fprintf(stderr, "Error in servo_new %d for servo : %s\n", status, i, strerror(status * -1));
			return status;
		}
	}	
	
	if ((status = pca9685_pwm_init(&pca, PCA_HERTZ)) < 0) {
		fprintf(stderr, "Error in pwm_init %d: %s=\n", status, strerror(status * -1));
		return status;
	}
	
}
