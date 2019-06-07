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

#define Y_THRESHOLD (50)
#define X_THRESHOLD (50)

void lean_right(struct pca9685_servo **servos) {
	pca9685_servo_write_deg(servos[SERVO_R_HIP], pca9685_servo_read_deg(servos[SERVO_R_HIP]) + 5);
	pca9685_servo_write_deg(servos[SERVO_L_HIP], pca9685_servo_read_deg(servos[SERVO_L_HIP]) + 5);
	pca9685_servo_write_deg(servos[SERVO_R_ANK], pca9685_servo_read_deg(servos[SERVO_R_ANK]) - 5);
	pca9685_servo_write_deg(servos[SERVO_L_ANK], pca9685_servo_read_deg(servos[SERVO_L_ANK]) - 5);
	delay(100);                       
}
void lean_left(struct pca9685_servo **servos) {
	pca9685_servo_write_deg(servos[SERVO_R_HIP], pca9685_servo_read_deg(servos[SERVO_R_HIP]) - 5);
	pca9685_servo_write_deg(servos[SERVO_L_HIP], pca9685_servo_read_deg(servos[SERVO_L_HIP]) - 5);
	pca9685_servo_write_deg(servos[SERVO_R_ANK], pca9685_servo_read_deg(servos[SERVO_R_ANK]) + 5);
	pca9685_servo_write_deg(servos[SERVO_L_ANK], pca9685_servo_read_deg(servos[SERVO_L_ANK]) + 5);
	delay(100);                       
}

int main()
{
	int status, i;
	
	struct pca9685 pca;
	struct lsm9ds0 lsm;
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
	
	/* create pca */
	if ((status = pca9685_new(&pca, fd, PCA_ADDR)) < 0) {
		fprintf(stderr, "Error in pca_new %d: %s\n", status, strerror(status * -1));
		return status;
	}
	
	/* create lsm */
	if ((status = lsm9ds0_new(&lsm, &settings, fd, LSM_G_ADDR, LSM_AM_ADDR)) < 0) {
		fprintf(stderr, "Error in lsm_new %d: %s\n", status, strerror(status * -1));
		return status;
	}
	
	/* create servos */
	for (i = 0; i < SERVO_NUM; ++i) {
		if ((status = pca9685_servo_new(servos[i], &pca, i, SERVO_MIN, SERVO_MAX)) < 0) {
			fprintf(stderr, "Error in servo_new %d for servo %d: %s\n", status, i, strerror(status * -1));
			return status;
		}
	}	
	
	/* get pca ready for pwm I/O */
	if ((status = pca9685_pwm_init(&pca, PCA_HERTZ)) < 0) {
		fprintf(stderr, "Error in pwm_init %d: %s\n", status, strerror(status * -1));
		return status;
	}
	
	/* calibrate by getting data 3 times */
	printf("Calibrating LSM... Do not touch!!\n");
	
	if ((status = lsm9ds0_cal(&lsm, 3, 1000)) < 0) {
		fprintf(stderr, "Error in lsm_cal %d: %s\n", status, strerror(status * -1));
		return status;		
	}
	
	/* state biases */
	printf("Gyro: X Bias: %6.2f Y Bias: %6.2f Z Bias: %6.2f\n", lsm.g_bias[0], lsm.g_bias[1], lsm.g_bias[2]);	
	printf("Accel: X Bias: %6.2f Y Bias: %6.2f Z Bias: %6.2f\n", lsm.a_bias[0], lsm.a_bias[1], lsm.a_bias[2]);
	
	for (;;) {
			if ((status = lsm9ds0_gyro_read(&lsm)) < 0) {
			fprintf(stderr, "Error in gyro_read %d: %s\n", status, strerror(status * -1));
			return status;
		}
		
		lsm_update(&lsm);
		
		/* leaning too far right */
		if (lsm.gy > Y_THRESHOLD)
			lean_left(servos);
		/* leaning too far left */
		if (lsm.gy < -Y_THRESHOLD)
			lean_right(servos);
	}	
}
