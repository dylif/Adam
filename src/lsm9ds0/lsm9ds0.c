#include <stdint.h>
#include <errno.h>
#include <basic_i2c.h>

#include "lsm9ds0.h"

/* declare static functions */
static int g_read8(struct lsm9ds0 *lsm, uint8_t reg, uint8_t *buf);
static int g_write8(struct lsm9ds0 *lsm, uint8_t reg, uint8_t data);
static int g_read(struct lsm9ds0 *lsm, uint8_t base_reg, uint8_t *buf, size_t buf_sz);
static int am_read8(struct lsm9ds0 *lsm, uint8_t reg, uint8_t *buf);
static int am_write8(struct lsm9ds0 *lsm, uint8_t reg, uint8_t data);
static int am_read(struct lsm9ds0 *lsm, uint8_t base_reg, uint8_t *buf, size_t buf_sz);

static struct lsm9ds0_settings settings_def =
{
	G_SCALE_245DPS,
	A_SCALE_2G,
 	M_SCALE_2GS,
	G_ODR_95_BW_125, 
	A_ODR_50, 
	M_ODR_50
};

/* lsm9ds0_new: setup a new lsm9ds0 struct and initalize gyro, accel, and mag functions */
uint16_t lsm9ds0_new(struct lsm9ds0 *lsm, struct lsm9ds0_settings *settings, int fd, unsigned int g_addr, unsigned int am_addr)
{
	int status;
	
	uint8_t g_test;
	uint8_t am_test;
	
	if (lsm == NULL)
		return -EINVAL;
		
	/* set default values */
	lsm->fd = -1;
		
	lsm->g_addr = 0;
	lsm->am_addr = 0;

	lsm->g_scl = -1;
	lsm->a_scl = -1;
	lsm->m_scl = -1;
	
	lsm->g_res = -1;
	lsm->a_res = -1;
	lsm->m_res = -1;

	lsm->gx = 0;
	lsm->gy = 0; 
	lsm->gz = 0;
	lsm->ax = 0;
	lsm->ay = 0; 
	lsm->az = 0;
	lsm->mx = 0; 
	lsm->my = 0; 
	lsm->mz = 0;
    
    lsm->temp = 0;
    
    /* zero out bias arrays */
    size_t g_bias_sz = sizeof(lsm->g_bias) / sizeof(lsm->g_bias[0]);
    size_t a_bias_sz = sizeof(lsm->a_bias) / sizeof(lsm->a_bias[0]);
    
    int i;
    for (i = 0; i < g_bias_sz; ++i)
		lsm->g_bias[i] = 0;
	for (i = 0; i < a_bias_sz; ++i)
		lsm->a_bias[i] = 0;
	
	if (settings == NULL)
		settings = &settings_def;
		
	if (fd < 0)
		return fd;
		
	lsm->fd = fd;
		
	lsm->g_addr = g_addr;
	lsm->am_addr = am_addr;

	/* set scale members using settings struct */
	lsm->g_scl = settings->g_scl;
	lsm->a_scl = settings->a_scl;
	lsm->m_scl = settings->m_scl;
	
	/* calculate DPS, Gs, and g per ADC tick using the scale members */
	calc_g_res(lsm);
	calc_m_res(lsm);
	calc_a_res(lsm);
	
	if ((status = g_read8(lsm, WHO_AM_I_G, &g_test)) < 0)
		return status;
	if ((status = am_read8(lsm, WHO_AM_I_AM, &am_test)) < 0)
		return status;
	
	/* gyro init */
	
	/* turn on the gyro, set up interrupts, etc. */
	if ((status = lsm9ds0_gyro_init(lsm)) < 0)
		return status;
		
	/* set the gyro data rate and bandwidth. */
	if ((status = set_g_odr(lsm, settings->g_odr)) < 0)
		return status;
		
	/* Set the gyro range */
	if ((status = set_g_scl(lsm, lsm->g_scl)) < 0)
		return status;
	
	/* accel init */
	
	/* turn on all axes of the accel, set up interrupts, etc. */
	if ((status = lsm9ds0_accel_init(lsm)) < 0)
		return status;
		
	/* set the accel data rate */
	if ((status = set_a_odr(lsm, settings->a_odr)) < 0)
		return status;
	
	/* set the accel range */
	if ((status = set_a_scl(lsm, lsm->a_scl)) < 0)
		return status;
	
	/* mag init */
	
	/* turn on all axes of the mag, set up interrupts, etc. */
	if ((status = lsm9ds0_mag_init(lsm)) < 0)
		return status;
	
	/* set the mag data rate */	
	if ((status = set_m_odr(lsm, settings->m_odr)) < 0)
		return status;
		
	/* set the mag range */	
	if ((status = set_m_scl(lsm, lsm->m_scl)) < 0)
		return status;
	
	/* return the WHO_AM_I registers we read */
	return (am_test << 8) | g_test;
}

/* lsm9ds0_gyro_init: initalize the gyro and set interrupts, etc. */
int lsm9ds0_gyro_init(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* normal mode, enable all axes */
	if ((status = g_write8(lsm, CTRL_REG1_G, 0x0F)) < 0)
		return status;
	
	/* normal mode, high cutoff frequency */
	if ((status = g_write8(lsm, CTRL_REG2_G, 0x00)) < 0)
		return status;
	
	/* int1 enabled (pp, active low), data read on DRDY_G: */
	if ((status = g_write8(lsm, CTRL_REG3_G, 0x88)) < 0)
		return status;
	
	/* Set scale to 245 dps */
	if ((status = g_write8(lsm, CTRL_REG4_G, 0x00)) < 0)
		return status;
	
	if ((status = g_write8(lsm, CTRL_REG5_G, 0x00)) < 0)
		return status;
		
	return 0;
	
	/*
	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	configGyroInt(lsm,0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
	*/
}

/* lsm9ds0_accel_init: initalize the accel and set interrupts, etc. */
int lsm9ds0_accel_init(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	if ((status = am_write8(lsm, CTRL_REG0_AM, 0x00)) < 0)
		return status;
	
	/* 100Hz data rate, x/y/z all enabled */
	if ((status = am_write8(lsm, CTRL_REG1_AM, 0x57)) < 0)
		return status;
	
	/* set scale to 2g */
	if ((status = am_write8(lsm, CTRL_REG2_AM, 0x00)) < 0)
		return status;

	/* accelerometer data ready on INT1_AM (0x04) */
	if ((status = am_write8(lsm, CTRL_REG3_AM, 0x04)) < 0)
		return status;
		
	return 0;
}

/* lsm9ds0_mag_init: initalize the mag and set interrupts, etc. */
int lsm9ds0_mag_init(struct lsm9ds0 *lsm)
{			
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
							
	/* mag data rate - 100 Hz, enable temperature sensor */
	if ((status = am_write8(lsm, CTRL_REG5_AM, 0x94)) < 0)
		return status;
	
	/* mag scale to +/- 2GS */				
	if ((status = am_write8(lsm, CTRL_REG6_AM, 0x00)) < 0)
		return status;
	
	/* continuous conversion mode */
	if ((status = am_write8(lsm, CTRL_REG7_AM, 0x00)) < 0)
		return status;
	
	/* magnetometer data ready on INT2_AM (0x08) */
	if ((status = am_write8(lsm, CTRL_REG4_AM, 0x04)) < 0)
		return status;
	
	/* enable interrupts for mag, active-low, push-pull */
	if ((status = am_write8(lsm, INT_CTRL_REG_M, 0x09)) < 0)
		return status;
	
	return 0;
}

/* This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
 * them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
 * for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
 * the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
 * subtract the biases ourselves. This results in a more accurate measurement in general and can
 * remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
 * is good practice.
 */
/* lsm9ds0_cal: calibrate and find biases to be stored in the struct */
int lsm9ds0_cal(struct lsm9ds0 *lsm)
{  
	int status;
	
	/* declare and zero the arrays */
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int16_t gyro_bias[3] = {0, 0, 0};
	int16_t accel_bias[3] = {0, 0, 0};
	uint8_t temp;
	uint8_t c;
	
	int samples; 
	int i;

	size_t data_sz = sizeof(data) / sizeof(data[0]);

	if (lsm == NULL)
		return -EINVAL;

	/* get gyro bias */
	if ((status = g_read8(lsm, CTRL_REG5_G, &c)) < 0)
		return status;
	
	/* enable gyro FIFO and wait */
	if ((status = g_write8(lsm, CTRL_REG5_G, c | 0x40)) < 0)
		return status;
	delay(20);
	
	/* enable gyro FIFO stream mode, set watermark at 32 samples and delay to collect samples */
	if ((status = g_write8(lsm, FIFO_CTRL_REG_G, 0x20 | 0x1F)) < 0)
		return status;
	delay(1000);  								
	
	/* read number of stored samples */
	if ((status = g_read8(lsm, FIFO_SRC_REG_G, &temp)) < 0)
		return status;
	samples = (temp & 0x1F); 

	/* read the gyro data stored in the FIFO */
	for(i = 0; i < samples; ++i) {            
		if ((status = g_read(lsm, OUT_X_L_G, &data[0], data_sz)) < 0) 
			return status;
			
		gyro_bias[0] += (((int16_t) data[1] << 8) | data[0]);
		gyro_bias[1] += (((int16_t) data[3] << 8) | data[2]);
		gyro_bias[2] += (((int16_t) data[5] << 8) | data[4]);
	}  
	
	/* average the data */
	gyro_bias[0] /= samples; 						
	gyro_bias[1] /= samples; 
	gyro_bias[2] /= samples; 
	
	/* Properly scale the data to get deg/s */
	lsm->g_bias[0] = (float) gyro_bias[0] * lsm->g_res; 			 
	lsm->g_bias[1] = (float) gyro_bias[1] * lsm->g_res;
	lsm->g_bias[2] = (float) gyro_bias[2] * lsm->g_res;
	
	/* Disable gyro FIFO, delay, and enable gyro bypass mode */
	if ((status = g_read8(lsm, CTRL_REG5_G, &c)) < 0)
		return status;
	if ((status = g_write8(lsm, CTRL_REG5_G, c & ~0x40)) < 0)
		return status;   
	delay(20);
	if ((status = g_write8(lsm, FIFO_CTRL_REG_G, 0x00)) < 0)
		return status;

	/* get the accelerometer biases */
	if ((status = am_read8(lsm, CTRL_REG0_AM, &c)) < 0)
		return status;
		
	/* enable accelerometer FIFO and wait */
	if ((status = am_write8(lsm,CTRL_REG0_AM, c | 0x40)) < 0)
		return status;
	delay(20); 
	
	/* enable accelerometer FIFO stream mode, set watermark at 32 samples and delay to collect samples */
	if ((status = am_write8(lsm,FIFO_CTRL_REG, 0x20 | 0x1F)) < 0)
			return status;
	delay(1000);
	
	/* read number of stored samples */
	if ((status = am_read8(lsm, FIFO_SRC_REG, &temp)) < 0)
		return status;
	samples = (temp & 0x1F);
	
	/* read the accelerometer data stored in the FIFO
	 * NOTE: assumes sensor facing up!
	 */
	for(i = 0; i < samples; ++i) {          	
		if ((status = am_read(lsm, OUT_X_L_A, &data[0], data_sz)) < 0)
			return status;
		accel_bias[0] += (((int16_t) data[1] << 8) | data[0]);
		accel_bias[1] += (((int16_t) data[3] << 8) | data[2]);
		accel_bias[2] += (((int16_t) data[5] << 8) | data[4]) - (int16_t)(1.0f/lsm->a_res); 
	}  
	
	/* average the data */
	accel_bias[0] /= samples; 
	accel_bias[1] /= samples; 
	accel_bias[2] /= samples; 
	
	/* properly scale data to get gs */
	lsm->a_bias[0] = (float) accel_bias[0] * lsm->a_res; 
	lsm->a_bias[1] = (float) accel_bias[1] * lsm->a_res;
	lsm->a_bias[2] = (float) accel_bias[2] * lsm->a_res;
	
	/* disable accelerometer FIFO, wait and enable accelerometer bypass mode */
	if ((status = am_read8(lsm, CTRL_REG0_AM, &c)) < 0)
		return status;
	if ((status = am_write8(lsm, CTRL_REG0_AM, c & ~0x40)) < 0)
		return status;
	delay(20);
	if ((status = am_write8(lsm, FIFO_CTRL_REG, 0x00)) < 0)
		return status;
		
	return 0;
}

/* lsm9ds0_gyro_read: read raw data from the gyro */
int lsm9ds0_gyro_read(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* declare, zero and get size of array */
	uint8_t temp[6];
	size_t temp_sz = sizeof(temp) / sizeof(temp[0]);
	
	/* Read 6 bytes, beginning at OUT_X_L_G */
	if ((status = g_read(lsm, OUT_X_L_G, temp, temp_sz)) < 0)
		return status;
	
	/* x-axis values into gx, y-axis values into gy, z-axis values into gz */
	lsm->gx = (temp[1] << 8) | temp[0];
	lsm->gy = (temp[3] << 8) | temp[2];
	lsm->gz = (temp[5] << 8) | temp[4];
	
	return 0;
}

/* lsm9ds0_accel_read: read raw data from the accel */
int lsm9ds0_accel_read(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* declare, zero and get size of array */
	uint8_t temp[6] = {0, 0, 0, 0, 0, 0};
	size_t temp_sz = sizeof(temp) / sizeof(temp[0]);
	
	/* read 6 bytes, beginning at OUT_X_L_A */
	if ((status = am_read(lsm, OUT_X_L_A, temp, temp_sz)) < 0)
		return status;
	
	/* x-axis values into ax, y-axis values into ay, z-axis values into az */
	lsm->ax = (temp[1] << 8) | temp[0];
	lsm->ay = (temp[3] << 8) | temp[2];
	lsm->az = (temp[5] << 8) | temp[4];
	
	return 0;
}

/* lsm9ds0_mag_read: read raw data from the mag */
int lsm9ds0_mag_read(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* declare, zero and get size of array */
	uint8_t temp[6] = {0, 0, 0, 0, 0, 0};
	size_t temp_sz = sizeof(temp) / sizeof(temp[0]);
	
	/* read 6 bytes, beginning at OUT_X_L_M */
	if ((status = am_read(lsm, OUT_X_L_M, temp, temp_sz)) < 0)
		return status;
	
	/* x-axis values into mx, y-axis values into my, and z-axis values into mz */
	lsm->mx = (temp[1] << 8) | temp[0]; 
	lsm->my = (temp[3] << 8) | temp[2];
	lsm->mz = (temp[5] << 8) | temp[4];
	
	return 0;
}

/* lsm9ds0_temp_read: read raw data from the temp */
int lsm9ds0_temp_read(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;
	
	/* declare, zero and get size of array */
	uint8_t temp[2] = {0, 0};
	size_t temp_sz = sizeof(temp) / sizeof(temp[0]);	
	
	/* read 2 bytes, beginning at OUT_TEMP_L_AM */
	if ((status = am_read(lsm, OUT_TEMP_L_AM, temp, temp_sz)) < 0)
		return status;
		
	lsm->temp = (((int16_t) temp[1] << 12) | temp[0] << 4 ) >> 4; // Temperature is a 12-bit signed integer
	
	return 0;
}

/* calc_gyro: return the gyro raw reading multiplied by the pre-calculated DPS / (ADC tick) */
float calc_gyro(struct lsm9ds0 *lsm, int16_t gyro)
{
	return lsm->g_res * gyro; 
}

/* calc_accel: return the accel raw reading multiplied by the pre-calculated g's / (ADC tick) */
float calc_accel(struct lsm9ds0 *lsm, int16_t accel)
{
	return lsm->a_res * accel;
}

/* calc_mag: return the mag raw reading multiplied by the pre-calculated Gs / (ADC tick) */
float calc_mag(struct lsm9ds0 *lsm, int16_t mag)
{
	return lsm->m_res * mag;
}

/* set_g_scl: write g_scl to hardware registers and update g_res */
int set_g_scl(struct lsm9ds0 *lsm, int g_scl)
{
	int status;
	uint8_t temp;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* read CTRL_REG4_G so we don't modify the other the other bits */
	if ((status = g_read8(lsm, CTRL_REG4_G, &temp)) < 0)
		return status;
		
	/* mask out the gyro scale bits */
	temp &= 0xFF^(0x3 << 4);
	
	/* shift in our new scale bits */
	temp |= g_scl << 4;
	
	/* write the new register value back into CTRL_REG4_G */
	if ((status = g_write8(lsm, CTRL_REG4_G, temp)) < 0)
		return status;
	
	lsm->g_scl = g_scl;
	
	/* calculate a new g_res, which relies on g_scl being set correctly */ 
	calc_g_res(lsm);
	
	return lsm->g_scl;
}

/* set_a_scl: write a_scl to hardware registers and update a_res */
int set_a_scl(struct lsm9ds0 *lsm, int a_scl)
{
	int status;
	uint8_t temp;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* read CTRL_REG2_AM so we don't modify the other the other bits */
	if ((status = am_read8(lsm, CTRL_REG2_AM, &temp)) < 0)
		return status;
		
	/* mask out the accel scale bits */
	temp &= 0xFF^(0x3 << 3);
	
	/* shift in our new scale bits */
	temp |= a_scl << 3;
	
	/* write the new register value back into CTRL_REG2_AM */
	if ((status = am_write8(lsm,CTRL_REG2_AM, temp)) < 0)
		return status;
	
	lsm->a_scl = a_scl;
	
	/* calculate a new a_res, which relies on a_scl being set correctly */
	calc_a_res(lsm);
	
	return lsm->a_scl;
}

/* set_m_scl: write m_scl to hardware registers and update m_res */
int set_m_scl(struct lsm9ds0 *lsm, int m_scl)
{
	int status;
	uint8_t temp;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* read CTRL_REG6_AM so we don't modify the other the other bits */
	if ((status = am_read8(lsm, CTRL_REG6_AM, &temp)) < 0)
		return status;
	
	/* mask out the mag scale bits */
	temp &= 0xFF^(0x3 << 5);
	
	/* shift in our new scale bits */
	temp |= m_scl << 5;
	
	/* write the new register value back into CTRL_REG6_AM */
	if ((status = am_write8(lsm, CTRL_REG6_AM, temp)) < 0)
		return status;
	
	lsm->m_scl = m_scl;
	
	/* calculate a new m_res, which relies on m_scl being set correctly */
	calc_m_res(lsm);
	
	return lsm->m_scl;
}

/* set_g_odr: write g_rate to hardware registers */
int set_g_odr(struct lsm9ds0 *lsm, int g_rate)
{
	int status;
	uint8_t temp;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* read CTRL_REG1_G so we don't modify the other the other bits */
	if ((status = g_read8(lsm, CTRL_REG1_G, &temp)) < 0)
		return status;
		
	/* mask out the gyro ODR bits */
	temp &= 0xFF^(0xF << 4);
	
	/* shift in our new ODR bits */
	temp |= (g_rate << 4);
	
	/* write the new register value back into CTRL_REG1_G */
	if ((status = g_write8(lsm, CTRL_REG1_G, temp)) < 0)
		return status;
	
	return g_rate;
}

/* set_a_odr: write a_rate to hardware registers */
int set_a_odr(struct lsm9ds0 *lsm, int a_rate)
{
	int status;
	uint8_t temp;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* read CTRL_REG1_AM so we don't modify the other the other bits */
	if ((status = am_read8(lsm, CTRL_REG1_AM, &temp)) < 0)
		return status;
		
	/* mask out the accel ODR bits */
	temp &= 0xFF^(0xF << 4);
	
	/* shift in our new ODR bits */
	temp |= (a_rate << 4);
	
	/* write the new register value back into CTRL_REG1_AM */
	if ((status = am_write8(lsm,CTRL_REG1_AM, temp)) < 0)
		return status;
		
	return a_rate;
}

/* set_a_abw: write abw_rate to hardware registers */
int set_a_abw(struct lsm9ds0 *lsm, int abw_rate)
{
	int status;
	uint8_t temp;

	if (lsm == NULL)
		return -EINVAL;
	
	/* read CTRL_REG2_AM so we don't modify the other the other bits */
	if ((status = am_read8(lsm, CTRL_REG2_AM, &temp)) < 0)
		return status;
		
	/* mask out the accel ABW bits */
	temp &= 0xFF^(0x3 << 7);
	
	/* shift in our new ODR bits */
	temp |= (abw_rate << 7);
	
	/* write the new register value back into CTRL_REG2_AM */
	if ((status = am_write8(lsm, CTRL_REG2_AM, temp)) < 0)
		return status;
		
	return abw_rate;
}

/* set_m_odr: write m_rate to hardware registers */
int set_m_odr(struct lsm9ds0 *lsm, int m_rate)
{
	int status;
	uint8_t temp;
	
	if (lsm == NULL)
		return -EINVAL;
	
	/* read CTRL_REG5_AM so we don't modify the other the other bits */
	if ((status = am_read8(lsm, CTRL_REG5_AM, &temp)) < 0)
		return status;
		
	/* mask out the mag ODR bits */
	temp &= 0xFF^(0x7 << 2);
	
	/* shift in our new ODR bits */
	temp |= (m_rate << 2);
	
	/* write the new register value back into CTRL_REG5_AM */
	if ((status = am_write8(lsm, CTRL_REG5_AM, temp)) < 0)
		return status;
		
	return m_rate;
}

/* lsm9ds0_gyro_cfg_int: configure logical interrupts for the gyro */
int lsm9ds0_gyro_cfg_int(struct lsm9ds0 *lsm, uint8_t int1_cfg, 
	uint16_t int1_thsx, uint16_t int1_thsy, uint16_t int1_thsz, uint8_t duration)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;
	
	if ((status = g_write8(lsm, INT1_CFG_G, int1_cfg)) < 0)
		return status;
		
	if ((status = g_write8(lsm, INT1_THS_XH_G, (int1_thsx & 0xFF00) >> 8)) < 0)
		return status;
			
	if ((status = g_write8(lsm, INT1_THS_XL_G, (int1_thsx & 0xFF))) < 0)
		return status;
		
	if ((status = g_write8(lsm, INT1_THS_YH_G, (int1_thsy & 0xFF00) >> 8)) < 0)
		return status;
	
	if ((status = g_write8(lsm, INT1_THS_YL_G, (int1_thsy & 0xFF))) < 0)
		return status;
		
	if ((status = g_write8(lsm, INT1_THS_ZH_G, (int1_thsz & 0xFF00) >> 8)) < 0)
		return status;
		
	if ((status = g_write8(lsm, INT1_THS_ZL_G, (int1_thsz & 0xFF))) < 0)
		return status;
		
	if (duration) {
		if ((status = g_write8(lsm, INT1_DURATION_G, 0x80 | duration)) < 0)
			return status;
	} else {
		if ((status = g_write8(lsm, INT1_DURATION_G, 0x00)) < 0)
			return status;
	}
	
	return 0;
}

/* calc_g_res: calculate the gyro data resolution */ 
int calc_g_res(struct lsm9ds0 *lsm)
{
	/*
	 * Possible gyro scales (and their register bit settings) are:
	 * 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	 * to calculate DPS/(ADC tick) based on that 2-bit value 
	 */
	
	if (lsm == NULL)
		return -EINVAL;
		
	if (lsm->g_scl < 0)
		return -EINVAL;
	
	switch (lsm->g_scl) {
	case G_SCALE_245DPS:
		lsm->g_res = 245.0f / 32768.0f;
		break;
	case G_SCALE_500DPS:
		lsm->g_res = 500.0f / 32768.0f;
		break;
	case G_SCALE_2000DPS:
		lsm->g_res = 2000.0f / 32768.0f;
		break;
	}
	
	return 0;
}

/* calc_a_res: calculate the accel data resolution */ 
int calc_a_res(struct lsm9ds0 *lsm)
{
	/*
	 * Possible accelerometer scales (and their register bit settings) are:
	 * 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an 
	 * algorithm to calculate g/(ADC tick) based on that 3-bit value
	 */
	
	if (lsm == NULL)
		return -EINVAL;
		
	if (lsm->a_scl < 0)
		return -EINVAL;
	
	lsm->a_res = (lsm->a_scl == A_SCALE_16G) ? 16.0f / 32768.0f : 
		   (((float) lsm->a_scl + 1.0f) * 2.0f) / 32768.0f;
		   
	return 0;
}

/* calc_m_res: calculate the mag data resolution */ 
int calc_m_res(struct lsm9ds0 *lsm)
{
	/* Possible magnetometer scales (and their register bit settings) are:
	 * 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	 * to calculate Gs/(ADC tick) based on that 2-bit value
	 */
	
	if (lsm == NULL)
		return -EINVAL;
		
	if (lsm->m_scl < 0)
		return -EINVAL;
	
	lsm->m_res = (lsm->m_scl == M_SCALE_2GS) ? 2.0f / 32768.0f : 
	       (float) (lsm->m_scl << 2) / 32768.0f;
	       
	return 0;
}

/* static functions */
static int g_read8(struct lsm9ds0 *lsm, uint8_t reg, uint8_t *buf)
{
	return i2c_read_reg8(lsm->fd, lsm->g_addr, reg, buf);
}

static int g_write8(struct lsm9ds0 *lsm, uint8_t reg, uint8_t data)
{
	return i2c_write_reg8(lsm->fd, lsm->g_addr, reg, data);
}

static int g_read(struct lsm9ds0 *lsm, uint8_t base_reg, uint8_t *buf, size_t buf_sz)
{
	return i2c_read(lsm->fd, lsm->g_addr, base_reg, buf, buf_sz);
}

static int am_read8(struct lsm9ds0 *lsm, uint8_t reg, uint8_t *buf)
{
	return i2c_read_reg8(lsm->fd, lsm->am_addr, reg, buf);
}

static int am_write8(struct lsm9ds0 *lsm, uint8_t reg, uint8_t data)
{
	return i2c_write_reg8(lsm->fd, lsm->am_addr, reg, data);
}

static int am_read(struct lsm9ds0 *lsm, uint8_t base_reg, uint8_t *buf, size_t buf_sz)
{
	return i2c_read(lsm->fd, lsm->am_addr, base_reg, buf, buf_sz);
}
