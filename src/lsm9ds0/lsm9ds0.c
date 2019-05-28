#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <basic_i2c.h>

#include "lsm9ds0.h"

/* declare static functions */
static void set_defaults(struct lsm9ds0 *lsm);
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
int lsm9ds0_new(struct lsm9ds0 *lsm, struct lsm9ds0_settings *settings, int fd, unsigned int g_addr, unsigned int am_addr)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;
		
	/* set default values */
	set_defaults(lsm);
	
	/* assume default settings */
	if (settings == NULL)
		settings = &settings_def;
		
	if (fd < 0)
		return fd;
	
	/* set members to given values */
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
	
	return 0;
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
	
	/* Set scale to default 245 dps */
	if ((status = g_write8(lsm, CTRL_REG4_G, 0x00)) < 0)
		return status;
	
	if ((status = g_write8(lsm, CTRL_REG5_G, 0x00)) < 0)
		return status;
		
	return 0;
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
	
	/* set scale to default 2g */
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

/* lsm9ds0_cal: zero out the gyro and the accel by finding biases */
int lsm9ds0_cal(struct lsm9ds0 *lsm, int trials, int ms)
{  
	int status, i;
	
	float *g_data;
	float *a_data;
	
	float gx_sum = 0;
	float gy_sum = 0;
	float gz_sum = 0;
	float ax_sum = 0;
	float ay_sum = 0;
	float az_sum = 0;
	
	size_t len;
	
	if (trials <= 0 || lsm == NULL)
		return -EINVAL;
	
	/* lenght of our arrays for 3 dimensions */	
	len = trials * 3;
	
	g_data = calloc(len, sizeof(*g_data));
	if (g_data == NULL)
		return abs(errno) * -1;
		
	a_data = calloc(len, sizeof(*a_data));
	if (g_data == NULL) {
		free(g_data);
		return abs(errno) * -1;
	}
	
	/* reset biases */
	for (i = 0; i < 3; ++i) {
		lsm->g_bias[i] = 0;
		lsm->g_bias[i] = 0;
	}	
	
	/* collect and organize the data into our arrays */
	for (i = 0; i < trials; ++i) {
		delay(ms);
		
		if ((status = lsm9ds0_gyro_read(lsm)) < 0)
			goto error_free_all;
		if ((status = lsm9ds0_accel_read(lsm)) < 0)
			goto error_free_all;
		
		lsm_update(lsm);
		
		g_data[(i * 3) + 0] = lsm->gx;
		g_data[(i * 3) + 1] = lsm->gy;
		g_data[(i * 3) + 2] = lsm->gz;
		
		a_data[(i * 3) + 0] = lsm->ax;
		a_data[(i * 3) + 1] = lsm->ay;
		a_data[(i * 3) + 2] = lsm->az;
	}
	
	/* average the data	*/
	for (i = 0; i < trials; ++i) {
		gx_sum += g_data[(i * 3) + 0];
		gy_sum += g_data[(i * 3) + 1];
		gz_sum += g_data[(i * 3) + 2];
		
		ax_sum += a_data[(i * 3) + 0];
		ay_sum += a_data[(i * 3) + 1];
		az_sum += a_data[(i * 3) + 2];
	}
	
	lsm->g_bias[0] = gx_sum / trials;
	lsm->g_bias[1] = gy_sum / trials;
	lsm->g_bias[2] = gz_sum / trials;
	
	lsm->a_bias[0] = ax_sum / trials;
	lsm->a_bias[1] = ay_sum / trials;
	lsm->a_bias[2] = az_sum / trials;
	
	/* clean up */
	if (g_data != NULL)
		free(g_data);
	if (a_data != NULL)
		free(a_data);
	
	return 0;
	
error_free_all:
	if (g_data != NULL)
		free(g_data);
	if (a_data != NULL)
		free(a_data);
		
	return status;
}

/* lsm9ds0_gyro_read: read raw data from the gyro */
int lsm9ds0_gyro_read(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* declare, zero and get size of array */
	uint8_t tmp[6];
	size_t tmp_sz = sizeof(tmp) / sizeof(tmp[0]);
	
	/* Read 6 bytes, beginning at OUT_X_L_G */
	if ((status = g_read(lsm, OUT_X_L_G, tmp, tmp_sz)) < 0)
		return status;
	
	/* x-axis values into gx_raw, y-axis values into gy_raw, z-axis values into gz_raw */
	lsm->gx_raw = (tmp[1] << 8) | tmp[0];
	lsm->gy_raw = (tmp[3] << 8) | tmp[2];
	lsm->gz_raw = (tmp[5] << 8) | tmp[4];
	
	return 0;
}

/* lsm9ds0_accel_read: read raw data from the accel */
int lsm9ds0_accel_read(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* declare, zero and get size of array */
	uint8_t tmp[6] = {0, 0, 0, 0, 0, 0};
	size_t tmp_sz = sizeof(tmp) / sizeof(tmp[0]);
	
	/* read 6 bytes, beginning at OUT_X_L_A */
	if ((status = am_read(lsm, OUT_X_L_A, tmp, tmp_sz)) < 0)
		return status;
	
	/* x-axis values into ax_raw, y-axis values into ay_raw, z-axis values into az_raw */
	lsm->ax_raw = (tmp[1] << 8) | tmp[0];
	lsm->ay_raw = (tmp[3] << 8) | tmp[2];
	lsm->az_raw = (tmp[5] << 8) | tmp[4];
	
	return 0;
}

/* lsm9ds0_mag_read: read raw data from the mag */
int lsm9ds0_mag_read(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;	
	
	/* declare, zero and get size of array */
	uint8_t tmp[6] = {0, 0, 0, 0, 0, 0};
	size_t tmp_sz = sizeof(tmp) / sizeof(tmp[0]);
	
	/* read 6 bytes, beginning at OUT_X_L_M */
	if ((status = am_read(lsm, OUT_X_L_M, tmp, tmp_sz)) < 0)
		return status;
	
	/* x-axis values into mx_raw, y-axis values into my_raw, and z-axis values into mz_raw */
	lsm->mx_raw = (tmp[1] << 8) | tmp[0]; 
	lsm->my_raw = (tmp[3] << 8) | tmp[2];
	lsm->mz_raw = (tmp[5] << 8) | tmp[4];
	
	return 0;
}

/* lsm9ds0_temp_read: read raw data from the temp */
int lsm9ds0_temp_read(struct lsm9ds0 *lsm)
{
	int status;
	
	if (lsm == NULL)
		return -EINVAL;
	
	/* declare, zero and get size of array */
	uint8_t tmp[2] = {0, 0};
	size_t tmp_sz = sizeof(tmp) / sizeof(tmp[0]);	
	
	/* read 2 bytes, beginning at OUT_TEMP_L_AM */
	if ((status = am_read(lsm, OUT_TEMP_L_AM, tmp, tmp_sz)) < 0)
		return status;
	
	/* temperature is a 12-bit signed int */
	lsm->temp = (((int16_t) tmp[1] << 12) | tmp[0] << 4 ) >> 4; 
	
	return 0;
}

/* lsm_update: update reading members to correctly scaled and calibrated values */
void lsm_update(struct lsm9ds0 *lsm)
{	
	/* scale the raw gyro readings and subtract the bias */
	lsm->gx = (lsm->gx_raw * lsm->g_res) - lsm->g_bias[0];
	lsm->gy = (lsm->gy_raw * lsm->g_res) - lsm->g_bias[1];
	lsm->gz = (lsm->gz_raw * lsm->g_res) - lsm->g_bias[2];
	
	/* scale the raw accel readings and subtract the bias */
	lsm->ax = (lsm->ax_raw * lsm->a_res) - lsm->a_bias[0];
	lsm->ay = (lsm->ay_raw * lsm->a_res) - lsm->a_bias[1];
	lsm->az = (lsm->az_raw * lsm->a_res) - lsm->a_bias[2];
	
	/* scale the raw mag readings */
	lsm->mx = lsm->mx_raw * lsm->m_res;
	lsm->my = lsm->my_raw * lsm->m_res;
	lsm->mz = lsm->mz_raw * lsm->m_res;
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
static void set_defaults(struct lsm9ds0 *lsm)
{
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
	
	lsm->gx_raw = 0;
	lsm->gy_raw = 0; 
	lsm->gz_raw = 0;
	lsm->ax_raw = 0;
	lsm->ay_raw = 0; 
	lsm->az_raw = 0;
	lsm->mx_raw = 0; 
	lsm->my_raw = 0; 
	lsm->mz_raw = 0;
    lsm->temp = 0;
    
    /* zero out bias arrays */
    size_t g_bias_sz = sizeof(lsm->g_bias) / sizeof(lsm->g_bias[0]);
    size_t a_bias_sz = sizeof(lsm->a_bias) / sizeof(lsm->a_bias[0]);
    
    int i;
    for (i = 0; i < g_bias_sz; ++i)
		lsm->g_bias[i] = 0;
	for (i = 0; i < a_bias_sz; ++i)
		lsm->a_bias[i] = 0;
}	

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
