#include <errno.h>

#include "LSM9DS0.h"

static struct lsm9ds0_settings settings_def =
{
	G_SCALE_245DPS,
	A_SCALE_2G,
 	M_SCALE_2GS,
	G_ODR_95_BW_125, 
	A_ODR_50, 
	M_ODR_50
};

uint16_t lsm9ds0_new(struct lsm9ds0 *lsm, struct lsm9ds0_settings *settings, int fd, uint8_t g_addr, uint8_t am_addr;)
{
	int status;
	
	uint8_t g_test;
	uint8_t am_test;
	
	if (lsm == NULL)
		return -EINVAL;
	
	if (settings == NULL)
		settings = &LMS_Device_Defaults;
		
	if (fd < 0)
		return fd;
		
	lsm->fd = fd;
		
	lsm->g_addr = g_addr;
	lsm->am_addr = am_addr;

	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	lsm->g_scl = settings->g_scl;
	lsm->a_scl = settings->a_scl;
	lsm->m_scl = settings->m_scl;
	
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calc_g_res(lsm); // Calculate DPS / ADC tick, stored in gRes variable
	calc_m_res(lsm); // Calculate Gs / ADC tick, stored in mRes variable
	calc_a_res(lsm); // Calculate g / ADC tick, stored in aRes variable
	
	status = i2c_read_reg8(fd, WHO_AM_I_G, &g_test);
	if (status < 0)
		return status;
	status = i2c_read_reg8(fd, WHO_AM_I_AM, &am_test);
	if (status < 0)
		return status;
	
	// Gyro initialization stuff:
	initGyro(lsm);		// This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(lsm, settings->g_odr); 		// Set the gyro output data rate and bandwidth.
	setGyroScale(lsm, lsm->g_scl); 	// Set the gyro range
	
	// Accelerometer initialization stuff:
	initAccel(lsm); // "Turn on" all axes of the accel. Set up interrupts, etc.
	setAccelODR(lsm,init_t->a_odr); // Set the accel data rate.
	setAccelScale(lsm_t,lsm_t->a_scl); // Set the accel range.
	
	// Magnetometer initialization stuff:
	initMag(lsm_t); // "Turn on" all axes of the mag. Set up interrupts, etc.
	setMagODR(lsm_t,init_t->mODR); // Set the magnetometer output data rate.
	setMagScale(lsm_t,lsm_t->mScale); // Set the magnetometer's range.
	
	// Once everything is initialized, return the WHO_AM_I registers we read:
	return (am_test << 8) | g_test;
}

int lsm9ds0_gyro_init(struct lsm9ds0 *lsm)
{
	int status;
	
	/* Normal mode, enable all axes */
	if ((status = g_write8(lsm, CTRL_REG1_G, 0x0F)) < 0)
		return status;
	
	/* Normal mode, high cutoff frequency */
	if ((status = g_write8(lsm, CTRL_REG2_G, 0x00)) < 0)
		return status;
	
	/* Int1 enabled (pp, active low), data read on DRDY_G: */
	if ((status = g_write8(lsm, CTRL_REG3_G, 0x88)) < 0)
		return status;
	
	/* Set scale to 245 dps */
	if ((status = g_write8(lsm, CTRL_REG4_G, 0x00)) < 0)
		return status;
	
	if ((status = g_write8(lsm, CTRL_REG5_G, 0x00)) < 0)
		return status;
		
	return 0;
	
	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	configGyroInt(lsm_t,0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
}

int lsm9ds0_accel_init(struct lsm9ds0 *lsm)
{
	int status;
	
	if ((status = am_write8(lsm, CTRL_REG0_AM, 0x00)) < 0)
		return status;
	
	/* 100Hz data rate, x/y/z all enabled */
	if ((status = am_write8(lsm, CTRL_REG1_AM, 0x57)) < 0)
		return status;
	
	/* Set scale to 2g */
	if ((status = am_write8(lsm, CTRL_REG2_AM, 0x00)) < 0)
		return status;

	/* Accelerometer data ready on INT1_AM (0x04) */
	if ((status = am_write8(lsm, CTRL_REG3_AM, 0x04)) < 0)
		return status;
		
	return 0;
}



int lsm9ds0_mag_init(struct lsm9ds0 *lsm)
{									
	/* Mag data rate - 100 Hz, enable temperature sensor */
	xmWriteByte(lsm_t,CTRL_REG5_XM, 0x94); 
	
	/* Mag scale to +/- 2GS */				
	xmWriteByte(lsm_t,CTRL_REG6_XM, 0x00); 
	
	/* Continuous conversion mode */
	xmWriteByte(lsm_t,CTRL_REG7_XM, 0x00); 
	
	/* Magnetometer data ready on INT2_XM (0x08) */
	xmWriteByte(lsm_t,CTRL_REG4_XM, 0x04); 
	
	/* Enable interrupts for mag, active-low, push-pull */
	xmWriteByte(lsm_t,INT_CTRL_REG_M, 0x09); 
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void calLSM9DS0(struct lsm9ds0 *lsm, float *gbias, float *abias)
{  
  uint8_t data[6] 	= {0, 0, 0, 0, 0, 0};
  int16_t 
  	gyro_bias[3] 	= {0, 0, 0}, 
  	accel_bias[3] 	= {0, 0, 0};
  int samples, ii;
  
  // First get gyro bias
  uint8_t c = gReadByte(lsm_t,CTRL_REG5_G);
  gWriteByte(lsm_t, CTRL_REG5_G, c | 0x40);         	// Enable gyro FIFO  
  delay(20);                                 	// Wait for change to take effect
  gWriteByte(lsm_t, FIFO_CTRL_REG_G, 0x20 | 0x1F);  	// Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  								// delay 1000 milliseconds to collect FIFO samples
  
  samples = (gReadByte(lsm_t,FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    gReadBytes(lsm_t,OUT_X_L_G,  &data[0], 6);
    gyro_bias[0] += (((int16_t)data[1] << 8) | data[0]);
    gyro_bias[1] += (((int16_t)data[3] << 8) | data[2]);
    gyro_bias[2] += (((int16_t)data[5] << 8) | data[4]);
  }  

  gyro_bias[0] /= samples; 						// average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  gbias[0] = (float)gyro_bias[0]*lsm_t->gRes; 			 // Properly scale the data to get deg/s
  gbias[1] = (float)gyro_bias[1]*lsm_t->gRes;
  gbias[2] = (float)gyro_bias[2]*lsm_t->gRes;
  
  c = gReadByte(lsm_t,CTRL_REG5_G);
  gWriteByte(lsm_t, CTRL_REG5_G, c & ~0x40);  			// Disable gyro FIFO  
  delay(20);
  gWriteByte(lsm_t, FIFO_CTRL_REG_G, 0x00);   			// Enable gyro bypass mode
  

  //  Now get the accelerometer biases
  c = xmReadByte(lsm_t,CTRL_REG0_XM);
  xmWriteByte(lsm_t,CTRL_REG0_XM, c | 0x40);      	// Enable accelerometer FIFO  
  delay(20);                                	// Wait for change to take effect
  xmWriteByte(lsm_t,FIFO_CTRL_REG, 0x20 | 0x1F);  	// Enable accelerometer FIFO stream mode and set watermark at 32 samples
  delay(1000); 	 								// delay 1000 milliseconds to collect FIFO samples

  samples = (xmReadByte(lsm_t,FIFO_SRC_REG) & 0x1F); 	// Read number of stored accelerometer samples

   for(ii = 0; ii < samples ; ii++) {          	// Read the accelerometer data stored in the FIFO
    xmReadBytes(lsm_t,OUT_X_L_A, &data[0], 6);
    accel_bias[0] += (((int16_t)data[1] << 8) | data[0]);
    accel_bias[1] += (((int16_t)data[3] << 8) | data[2]);
    accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) - (int16_t)(1.0f/lsm_t->aRes); // Assumes sensor facing up!
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  abias[0] = (float)accel_bias[0]*lsm_t->aRes; // Properly scale data to get gs
  abias[1] = (float)accel_bias[1]*lsm_t->aRes;
  abias[2] = (float)accel_bias[2]*lsm_t->aRes;

  c = xmReadByte(lsm_t,CTRL_REG0_XM);
  xmWriteByte(lsm_t,CTRL_REG0_XM, c & ~0x40);    // Disable accelerometer FIFO  
  delay(20);
  xmWriteByte(lsm_t,FIFO_CTRL_REG, 0x00);       // Enable accelerometer bypass mode
}

void LSM9DS0_readAccel(LSM9DS0_t* lsm_t)
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	xmReadBytes(lsm_t,OUT_X_L_A, temp, 6); // Read 6 bytes, beginning at OUT_X_L_A
	lsm_t->ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	lsm_t->ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	lsm_t->az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

void LSM9DS0_readMag(LSM9DS0_t* lsm_t)
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	xmReadBytes(lsm_t,OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	lsm_t->mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	lsm_t->my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	lsm_t->mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void LSM9DS0_readTemp(LSM9DS0_t* lsm_t)
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	xmReadBytes(lsm_t,OUT_TEMP_L_XM, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L_M
	lsm_t->temperature = (((int16_t) temp[1] << 12) | temp[0] << 4 ) >> 4; // Temperature is a 12-bit signed integer
}

void LSM9DS0_readGyro(struct lsm9ds0 *lsm)
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	gReadBytes(lsm_t,OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	lsm_t->gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	lsm_t->gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	lsm_t->gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

float calcGyro(struct lsm9ds0 *lsm, int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return lsm_t->gRes * gyro; 
}

float calcAccel(struct lsm9ds0 *lsm, int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return lsm_t->aRes * accel;
}

float calcMag(struct lsm9ds0 *lsm, int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return lsm_t->mRes * mag;
}

void setGyroScale(struct lsm9ds0 *lsm, gyro_scale gScl)
{
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = gReadByte(lsm_t,CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 4);
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(lsm_t, CTRL_REG4_G, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	lsm_t->gScale = gScl;
	
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes(lsm_t);
}

void setAccelScale(struct lsm9ds0 *lsm, accel_scale aScl)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG2_XM);
	// Then mask out the accel scale bits:
	temp &= 0xFF^(0x3 << 3);
	// Then shift in our new scale bits:
	temp |= aScl << 3;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(lsm_t,CTRL_REG2_XM, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	lsm_t->aScale = aScl;
	
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes(lsm_t);
}

void setMagScale(struct lsm9ds0 *lsm, mag_scale mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG6_XM);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG6_XM:
	xmWriteByte(lsm_t,CTRL_REG6_XM, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	lsm_t->mScale = mScl;
	
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes(lsm_t);
}

void setGyroODR(struct lsm9ds0 *lsm, gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = gReadByte(lsm_t,CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(lsm_t, CTRL_REG1_G, temp);
}

void setAccelODR(struct lsm9ds0 *lsm, accel_odr aRate)
{
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG1_XM);
	// Then mask out the accel ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(lsm_t,CTRL_REG1_XM, temp);
}

void setAccelABW(struct lsm9ds0 *lsm, accel_abw abwRate)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG2_XM);
	// Then mask out the accel ABW bits:
	temp &= 0xFF^(0x3 << 7);
	// Then shift in our new ODR bits:
	temp |= (abwRate << 7);
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(lsm_t,CTRL_REG2_XM, temp);
}

void setMagODR(struct lsm9ds0 *lsm, mag_odr mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG5_XM);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	xmWriteByte(lsm_t,CTRL_REG5_XM, temp);
}


void configGyroInt(struct lsm9ds0 *lsm,  uint8_t int1Cfg, uint16_t int1ThsX, uint16_t int1ThsY, uint16_t int1ThsZ, uint8_t duration)
{
	gWriteByte(lsm_t, INT1_CFG_G, int1Cfg);
	gWriteByte(lsm_t, INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8);
	gWriteByte(lsm_t, INT1_THS_XL_G, (int1ThsX & 0xFF));
	gWriteByte(lsm_t, INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8);
	gWriteByte(lsm_t, INT1_THS_YL_G, (int1ThsY & 0xFF));
	gWriteByte(lsm_t, INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	gWriteByte(lsm_t, INT1_THS_ZL_G, (int1ThsZ & 0xFF));
	if (duration)
		gWriteByte(lsm_t, INT1_DURATION_G, 0x80 | duration);
	else
		gWriteByte(lsm_t, INT1_DURATION_G, 0x00);
}


void calcgRes(struct lsm9ds0 *lsm)
{
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	switch (lsm_t->gScale)
	{
	case G_SCALE_245DPS:
		lsm_t->gRes = 245.0f / 32768.0f;
		break;
	case G_SCALE_500DPS:
		lsm_t->gRes = 500.0f / 32768.0f;
		break;
	case G_SCALE_2000DPS:
		lsm_t->gRes = 2000.0f / 32768.0f;
		break;
	}
}

void calcaRes(struct lsm9ds0 *lsm)
{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an 
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:
	lsm_t->aRes = lsm_t->aScale == A_SCALE_16G ? 16.0f / 32768.0f : 
		   (((float)lsm_t->aScale + 1.0f) * 2.0f) / 32768.0f;
}

void calcmRes(struct lsm9ds0 *lsm)
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	lsm_t->mRes = lsm_t->mScale == M_SCALE_2GS ? 2.0f / 32768.0f : 
	       (float) (lsm_t->mScale << 2) / 32768.0f;
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

static int g_write(struct lsm9ds0 *lsm, uint8_t base_reg, uint8_t *data, size_t data_sz)
{
	return i2c_write(lsm->fd, lsm->g_addr, base_reg, data, data_sz);
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

static int am_write(struct lsm9ds0 *lsm, uint8_t base_reg, uint8_t *data, size_t data_sz)
{
	return i2c_write(lsm->fd, lsm->am_addr, base_reg, data, data_sz);
}
