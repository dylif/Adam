#include <stdlib.h>
#include <stdint.h>

#ifndef LSM9DS0_H
#define LSM9DS0_H

/* lsm9ds0 gyro registers */
#define WHO_AM_I_G			0x0F
#define CTRL_REG1_G			0x20
#define CTRL_REG2_G			0x21
#define CTRL_REG3_G			0x22
#define CTRL_REG4_G			0x23
#define CTRL_REG5_G			0x24
#define REFERENCE_G			0x25
#define STATUS_REG_G		0x27
#define OUT_X_L_G			0x28
#define OUT_X_H_G			0x29
#define OUT_Y_L_G			0x2A
#define OUT_Y_H_G			0x2B
#define OUT_Z_L_G			0x2C
#define OUT_Z_H_G			0x2D
#define FIFO_CTRL_REG_G		0x2E
#define FIFO_SRC_REG_G		0x2F
#define INT1_CFG_G			0x30
#define INT1_SRC_G			0x31
#define INT1_THS_XH_G		0x32
#define INT1_THS_XL_G		0x33
#define INT1_THS_YH_G		0x34
#define INT1_THS_YL_G		0x35
#define INT1_THS_ZH_G		0x36
#define INT1_THS_ZL_G		0x37
#define INT1_DURATION_G		0x38

/* lsm9ds0 accel/mag (AM) registers */
#define OUT_TEMP_L_AM		0x05
#define OUT_TEMP_H_AM		0x06
#define STATUS_REG_M		0x07
#define OUT_X_L_M			0x08
#define OUT_X_H_M			0x09
#define OUT_Y_L_M			0x0A
#define OUT_Y_H_M			0x0B
#define OUT_Z_L_M			0x0C
#define OUT_Z_H_M			0x0D
#define WHO_AM_I_AM			0x0F
#define INT_CTRL_REG_M		0x12
#define INT_SRC_REG_M		0x13
#define INT_THS_L_M			0x14
#define INT_THS_H_M			0x15
#define OFFSET_X_L_M		0x16
#define OFFSET_X_H_M		0x17
#define OFFSET_Y_L_M		0x18
#define OFFSET_Y_H_M		0x19
#define OFFSET_Z_L_M		0x1A
#define OFFSET_Z_H_M		0x1B
#define REFERENCE_X			0x1C
#define REFERENCE_Y			0x1D
#define REFERENCE_Z			0x1E
#define CTRL_REG0_AM		0x1F
#define CTRL_REG1_AM		0x20
#define CTRL_REG2_AM		0x21
#define CTRL_REG3_AM		0x22
#define CTRL_REG4_AM		0x23
#define CTRL_REG5_AM		0x24
#define CTRL_REG6_AM		0x25
#define CTRL_REG7_AM		0x26
#define STATUS_REG_A		0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D
#define FIFO_CTRL_REG		0x2E
#define FIFO_SRC_REG		0x2F
#define INT_GEN_1_REG		0x30
#define INT_GEN_1_SRC		0x31
#define INT_GEN_1_THS		0x32
#define INT_GEN_1_DURATION	0x33
#define INT_GEN_2_REG		0x34
#define INT_GEN_2_SRC		0x35
#define INT_GEN_2_THS		0x36
#define INT_GEN_2_DURATION	0x37
#define CLICK_CFG			0x38
#define CLICK_SRC			0x39
#define CLICK_THS			0x3A
#define TIME_LIMIT			0x3B
#define TIME_LATENCY		0x3C
#define TIME_WINDOW			0x3D
#define ACT_THS				0x3E
#define ACT_DUR				0x3F

/* gyro_scale defines the possible full-scale ranges of the gyroscope:
 * 00:  245 degrees per second
 * 01:  500 dps
 * 10:  2000 dps
 */
enum gyro_scale
{
	G_SCALE_245DPS,		
	G_SCALE_500DPS,
	G_SCALE_2000DPS,
};

/* accel_scale defines all possible FSR's of the accelerometer:
 * 000:  2g
 * 001:  4g
 * 010:  6g
 * 011:  8g
 * 100:  16g
 */
enum accel_scale
{
	A_SCALE_2G,	
	A_SCALE_4G,	
	A_SCALE_6G,	
	A_SCALE_8G,	
	A_SCALE_16G	
};

/* mag_scale defines all possible FSR's of the magnetometer:
 * 00:  2Gs
 * 01:  4Gs
 * 10:  8Gs
 * 11:  12Gs
 */
enum mag_scale
{
	M_SCALE_2GS,	
	M_SCALE_4GS, 	
	M_SCALE_8GS,	
	M_SCALE_12GS,	
};

/* gyro_odr defines all possible data rate/bandwidth combos of the gyro:
 * ODR (Hz) --- Cutoff
 * 95           12.5
 * 95           25
 * 0x2 and 0x3 define the same data rate and bandwidth
 * 190          12.5
 * 190          25
 * 190          50
 * 190          70
 * 380          20
 * 380          25
 * 380          50
 * 380          100
 * 760          30
 * 760          35
 * 760          50
 * 760          100
 */
enum gyro_odr
{							
	G_ODR_95_BW_125  = 0x0, 
	G_ODR_95_BW_25   = 0x1,    
	
	G_ODR_190_BW_125 = 0x4,
	G_ODR_190_BW_25  = 0x5,
	G_ODR_190_BW_50  = 0x6,
	G_ODR_190_BW_70  = 0x7,
	G_ODR_380_BW_20  = 0x8,    
	G_ODR_380_BW_25  = 0x9,    
	G_ODR_380_BW_50  = 0xA,    
	G_ODR_380_BW_100 = 0xB,    
	G_ODR_760_BW_30  = 0xC,    
	G_ODR_760_BW_35  = 0xD,    
	G_ODR_760_BW_50  = 0xE,    
	G_ODR_760_BW_100 = 0xF,    
};

/* accel_oder defines all possible output data rates of the accelerometer:
 * Power-down mode (0x0)
 * 3.125 Hz	(0x1)
 * 6.25 Hz (0x2)
 * 12.5 Hz (0x3)
 * 25 Hz (0x4)
 * 50 Hz (0x5)
 * 100 Hz (0x6)
 * 200 Hz (0x7)
 * 400 Hz (0x8)
 * 800 Hz (9)
 * 1600 Hz (0xA)
 */
enum accel_odr
{
	A_POWER_DOWN, 	
	A_ODR_3125,		
	A_ODR_625,		
	A_ODR_125,		
	A_ODR_25,		
	A_ODR_50,		
	A_ODR_100,		
	A_ODR_200,		
	A_ODR_400,		
	A_ODR_800,		
	A_ODR_1600		
};

/* accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
 * 773 Hz (0x0)
 * 194 Hz (0x1)
 * 362 Hz (0x2)
 * 50 Hz (0x3)
 */
enum accel_abw
{
	A_ABW_773,		
	A_ABW_194,		
	A_ABW_362,		
	A_ABW_50,		
};


/* mag_oder defines all possible output data rates of the magnetometer:
 * 3.125 Hz (0x00)
 * 6.25 Hz (0x01)
 * 12.5 Hz (0x02)
 * 25 Hz (0x03)
 * 50 (0x04)
 * 100 Hz (0x05)
 */
enum mag_odr
{
	M_ODR_3125,	
	M_ODR_625,	
	M_ODR_125,	
	M_ODR_25,	
	M_ODR_50,	
	M_ODR_100,	
};

/* declare structs */
struct lsm9ds0
{	
	int fd;
		
	unsigned int g_addr;
	unsigned int am_addr;

	int g_scl;
	int a_scl;
	int m_scl;
	
	float g_res;
	float a_res;
	float m_res;

	/* these will be updated when lsm_update is called */
	float gx;
	float gy; 
	float gz;
	float ax;
	float ay; 
	float az;
	float mx; 
	float my; 
	float mz;
	
	/* members to store raw readings from the lsm */
	int16_t gx_raw;
	int16_t	gy_raw; 
	int16_t	gz_raw;
	int16_t ax_raw;
	int16_t	ay_raw; 
	int16_t	az_raw;
	int16_t mx_raw; 
	int16_t	my_raw; 
	int16_t	mz_raw;
    int16_t temp;
    
    /* arrays to store gyro and accel biases 
     * 0: X bias
     * 1: Y bias
     * 2: Z bias
     */
    float g_bias[3];
	float a_bias[3];
};

struct lsm9ds0_settings
{
	int	g_scl;
	int a_scl;
	int m_scl;
	int g_odr; 
	int a_odr; 
	int m_odr;
};

/* declare functions */
uint16_t lsm9ds0_new(struct lsm9ds0 *lsm, struct lsm9ds0_settings *settings, int fd, unsigned int g_addr, unsigned int am_addr);
int lsm9ds0_gyro_init(struct lsm9ds0 *lsm);
int lsm9ds0_gyro_read(struct lsm9ds0 *lsm);
int lsm9ds0_accel_init(struct lsm9ds0 *lsm);
int lsm9ds0_accel_read(struct lsm9ds0 *lsm);
int lsm9ds0_mag_init(struct lsm9ds0 *lsm);
int lsm9ds0_mag_read(struct lsm9ds0 *lsm);
int lsm9ds0_temp_read(struct lsm9ds0 *lsm);
int lsm9ds0_cal(struct lsm9ds0 *lsm);
void lsm_update(struct lsm9ds0 *lsm);
float calc_gyro(struct lsm9ds0 *lsm, int16_t gyro);
float calc_accel(struct lsm9ds0 *lsm, int16_t accel);
float calc_mag(struct lsm9ds0 *lsm, int16_t mag);
int set_g_scl(struct lsm9ds0 *lsm, int g_scl);
int set_a_scl(struct lsm9ds0 *lsm, int a_scl);
int set_m_scl(struct lsm9ds0 *lsm, int m_scl);
int set_g_odr(struct lsm9ds0 *lsm, int g_rate);
int set_a_odr(struct lsm9ds0 *lsm, int a_rate);
int set_a_abw(struct lsm9ds0 *lsm, int abw_rate);
int set_m_odr(struct lsm9ds0 *lsm, int m_rate);
int lsm9ds0_gyro_cfg_int(struct lsm9ds0 *lsm, uint8_t int1_cfg, 
	uint16_t int1_thsx, uint16_t int1_thsy, uint16_t int1_thsz, uint8_t duration);
int calc_g_res(struct lsm9ds0 *lsm);
int calc_a_res(struct lsm9ds0 *lsm);
int calc_m_res(struct lsm9ds0 *lsm);

#endif /* LSM9DS0_H */
