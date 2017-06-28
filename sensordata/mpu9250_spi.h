#ifndef MPU9250_SPI_H_
#define MPU9250_SPI_H_


/*register and associated bit definition*/
#define INV_MPU9250_REG_XA_OFFSET_H         0x77
#define INV_MPU9250_REG_XG_OFFSET_H         0x13

#define INV_MPU9250_REG_SAMPLE_RATE_DIV     0x19
#define INV_MPU9250_REG_CONFIG              0x1A
#define INV_MPU9250_REG_GYRO_CONFIG         0x1B
#define INV_MPU9250_REG_ACCEL_CONFIG        0x1C
#define INV_MPU9250_REG_ACCEL_CONFIG2       0x1D

#define INV_MPU9250_REG_FIFO_EN             0x23
#define INV_MPU9250_BIT_ACCEL_OUT           0x08
#define INV_MPU9250_BITS_GYRO_OUT           0x70
#define INV_MPU9250_BITS_TEMP_OUT           0x80

#define INV_MPU9250_REG_I2C_MST_CTRL        0x24
#define INV_MPU9250_REG_I2C_SLV0_ADDR       0x25
#define INV_MPU9250_REG_I2C_SLV0_REG        0x26
#define INV_MPU9250_REG_I2C_SLV0_CTRL       0x27
#define INV_MPU9250_EXT_SENS_DATA_00    0x49
#define INV_MPU9250_EXT_SENS_DATA_01    0x4A
#define INV_MPU9250_EXT_SENS_DATA_02    0x4B
#define INV_MPU9250_EXT_SENS_DATA_03    0x4C
#define INV_MPU9250_EXT_SENS_DATA_04    0x4D
#define INV_MPU9250_EXT_SENS_DATA_05    0x4E
#define INV_MPU9250_EXT_SENS_DATA_06    0x4F
#define INV_MPU9250_EXT_SENS_DATA_07    0x50
#define INV_MPU9250_REG_I2C_SLV0_DO         0x63

#define INV_MPU9250_REG_INT_PIN_CFG	0x37
#define INV_MPU9250_BIT_BYPASS_EN	0x2
#define INV_MPU9250_INT_PIN_CFG		0


#define INV_MPU9250_REG_INT_ENABLE          0x38
#define INV_MPU9250_BIT_DATA_RDY_EN         0x01
#define INV_MPU9250_BIT_FIFO_OFLOW_EN       0x10
#define INV_MPU9250_BIT_FSYNC_INT_EN        0x08
//#define INV_MPU9250_BIT_DMP_INT_EN          0x02

#define INV_MPU9250_REG_INT_STATUS          0x3A

#define INV_MPU9250_REG_RAW_ACCEL           0x3B
#define INV_MPU9250_REG_TEMPERATURE         0x41
#define INV_MPU9250_REG_RAW_GYRO            0x43

#define INV_MPU9250_REG_USER_CTRL           0x6A
#define INV_MPU9250_BIT_SIG_COND_RST	    0x01
#define INV_MPU9250_BIT_I2C_MST_RST         0x02
#define INV_MPU9250_BIT_FIFO_RST            0x04
#define INV_MPU9250_BIT_DMP_RST             0x08
#define INV_MPU9250_BIT_I2C_IF_DIS          0x10
#define INV_MPU9250_BIT_I2C_MST_EN          0x20
#define INV_MPU9250_BIT_FIFO_EN             0x40
//#define INV_MPU9250_BIT_DMP_EN              0x80


#define INV_MPU9250_REG_PWR_MGMT_1          0x6B
#define INV_MPU9250_BIT_H_RESET             0x80
#define INV_MPU9250_BIT_SLEEP               0x40
#define INV_MPU9250_BIT_CLK_MASK            0x7

#define INV_MPU9250_REG_PWR_MGMT_2          0x6C
#define INV_MPU9250_BIT_PWR_ACCL_STBY       0x38
#define INV_MPU9250_BIT_PWR_GYRO_STBY       0x07

#define INV_MPU9250_REG_FIFO_COUNT_H        0x72
#define INV_MPU9250_REG_FIFO_R_W            0x74
#define INV_MPU9250_REG_WHO_AM_I	    0x75



#define INV_MPU9250_BYTES_PER_3AXIS_SENSOR   6
#define INV_MPU9250_FIFO_COUNT_BYTE          2
#define INV_MPU9250_FIFO_THRESHOLD           500

/* mpu6500 registers */
//#define INV_MPU6500_REG_ACCEL_OFFSET        0x77

/* delay time in milliseconds */
#define INV_MPU9250_POWER_UP_TIME            100
#define INV_MPU9250_TEMP_UP_TIME             100
#define INV_MPU9250_SENSOR_UP_TIME           30

/* delay time in microseconds */
#define INV_MPU9250_REG_UP_TIME_MIN          5000
#define INV_MPU9250_REG_UP_TIME_MAX          10000

#define INV_MPU9250_TEMP_OFFSET	             12421
#define INV_MPU9250_TEMP_SCALE               2941
#define INV_MPU9250_MAX_GYRO_FS_PARAM        3
#define INV_MPU9250_MAX_ACCL_FS_PARAM        3
#define INV_MPU9250_THREE_AXIS               3
#define INV_MPU9250_GYRO_CONFIG_FSR_SHIFT    3
#define INV_MPU9250_ACCL_CONFIG_FSR_SHIFT    3

/* 6 + 6 round up and plus 8 */
#define INV_MPU9250_OUTPUT_DATA_SIZE         24


/* init parameters */
#define INV_MPU9250_INIT_FIFO_RATE           50
#define INV_MPU9250_TIME_STAMP_TOR           5
#define INV_MPU9250_MAX_FIFO_RATE            1000
#define INV_MPU9250_MIN_FIFO_RATE            4
#define INV_MPU9250_ONE_K_HZ                 1000


/**
 *  struct inv_mpu9250_reg_map - Notable registers.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal low pass filter.
 *  @user_ctrl:		Enables/resets the FIFO.
 *  @fifo_en:		Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accl_config:	accel config register
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:		FIFO register.
 *  @raw_gyro:		Address of first gyro register.
 *  @raw_accl:		Address of first accel register.
 *  @temperature:	temperature register
 *  @int_enable:	Interrupt enable register.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 *  @int_pin_cfg;	Controls interrupt pin configuration.
 *  @accl_offset:	Controls the accelerometer calibration offset.
 *  @gyro_offset:	Controls the gyroscope calibration offset.
 */
struct inv_mpu9250_reg_map {
	unsigned char sample_rate_div;
	unsigned char lpf;
	unsigned char user_ctrl;
	unsigned char fifo_en;
	unsigned char i2c_mst_ctrl;
	unsigned char i2c_slv0_addr;
	unsigned char i2c_slv0_reg;
	unsigned char i2c_slv0_ctrl;
	unsigned char i2c_slv0_DO;
	unsigned char i2c_ext_sens_data0;
	unsigned char i2c_ext_sens_data1;
	unsigned char i2c_ext_sens_data2;
	unsigned char i2c_ext_sens_data3;
	unsigned char i2c_ext_sens_data4;
	unsigned char i2c_ext_sens_data5;
	unsigned char i2c_ext_sens_data6;
	unsigned char i2c_ext_sens_data7;
	unsigned char gyro_config;
	unsigned char accl_config;
	unsigned char accl_config2;
	unsigned char fifo_count_h;
	unsigned char fifo_r_w;
	unsigned char raw_gyro;
	unsigned char raw_accl;
	unsigned char temperature;
	unsigned char raw_asa;
	unsigned char raw_mag;
	unsigned char int_pin_cfg;
	unsigned char int_enable;
	unsigned char int_status;
	unsigned char pwr_mgmt_1;
	unsigned char pwr_mgmt_2;	
	unsigned char who_am_I;
	unsigned char accl_offset;
	unsigned char gyro_offset;
};

/* scan element definition */
enum inv_mpu9250_scan {
	INV_MPU9250_SCAN_ACCL_X,
	INV_MPU9250_SCAN_ACCL_Y,
	INV_MPU9250_SCAN_ACCL_Z,
	INV_MPU9250_SCAN_GYRO_X,
	INV_MPU9250_SCAN_GYRO_Y,
	INV_MPU9250_SCAN_GYRO_Z,
	INV_MPU9250_SCAN_MAG_X,
	INV_MPU9250_SCAN_MAG_Y,
	INV_MPU9250_SCAN_MAG_Z,	
	INV_MPU9250_SCAN_TIMESTAMP,
};

enum inv_mpu9250_filter_e {
	INV_MPU9250_FILTER_256HZ_NOLPF2 = 0,
	INV_MPU9250_FILTER_184HZ,
	INV_MPU9250_FILTER_92HZ,
	INV_MPU9250_FILTER_41HZ,
	INV_MPU9250_FILTER_20HZ,
	INV_MPU9250_FILTER_10HZ,
	INV_MPU9250_FILTER_5HZ,
	INV_MPU9250_FILTER_2100HZ_NOLPF,
	NUM_MPU9250_FILTER
};

typedef struct  
{
	char	SensorHour;
	char 	SensorMin;
	char	SensorSec;	
	short 	ACC_OUT[3];
	short 	TEMP_OUT;
	short 	GYRO_OUT[3];
        float 	ACC_fl[3];
	float   TEMP_fl;
	float   GYRO_fl[3];
	short 	Mag_out[3];
        float 	Magnetometer[3];
} MPU9250_sensor;

/*device enum */
enum inv_devices {
	INV_MPU9250,
	INV_MPU6500,
	INV_MPU6000,
	INV_NUM_PARTS
};


enum inv_mpu9250_accl_fs_e {
	INV_MPU9250_FS_02G = 0,
	INV_MPU9250_FS_04G,
	INV_MPU9250_FS_08G,
	INV_MPU9250_FS_16G,
	NUM_ACCL_FSR
};

enum inv_mpu9250_fsr_e {
	INV_MPU9250_FSR_250DPS = 0,
	INV_MPU9250_FSR_500DPS,
	INV_MPU9250_FSR_1000DPS,
	INV_MPU9250_FSR_2000DPS,
	NUM_MPU9250_FSR
};


/**
 *  struct inv_mpu9250_chip_config - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accl_fs:		accel full scale range.
 *  @enable:		master enable state.
 *  @accl_fifo_enable:	enable accel data output
 *  @gyro_fifo_enable:	enable gyro data output
 *  @fifo_rate:		FIFO update rate.
 */
struct inv_mpu9250_chip_config {
	unsigned int fsr:2;
	unsigned int lpf:3;
	unsigned int accl_fs:1;
	unsigned int enable:1;
	unsigned int accl_fifo_enable:1;
	unsigned int gyro_fifo_enable:1;
	unsigned short fifo_rate;
};



#define READ_FLAG       0x80
#define WRITE_FLAG      0x00

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
#define AK8963_I2C_ADDR             0x0c
#define AK8963_Device_ID            0x48
 
// Read-only Reg
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12
 
// Configuration bits mpu9250
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

/* ---- Sensitivity --------------------------------------------------------- */
 
#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB
 
#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB
 
#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB
 
#define MPU9250T_85degC   ((float)0.002995177763f) // 0.002995177763 degC/LSB
#define     Magnetometer_Sensitivity_Scale_Factor_14bits ((float)0.6f)  //14bits
#define     Magnetometer_Sensitivity_Scale_Factor_16bits ((float)0.15f)  //16bits

#define MAGNETIC_14bits                                 0x00
#define MAGNETIC_16bits                                 0x10
#define MAGNETIC_PWD					0x00
#define MAGNETIC_SINGLE_MEASUREMENT		0x01            //time of measurement = 7.2ms, automatically go to pwd mode
#define MAGNETIC_CONT1_MEASUREMENT		0x02            //8Hz
#define MAGNETIC_CONT2_MEASUREMENT		0x06            //100Hz
#define MAGNETIC_SELF_TEST				0x08
#define MAGNETIC_FUSE_ROM				0x0F


void inv_mpu9250_init_config(unsigned int spi_speed_set, unsigned char gyro_fs, unsigned char acc_fs);
void Motion_MPU9250_REGs_rd(MPU9250_sensor *MPU9250_DATA);


#endif
