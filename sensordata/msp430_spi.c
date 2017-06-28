#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "msp430_spi.h"

#define ARRAY_SIZE(array) sizeof(array)/sizeof(array[0])

const char *msp430_spi_DEV= "/dev/spidev32766.0";

# define TX_LEN 48
# define RX_LEN 48


struct spi_ioc_transfer xfer[2];
unsigned char buf[8];


int msp430spifd;
unsigned char spi_mode;
unsigned char spi_bitsPerWord;
unsigned int spi_speed;
unsigned char lsb;

#define MAX_SPI_SPEED 50000000  //50MHz


void WriteReg_msp430(unsigned char *txbuf, int nbytes)
{
    	unsigned short status;
	int i;
    
	xfer[0].tx_buf = (unsigned long)txbuf;
	xfer[0].len = nbytes;
	xfer[0].speed_hz = spi_speed;
	//printf("msp430spifd: %d, spi_speed:%d\n", msp430spifd, spi_speed);
	status = ioctl(msp430spifd, SPI_IOC_MESSAGE(1), xfer);
    	if (status < 0) {
                perror("WriteReg error!!!");
        }

	for (i = 0; i < 10; i++) {   
		asm("    nop");
	}
}

void ReadReg_msp430(unsigned char *rxbuf, int nbytes)
{

    	unsigned short i, status;

    	//    * tx_buf - a pointer to the data to be transferred
  	//    * rx_buf - a pointer to storage for received data
  	//    * len - length in bytes of tx and rx buffers
  	//    * speed_hz - the clock speed, in Hz
  	//    * delay_usecs - delay between last bit and deassertion of CS
  	//    * bits_per_word - override global word length for this transfer
  	//    * cs_change - strobe chip select between transfers?

	//memset(rxbuf, 0, nbytes);

	xfer[0].rx_buf = (unsigned long) rxbuf;
	xfer[0].len = nbytes;
	xfer[0].speed_hz = spi_speed;
	status = ioctl(msp430spifd, SPI_IOC_MESSAGE(1), xfer);

        if (status < 0) {
                perror("READREG");
                return;
        }


	//printf("rxbuf: %d,%d,%d\n", rxbuf[0],rxbuf[1],rxbuf[2]);
}

static int open_mspspi(unsigned int spi_speed_set)
{
	int status_value = 0;

	//open spi device
	msp430spifd = open(msp430_spi_DEV, O_RDWR);
	if (msp430spifd<=0) { 
		perror(msp430_spi_DEV);
		exit(1);
	}


    	//----- SET SPI MODE -----
    	//SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
    	//SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
    	//SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
    	//SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
    	spi_mode = SPI_MODE_1;
    
    	//----- SET BITS PER WORD -----
    	spi_bitsPerWord = 8;
    
    	//----- SET SPI BUS SPEED -----
    	spi_speed = spi_speed_set;		

	lsb = 0;

    	status_value = ioctl(msp430spifd, SPI_IOC_WR_MODE, &spi_mode);
    	if(status_value < 0)
    	{
        	perror("Could not set SPIMode (WR)...ioctl fail");
        	exit(1);
   	}
	//printf("WR_MODE: %d\n", spi_mode);

    	status_value = ioctl(msp430spifd, SPI_IOC_RD_MODE, &spi_mode);
    	if(status_value < 0)
    	{
      		perror("Could not set SPIMode (RD)...ioctl fail");
      		exit(1);
    	}
	//printf("RD_MODE: %d\n", spi_mode);

	status_value = ioctl(msp430spifd, SPI_IOC_RD_LSB_FIRST, &lsb);
	if ( status_value < 0) 
	{
                perror("SPI rd_lsb_fist");
                exit(1);
        }

    	status_value = ioctl(msp430spifd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
    	if(status_value < 0)
    	{
      		perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
      		exit(1);
    	}
	//printf("WR_BITS_PER_WORD: %d\n", spi_bitsPerWord);

    	status_value = ioctl(msp430spifd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
    	if(status_value < 0)
    	{
     		perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
      		exit(1);
    	}
	//printf("WR_BITS_PER_WORD: %d\n", spi_bitsPerWord);

    	status_value = ioctl(msp430spifd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    	if(status_value < 0)
    	{
      		perror("Could not set SPI speed (WR)...ioctl fail");
      		exit(1);
    	}
	//printf("Max WR speed: %dHz\n", spi_speed);


    	status_value = ioctl(msp430spifd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    	if(status_value < 0)
    	{
      		perror("Could not set SPI speed (RD)...ioctl fail");
      		exit(1);
    	}
	//printf("Max RD speed: %dHz\n", spi_speed);

	return status_value;

}


void MSP430_init_config(unsigned int spi_speed_set, MSP430_CFG msp430_cfg_data)
{
	int i,mpufd;
	unsigned char d;
	char wr_buf[]={0xff,0x00,0x1f,0x0f};
	char rd_buf[10];
	unsigned char mpu_ID;
	unsigned char MAG_ID;	
	unsigned int  status;

	unsigned char txbuf [ 10 ];
	unsigned char rxbuf [ 2 ];

	//open spi device
	if(spi_speed_set>MAX_SPI_SPEED) //
		spi_speed_set = MAX_SPI_SPEED;
	if(open_mspspi(spi_speed_set)<0)
	{
		perror("Error - Could not open SPI");
        	exit(1);
	}
		
	printf("MSP430 handshaking......!!!\n");

	//send initialization data
	txbuf[0] = 0x37;
	txbuf[1] = 0x56;
	txbuf[2] = msp430_cfg_data.rtc_sec;
	txbuf[3] = msp430_cfg_data.rtc_min;
	txbuf[4] = msp430_cfg_data.rtc_hr;
	txbuf[5] = msp430_cfg_data.rtc_day;
	txbuf[6] = msp430_cfg_data.rtc_mon;
	txbuf[7] = msp430_cfg_data.rtc_yr;
	txbuf[8] = msp430_cfg_data.msp430_mode;
	txbuf[9] = (((msp430_cfg_data.msp430_SAMPLING_RATE)<<4)|((msp430_cfg_data.msp430_OPTIC_on)<<3)|((msp430_cfg_data.msp430_LIS_MAG_on)<<2)|((msp430_cfg_data.msp430_MOTION_on)<<1)|msp430_cfg_data.msp430_BARO_on);
	//printf("txbuf: %d,%d,%d,%d\n", txbuf[0],txbuf[5],txbuf[6],txbuf[7]);
	
	do{
		WriteReg_msp430(txbuf,1);//A5 is ready	

		rxbuf[0] = 0;	//msp430 is ready
		ReadReg_msp430(rxbuf,1);
		//printf("hand-shaking: 0x%02x\n,", rxbuf[0]);

	}while(rxbuf[0]!=0x70);


	printf("MSP430 initializing......!!!\n");
	txbuf[0] = 0x37;
	txbuf[1] = 0x56;
	do{
		//send configuration data
		WriteReg_msp430(txbuf,10);

		rxbuf[0] = 0;
		ReadReg_msp430(rxbuf,1);
		//printf("rxbuf[0,1]: 0x%02x, 0x%02x\n", rxbuf[0],rxbuf[1]);
	}while(rxbuf[0]!=0x71);

	printf("MSP430 initialized!!!\n");

/*	//wait for sensor ready
	do{
		rxbuf[0] = 0;
		rxbuf[1] = 0;
		ReadReg_msp430(rxbuf,2);
		//printf("rxbuf[0,1]: 0x%02x, 0x%02x\n", rxbuf[0],rxbuf[1]);
	}while(rxbuf[0]!=0x70&&(rxbuf[1]!=0x72));

	printf("sensor initialized!!!\n");*/
}

/*
unsigned char MSP430_read_bt_status(unsigned int numFrames)
{

	unsigned char rxbuf [2];
	unsigned char buffer[8];

	buffer[0] = 0x37;
	buffer[1] = numFrames%256;
	buffer[2] = numFrames/256;

	//looking for header
	do{	
		//request connection status
		WriteReg_msp430(buffer, 8);

		rxbuf[0] = 0;
		rxbuf[1] = 0;
		ReadReg_msp430(rxbuf,2);
		printf("bt_status, rxbuf: 0x%x, 0x%x\n",rxbuf[0],rxbuf[1]);
	}while(rxbuf[0]!=0x73);

	return rxbuf[1];

}*/
unsigned char MSP430_read_bt_status()
{

	unsigned char rxbuf [1];

	ReadReg_msp430(rxbuf,1);
	ReadReg_msp430(rxbuf,1);
	ReadReg_msp430(rxbuf,1);
	printf("bt_status, rxbuf: 0x%x\n",rxbuf[0]);

	if(rxbuf[0]==0x35)
		return 1;
	else
		return 0;

}


void MSP430_write_bt(unsigned char *pImg, unsigned char num)
{

	//printf("pImg[0]-0x%x ",pImg[0]);

	WriteReg_msp430(pImg,num);
}

unsigned char MSP430_sensor_rd(MSP430_DATA *msp430_data)
{
	int i;
	float data;
	unsigned char error=0;
	unsigned char sensor_data_header[6]={0x73,0x74,0x75,0x76,0x77,0x78};
	unsigned char txbuf [ 2 ];
	unsigned char rxbuf [ RX_LEN ];

	unsigned char *tmp;

	rxbuf[0]=0;

	//looking for header
	do{
		rxbuf[0] = 0;
		ReadReg_msp430(rxbuf,1);
		//printf("header: 0x%02x\n,", rxbuf[0]);

	}while(rxbuf[0]!=0x70);

	//ready to receive data
	//txbuf[0] = 0x37;
	//txbuf[1] = 0x56;
	//WriteReg_msp430(txbuf,2);


	//wait a while for msp430
	/*for (i = 0; i < 100; i++) {   
		asm("    nop");
	}*/

	//receive data
	error = 0;
	
	tmp = rxbuf;
	
	for(i=0;i<47;i++)
	{
		//ReadReg_msp430(tmp+8*i,8);
		ReadReg_msp430(tmp+i,1);
				
		//printf("0x%02x, 0x%02x,", rxbuf[8*i],rxbuf[8*i+1]);


		/*if(i%2)
			txbuf[0] = 0x37;
		else
			txbuf[0] = 0x46;
		WriteReg_msp430(txbuf,1);*/
	}
	//printf("\n");

	//verify data
	for(i=0;i<6;i++)
	{		
		if(sensor_data_header[i]!=rxbuf[8*i])
		{
			error = 1;
			break;
		}
	}

	//sparse data
	if(!error)
	{
		//printf("0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", rxbuf[0],rxbuf[1],rxbuf[2],rxbuf[3], rxbuf[4],rxbuf[5],rxbuf[6],rxbuf[7]);

		msp430_data->sec 	= rxbuf[1];
		msp430_data->min 	= rxbuf[2];
		msp430_data->hour 	= rxbuf[3];
		msp430_data->MPU_ACC_mG[0] 	= (256*rxbuf[5]) + rxbuf[4];
		msp430_data->MPU_ACC_mG[1] 	= (256*rxbuf[7]) + rxbuf[6];
		msp430_data->MPU_ACC_mG[2] 	= (256*rxbuf[10]) + rxbuf[9];
		msp430_data->MPU_GYRO_10[0] 	= (256*rxbuf[12]) + rxbuf[11];
		msp430_data->MPU_GYRO_10[1] 	= (256*rxbuf[14]) + rxbuf[13];
		msp430_data->MPU_GYRO_10[2] 	= (256*rxbuf[17]) + rxbuf[15];
		msp430_data->MPU_MAG_10[0] 	= (256*rxbuf[19]) + rxbuf[18];
		msp430_data->MPU_MAG_10[1] 	= (256*rxbuf[21]) + rxbuf[20];
		msp430_data->MPU_MAG_10[2] 	= (256*rxbuf[23]) + rxbuf[22];
		msp430_data->LIS_Mag[0] 	= (256*rxbuf[26]) + rxbuf[25];
		msp430_data->LIS_Mag[1] 	= (256*rxbuf[28]) + rxbuf[27];
		msp430_data->LIS_Mag[2] 	= (256*rxbuf[30]) + rxbuf[29];
		msp430_data->BEM280_temp_Cdegree 	= (256*rxbuf[33]) + rxbuf[31];
		msp430_data->BEM280_pressure_10Pa 	= (256*rxbuf[35]) + rxbuf[34];
		msp430_data->BEM280_humidity_100percent 	= (256*rxbuf[37]) + rxbuf[36];
		msp430_data->visiblelight 	= (256*rxbuf[39]) + rxbuf[38];
		msp430_data->IRlight 	= (256*rxbuf[42]) + rxbuf[41];
		msp430_data->uv_index 	= (256*rxbuf[44]) + rxbuf[43];
		msp430_data->proximity 	= (256*rxbuf[46]) + rxbuf[44];	

		//printf("%d, %d, %d, %d\n\r",msp430_data->sec, msp430_data->MPU_ACC_mG[0],msp430_data->MPU_ACC_mG[1],msp430_data->MPU_ACC_mG[2]);
	
	//printf("%d, %d, %d, %d, (%d, %d, %d), (%d, %d, %d) \n\r",msp430_data->sec, msp430_data->MPU_ACC_mG[0],msp430_data->MPU_ACC_mG[1],msp430_data->MPU_ACC_mG[2],msp430_data->BEM280_temp_Cdegree,msp430_data->BEM280_pressure_10Pa,msp430_data->BEM280_humidity_100percent, msp430_data->visiblelight, msp430_data->uv_index,msp430_data->proximity);

	}

	return error;

}
