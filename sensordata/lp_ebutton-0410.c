/*
 * lpebutton_cam_App.c
 *
 *  Created on: Jun 25, 2016
 *      Author: root
 */

#include <stdio.h>
#include <fcntl.h>
#include <linux/input.h>  
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <linux/i2c-dev.h>
#include <sys/syscall.h>

#include <dirent.h>
#include <pthread.h>

//#include <iio.h>

#include <signal.h>
//#include "JPGApi.h"
#include <semaphore.h>

#include <dirent.h>
//#include "lp_ebutton.h"

#include <sys/soundcard.h> 

#include "global_def.h"
#include "get_time.h"
#include "mpu9250_spi.h"

#include "msp430_spi.h"

#include "timer.h"
//#include "timer_imu.h"


#include "opus.h"
/*The frame size is hardcoded for this sample code but it doesn't have to be*/
#define FRAME_SIZE 960
#define SAMPLE_RATE 24000
#define CHANNELS 1
#define APPLICATION OPUS_APPLICATION_AUDIO
#define BITRATE 24000

#define MAX_FRAME_SIZE 6*FRAME_SIZE
#define MAX_PACKET_SIZE (3*1276)

// Mandatory variables for audio 
#define Format	16
#define NUM_FRAMES_PER_SEC	SAMPLE_RATE/FRAME_SIZE	
#define RECORDING_SEC	5	//encode data every 5 seconds
#define BUF_SIZE NUM_FRAMES_PER_SEC*RECORDING_SEC*FRAME_SIZE*(CHANNELS*Format/8) 

/*Holds the state of the encoder and decoder */
OpusEncoder *encoder;
//OpusDecoder *decoder;
int err;

unsigned char *audio_buffer;// = (unsigned char *)malloc( BUF_SIZE * sizeof(unsigned char)); 
unsigned char new_audio_ppflag = 0;
unsigned char new_audio_data=0;

#define LIGHT 0 //light for sdcard

/***************** etc *******************/

#define MFC_LINE_BUF_SIZE_PER_INSTANCE	(204800)
#define YUV_FRAME_NUM	100

const char *imu_DEV= "/dev/iio:device1";//"/sys/bus/iio/devices/iio:device1/in_accel_z_raw";//
const char *adc_DEV= "/dev/adc";
const char *gps_DEV= "/dev/ttySAC1";
const char *cam_DEV = "/dev/video0";
const char *clock_control_DEV = "/dev/clock_control";
const char *proximity_DEV= "/dev/proximity";
const char *dmic_DEV = "/dev/dsp";
/*
unsigned char moDayDir_t_flag=0;
char moDayDir_t[MAXSTRING];
char moDayDir_mcu[MAXSTRING];
char moDayDir_sensor[MAXSTRING];
*/

#define JPG	"image%d.jpg"
int IM_WIDTH=1280;
int IM_HEIGHT=720;

char deviceID[5]={"0"};
int camFd;
int gpsFd;
int proximityFd;
int imuFd;
int audioFd; 
	
struct time_comp
{
	char tm[MAXSTRING];
	int indx;
};
struct time_comp pictimecomp={"000000000",0};

int jpg_ctrl=0;

int cnt_pics=0;

//unsigned char stop_flag=0;

typedef struct{
	unsigned char *start;
	int length;
}BUFTYPE;
BUFTYPE *usr_buf;
static unsigned int n_buffer = 0;

char low_power_mode = 1;
char image_encryption_en = 0;

//pingpong buffer for sensor data
unsigned char flag_newdata=0;
unsigned char flag_sensorbuffer=0;
#define LEN_SENSOR	256//256
MPU9250_sensor MPU9250_DATA[2*LEN_SENSOR];


//pingpong buffer for mcu sensor data
unsigned char flag_newdata_mcu=0;
unsigned char flag_sensorbuffer_mcu=0;
#define LEN_SENSOR_MCU	64
MSP430_DATA  MCU_SENSOR_DATA[2*LEN_SENSOR_MCU];

//unsigned char MCU_SENSOR_ON=0;
//unsigned char MCU_BT_IMAGE_TRANS_ON=0;

unsigned char imu_file_open=0;
unsigned char mcu_file_open=0;
unsigned char audio_file_open=0;

pthread_mutex_t lock;
pthread_t imuThread,imusaveThread, msp430imageThread, msp430Thread, msp430saveThread, camThread,gpsThread,dmicThread, dmicThread_save, poweronoffThread, lowbatteryThread;

volatile option_struct  global_options;

unsigned char BT_connected=0;
unsigned char *pImage;
unsigned char *pImage_a;
unsigned char *pImage_b;
unsigned char pingpong_bt = 0;

unsigned char IMG_CNT = 0;
unsigned char BT_IMG_EN = 0;

unsigned int numFrames = 0;
unsigned int numTails=0;
unsigned char image4bt_ready = 0;
unsigned char image_trans_ing = 0;

unsigned char buffer_bt[48];
unsigned char Size_frame=16;
unsigned char numHeader = 4;
unsigned char test_data=0x41;

#define PMIC_I2C_ADDRESS 0x5b


//timer
int var_imu=0;


int cnt_sample_imu=0;

typedef struct TH_PARAMS
{
    uint threadNum;
    pthread_t tid;
    timer_t *timer;
    struct sigevent *event;
} ThreadParams_t;

#define NUM_TIMER 10
static timer_t timers[NUM_TIMER];
static struct itimerspec timeToWait[NUM_TIMER];
static struct sigevent events[NUM_TIMER];
static ThreadParams_t thParams[NUM_TIMER];
static char wait[NUM_TIMER];
/*
static void *threadTask(void *params)
{
    ThreadParams_t *threadParams = (ThreadParams_t *) params;

    printf("Thread num %d. Thread %ld. Pthread %ld.\n",
       threadParams->threadNum, syscall(SYS_gettid),
       pthread_self());

    if (0 != timer_settime(*(threadParams->timer), 0, &timeToWait[threadParams->threadNum], NULL))
    {
    printf("Failed to set timers. Error %d.\n", errno);
    pthread_exit(NULL);
    }

    while(wait) sleep(1);

    pthread_exit(NULL);
}
*/

int open_camera_device()
{
	int fd;
	//open video device with block
	fd = open(cam_DEV, O_RDWR);//O_RDONLY);
	if(fd < 0){
		perror(cam_DEV);
		exit(EXIT_FAILURE);
	}
	return fd;
}

//set video capture ways(mmap)
int init_mmap(int fd)
{
	//to request frame cache, contain requested counts
	struct v4l2_requestbuffers reqbufs;
	//request V4L2 driver allocation video cache
	//this cache is locate in kernel and need mmap mapping
	memset(&reqbufs, 0, sizeof(reqbufs));
	reqbufs.count = 1;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbufs.memory = V4L2_MEMORY_MMAP;

	if(-1 == ioctl(fd,VIDIOC_REQBUFS,&reqbufs)){
		perror("Fail to ioctl 'VIDIOC_REQBUFS'");
		exit(EXIT_FAILURE);
	}

	n_buffer = reqbufs.count;
	printf("n_buffer = %d\n", n_buffer);
	usr_buf = calloc(reqbufs.count, sizeof(usr_buf));
	if(usr_buf == NULL){
		printf("Out of memory\n");
		exit(-1);
	}

	//map kernel cache to user process 
	for(n_buffer = 0; n_buffer < reqbufs.count; ++n_buffer){
		//stand for a frame
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffer;
		
		//check the information of the kernel cache requested 
		if(-1 == ioctl(fd,VIDIOC_QUERYBUF,&buf))
		{
			perror("Fail to ioctl : VIDIOC_QUERYBUF");
			exit(EXIT_FAILURE);
		}

		usr_buf[n_buffer].length = buf.length;
		usr_buf[n_buffer].start = //mmap(
			(unsigned char *)mmap(
					NULL,
					buf.length,
					PROT_READ | PROT_WRITE,
					MAP_SHARED,//MAP_PRIVATE,
					fd,
					buf.m.offset
				);

		if(MAP_FAILED == usr_buf[n_buffer].start)
		{
			perror("Fail to mmap");
			exit(EXIT_FAILURE);
		}
	}
	return 0;
}

//initial camera device 
int init_camera_device(int fd)
{
	//decive fuction, such as video input
	struct v4l2_capability cap;
	//video standard,such as PAL,NTSC
	//struct v4l2_standard std;
	//frame format
	struct v4l2_format tv_fmt;
	//check control
	//struct v4l2_queryctrl query;
	//detail control value
	struct v4l2_fmtdesc fmt;
	int ret;
	//get the format of video supply
	memset(&fmt, 0, sizeof(fmt));
	fmt.index = 0;
	//supply to image capture
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	// show all format of supply
	/*printf("Support format:\n");
	while(ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0){
		fmt.index++;
		printf("pixelformat = ''%c%c%c%c''\ndescription = ''%s''\n",fmt.pixelformat & 0xFF, (fmt.pixelformat >> 8) & 0xFF,(fmt.pixelformat >> 16) & 0xFF, (fmt.pixelformat >> 24) & 0xFF,fmt.description);
	}*/
	//check video decive driver capability
/*	ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
	if(ret < 0){
		perror("Fail to ioctl VIDEO_QUERYCAP");
		exit(EXIT_FAILURE);
	}

	//judge wherher or not to be a video-get device
	if(!(cap.capabilities & V4L2_BUF_TYPE_VIDEO_CAPTURE))
	{
		printf("The Current device is not a video capture device\n");
		exit(-1);
	}

	//judge whether or not to supply the form of video stream
	if(!(cap.capabilities & V4L2_CAP_STREAMING))
	{
		printf("The Current device does not support streaming i/o\n");
		exit(EXIT_FAILURE);
	}
*/
	//set the form of camera capture data
	tv_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	tv_fmt.fmt.pix.width = IM_WIDTH/2;//680;
	tv_fmt.fmt.pix.height = IM_HEIGHT;//480;
	tv_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

	if (ioctl(fd, VIDIOC_S_FMT, &tv_fmt)< 0) {
		printf("VIDIOC_S_FMT\n");
		exit(-1);
		close(fd);
	}

	//initial video capture way(mmap)
	//init_mmap(fd);

	return 0;
}


int start_capture(int fd)
{
	unsigned int i;
	enum v4l2_buf_type type;
	//place the kernel cache to a queue
	for(i = 0; i < n_buffer; i++){
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if(-1 == ioctl(fd, VIDIOC_QBUF, &buf)){
			perror("Fail to ioctl 'VIDIOC_QBUF'");
			exit(EXIT_FAILURE);
		}
	}

	//start capture data
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(-1 == ioctl(fd, VIDIOC_STREAMON, &type)){
		printf("i=%d.\n", i);
		perror("VIDIOC_STREAMON");
		close(fd);
		exit(EXIT_FAILURE);
	}

	//printf("start_capture...\n");	

	return 0;
}

void moto_ctrl(int alert)
{
	int i;

	/*FILE *gpioE11_fd;
	char gpioE11_value[1];
	unsigned char v=0;

	gpioE11_value[0] = on;
	gpioD14_fd = fopen("/sys/class/gpio/pioD14/value", "rb");
	fread(gpioD14_value, sizeof(char), 1, gpioD14_fd);
	fclose(gpioD14_fd);

	v = atoi(gpioD14_value);
	//if(v==1) printf("%d,",v);*/
	
	if(alert==1)
	{
		system("echo 1 > /sys/class/gpio/pioE11/value");
		for(i = 0; i < 20000000; i++)
		{;}
		system("echo 0 > /sys/class/gpio/pioE11/value");
	}


}


int process_image(unsigned char *addr, int length, char *imFile)
{
	FILE *fp;
	
	unsigned int i,k;
	unsigned short valid_bytes_line;
	unsigned char *jpg_file;// = malloc(IM_WIDTH*IM_HEIGHT*sizeof(unsigned char));
	unsigned char *jpg_raw;
	unsigned int jpg_size;
	unsigned char *line_end;
	unsigned int cnt_trigger=0;

	{
		jpg_raw = (unsigned char *)addr;
		jpg_file = (unsigned char *)addr;
		jpg_size = 0;


		//printf("process_image...\n");
		//if(((jpg_raw[0]==0xff)&&(jpg_raw[1]==0xD8))&&(!((jpg_raw[1280]==0xff)&&(jpg_raw[1281]==0xD8))))//check valid data
		//if((jpg_raw[0]==0xFF)&&(jpg_raw[1]==0xD8))
		{	 
			if(low_power_mode!=0) 
				system("echo 1 > /sys/class/leds/led_green/brightness");
 
			for(i=0; i<IM_HEIGHT; i++)
			{
				//printf("jpg_mode: %d\n",i);
			
				//calculate size of valid data on current line
				line_end = jpg_raw+(IM_WIDTH-1);
				valid_bytes_line = ((*(line_end-1))<<8)+(*line_end);
				if(valid_bytes_line>IM_WIDTH)
					break;
		
				//copy valid data
				//printf("jpg_mode: %d\n",valid_bytes_line);
				memcpy(jpg_file,jpg_raw,valid_bytes_line);

				jpg_raw += IM_WIDTH;
			    	jpg_file += valid_bytes_line;
			    	jpg_size += valid_bytes_line;
			} 

			//printf("jpeg:%02x%02x, buffer_length: %d, jpg_size: %d\n",addr[0], addr[1], length, jpg_size);


			//while(flag_newdata)//wait for saving sensor data to be done
			//	;

			if((fp = fopen(imFile, "wb")) == NULL){
				perror("Fail to fopen");
				exit(EXIT_FAILURE);
			}
	
			//printf("addr jpg data: 0x%02x,0x%02x,0x%02x, --- 0x%02x,0x%02x,0x%02x\n",addr[0], addr[1], addr[2], addr[1280], addr[1281], addr[1282]);
			//look for FFD8
			if((addr[1]==0xFF)&&(addr[2]==0xD8))
			{
				addr++;
				jpg_size--;
			}

			//image encryption
			if(image_encryption_en)
			{
				addr[0]=0;
			//	printf("encryption, addr[0]: 0x%x\n", addr[0]);
			}

			//write file
			fwrite(addr, 4, ((jpg_size+3)/4), fp);
			//usleep(500);
			//fflush(fp);

			fclose(fp);

			/**********************************************motor control****************************************************/
			//alert user when camera file is too small, say 24kB
			if(low_power_mode != 0)
				cnt_trigger = 5;
			else
				cnt_trigger = 100;
			
			if(jpg_size<24000)
			{
				IMG_CNT++;

				//confirmation by using 3 images, and alerm every 5 images
				if(IMG_CNT == cnt_trigger)
				{
					moto_ctrl(1);
					IMG_CNT = 0;
				}
			}
			else
				IMG_CNT = 0;
			/*************************************************************************************************************/

			printf("ebutton, jpg_size: %d\n", jpg_size);
			printf(imFile);
			printf("\n");
			
			//BT trans
			if((global_options.MCU_MODE==2)&(BT_IMG_EN>=2)){

				
				image4bt_ready = 0;
					
				//update image data for BT only when BT is not transferring
				printf("previous image sending is done?  ");

				//while(image_trans_ing);

				//if(!image_trans_ing)
				{
					printf("Yes\n");
					//ready data from msp430 to check bt connection
					if(BT_connected!=1)
					{
						//check BT status
						BT_connected = MSP430_read_bt_status();//numFrames);
						printf("check BT STATUS done!\n");
					}

					//need to send latest image to phone			
					printf("MCU MODE: %d, BT_connected: %d\n",global_options.MCU_MODE,BT_connected);
					if(BT_connected==1){
						
						memcpy(pImage, addr, jpg_size);
						/*if(pingpong_bt==0)
							memcpy(pImage_a, addr, jpg_size);
						else
							memcpy(pImage_b, addr, jpg_size);*/

						numFrames = (jpg_size/Size_frame)+1;
						numTails = jpg_size%Size_frame;
						
						//for test
						//numFrames = 1000;

						//image4bt_ready = 1;
					}
				}

				//else
				//	printf("No\n");		
			}

			BT_IMG_EN++;

			if(low_power_mode!=0) 
				system("echo 0 > /sys/class/leds/led_green/brightness");


			//cnt_pics++;
			//if(cnt_pics%128==0)
			//	system("sync");
		}
		
	}
	
	return 0;
}

int read_frame(int fd, char save_en)
{
	char tm_pre[MAXSTRING];
	char tm[MAXSTRING];
	static Current_tm time_struct={"0000","000","00","00","00","00"};
	char sys_cmd[MAXSTRING];
	char cmd[MAXSTRING];
	char moDayDir[MAXSTRING];
	char hourDir[MAXSTRING];
	char FOLDER_NAME[MAXSTRING];
	char yrMonDayHrMinSec[MAXSTRING];
	char imFile[MAXSTRING];

	struct v4l2_buffer buf;
	//unsigned int i;
	memset(&buf, 0, sizeof(buf));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	//put cache from queue
	if(-1 == ioctl(fd, VIDIOC_DQBUF,&buf)){
		perror("Fail to ioctl 'VIDIOC_DQBUF'");
		exit(EXIT_FAILURE);
	}

	//assert(buf.index < n_buffer);
	//read process space's data to a file

	if(save_en)//
	{

		//************************************************* Path generation **************************************************************************//
		get_time(&time_struct);
		

		//reboot every hour
		//if((atoi(time_struct.time_min)==0)&&(atoi(time_struct.time_sec)==0))
		//	system("reboot");

		//multi frames per second
		sprintf(tm,"%s%c%s",time_struct.time_min,'.',time_struct.time_sec);
		if (!strcmp(pictimecomp.tm, tm))
		{
			pictimecomp.indx++;
		}
		else
		{
			pictimecomp.indx=0;
			strcpy(pictimecomp.tm, tm);


			strcpy(moDayDir,PATH);
			//printf("moDayDir is %s\n",moDayDir);

			sprintf(FOLDER_NAME,"%s%s%s%c%s%c%02d","Camera/","ID",deviceID, '_',time_struct.time_month,'.',atoi(time_struct.time_day));	 //month.day

			strcat(moDayDir,FOLDER_NAME);
			//printf("folder is %s\n",FOLDER_NAME);
			//printf("moDayDir is %s\n",moDayDir); 
			strcpy(sys_cmd,"mkdir -p ");
			strcat(sys_cmd,moDayDir);
			system(sys_cmd);

			//printf("folder is %s\n",FOLDER_NAME);
			//printf("moDayDir is %s\n",moDayDir); 

			//strcpy(moDayDir,'/'); //moDayDir: sdcard/month.day/

			// create a folder named as the current hour 
			sprintf(hourDir,"%s%c%d",moDayDir,'/', atoi(time_struct.time_hour));// moDayDir: sdard/month.day/; folderHead: hour
			//printf("hourDir is %s\n",hourDir);
			strcpy(cmd,"mkdir -p ");
			strcat(cmd,hourDir);
			system(cmd);

			//strcpy(hourDir,'/');//
		}

/*		//printf("store data to %s\n",PATH); //PATH="/sdcard/"
		strcpy(moDayDir,PATH);
		//printf("moDayDir is %s\n",moDayDir);

		sprintf(FOLDER_NAME,"%s%s%s%c%s%c%02d","Camera/","ID",deviceID, '_',time_struct.time_month,'.',atoi(time_struct.time_day));	 //month.day

		strcat(moDayDir,FOLDER_NAME);
		//printf("folder is %s\n",FOLDER_NAME);
		//printf("moDayDir is %s\n",moDayDir); 
		strcpy(sys_cmd,"mkdir -p ");
		strcat(sys_cmd,moDayDir);
		system(sys_cmd);

		sprintf(moDayDir,"%s%c",moDayDir,'/'); //moDayDir: sdcard/month.day/

		//get_time(&time_struct); //update time

		// create a folder named as the current hour 
		sprintf(hourDir,"%s%d%c",moDayDir,atoi(time_struct.time_hour),'/');// moDayDir: sdard/month.day/; folderHead: hour
		//printf("hourDir is %s\n",hourDir);
		strcpy(cmd,"mkdir -p ");
		strcat(cmd,hourDir);
		system(cmd);*/
		//******************************************************* end ********************************************************************//

		//get_time(&time_struct);
		sprintf(yrMonDayHrMinSec,"%s%s%02d%c%s%s%s%c%d",time_struct.time_year,time_struct.time_month,atoi(time_struct.time_day),'_',time_struct.time_hour,time_struct.time_min, time_struct.time_sec,'_',pictimecomp.indx);
		//strcat(yrMonDayHrMinSec,".dat");


		//sprintf(imFile,"/boot/uboot/Data/%d%s",k,".jpg");
		//printf("hourDir is %s\n",hourDir);
		sprintf(imFile,"%s%c%s%c%s%s",hourDir,'/',deviceID,'_',yrMonDayHrMinSec,".jpg");
		//printf("imFile is %s\n",imFile);

		//printf("read frame...\n");
		process_image(usr_buf[buf.index].start, usr_buf[buf.index].length, imFile);

	}

	if(-1 == ioctl(fd, VIDIOC_QBUF,&buf)){
		perror("Fail to ioctl 'VIDIOC_QBUF'");
		exit(EXIT_FAILURE);
	}
	return 1;
}

int mainloop(int fd)
{ 
	int count = 20;
	int i;
	char image_name[100];

	while(count-- > 0)
	{
		for(;;)
		{
			/*fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(fd,&fds);*/

			/*Timeout*/
			/*tv.tv_sec = 2;
			tv.tv_usec = 0;
			r = select(fd + 1,&fds,NULL,NULL,&tv);

			if(-1 == r)
			{
				if(EINTR == errno)
					continue;
				perror("Fail to select");
				exit(EXIT_FAILURE);
			}

			if(0 == r)
			{
				fprintf(stderr,"select Timeout\n");
				exit(-1);
			}*/

			for(i=0;i<1000;i++)
			{;}

			sprintf(image_name, JPG, 20-count);
			printf(image_name);
			printf("\n");
			if(read_frame(fd, image_name))
				break;
		}
	}
	return 0;
}

void stop_capture(int fd)
{
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(-1 == ioctl(fd,VIDIOC_STREAMOFF,&type))
	{
		perror("Fail to ioctl 'VIDIOC_STREAMOFF'");
		exit(EXIT_FAILURE);
	}
	return;
}

void close_camera_device(int fd)
{
	unsigned int i;
	for(i = 0;i < n_buffer; i++)
	{
		if(-1 == munmap(usr_buf[i].start,usr_buf[i].length)){
			exit(-1);
		}
	}
	free(usr_buf);

	if(-1 == close(fd))
	{
		perror("Fail to close fd");
		exit(EXIT_FAILURE);
	}
	return;
}


int audio_read(int audio_fd, unsigned char *outbuffer, int recording_len)
{

	int read_len;
	//FILE *fp;

	if ((read_len = read(audio_fd, outbuffer, recording_len)) == -1) 
	{ 
		perror("audio read"); exit(1); 
	} 

	//fflush(fp);

	//usleep(500);
	//fclose(fp);

	return read_len;
}

//void imuFunction(union sigval params)
void *imuFunction()
{
	int i;//
	int k;
	char file_flag=0;
	struct iio_context *ctx;
	struct iio_device *dev;
	struct iio_channel *chn;
	unsigned int  nb_devices;
	unsigned int nb_channels;
	double channel_value;

	static Current_tm time_struct={"0000","000","00","00","00","00"};
	int imu_samplingrate;
	char hourDir[MAXSTRING];
	char imuFile[MAXSTRING];
	FILE *fp;
	FILE *stream;
	char acc_z[10];
	short *imu_buffer;
	int read_len;

	int delay_cnt;

	//printf("Starting capture imu data!!!\n");

	MPU9250_sensor *pMPU9250_DATA;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	delay_cnt = 0;
	k=0;	
	flag_newdata = 0;
	while(1)
	{
		{
			//printf("%d\n",k);
			pMPU9250_DATA = MPU9250_DATA+LEN_SENSOR*flag_sensorbuffer+k;//cnt_sample_imu;//
		
			Motion_MPU9250_REGs_rd(pMPU9250_DATA);
				//printf("%f,%f,%f,%f,%f,%f,%f, %f,%f,%f\n\r",MPU9250_DATA.ACC_fl[0],MPU9250_DATA.ACC_fl[1],MPU9250_DATA.ACC_fl[2], MPU9250_DATA.TEMP_fl,MPU9250_DATA.GYRO_fl[0],MPU9250_DATA.GYRO_fl[1],MPU9250_DATA.GYRO_fl[2], MPU9250_DATA.Magnetometer[0],MPU9250_DATA.Magnetometer[1],MPU9250_DATA.Magnetometer[2]);

			get_time(&time_struct);
		
			pMPU9250_DATA->SensorHour = atoi(time_struct.time_hour);
			pMPU9250_DATA->SensorMin = atoi(time_struct.time_min);
			pMPU9250_DATA->SensorSec = atoi(time_struct.time_sec);
			
//			if(cnt_sample_imu==(LEN_SENSOR-1))
			if(k==(LEN_SENSOR-1))
			{
				k=0;//cnt_sample_imu=0;//

				if(flag_sensorbuffer==0)
					flag_sensorbuffer = 1;
				else if(flag_sensorbuffer==1)
					flag_sensorbuffer = 0;

				flag_newdata = 1;//new data is ready to be saved into file
			}
			else
				k++;//cnt_sample_imu++;//
		
			//slow down sampling rate	
			/*for(delay_cnt=0;delay_cnt<5000;delay_cnt++)
			{;}*/

			if((global_options.MCU_MODE==2)&(BT_connected==1)&(image_trans_ing==0)) 
			{
				sleep((numFrames/22)+10);///8);//sleep long time when BT transfer is used
				}
			else
				usleep(5000);//give 87sps averagely
				//usleep(10000);//give 45sps averagely
				//usleep(20000);//give 30sps averagely

			//printf("%02d%02d,%f,%f,%f,%f\n",pMPU9250_DATA->SensorMin, pMPU9250_DATA->SensorSec, pMPU9250_DATA->ACC_fl[0], pMPU9250_DATA->ACC_fl[1], pMPU9250_DATA->ACC_fl[2],  pMPU9250_DATA->TEMP_fl);
			//fprintf(fp,"%02d%02d,%f,%f,%f,%f,%f,%f,%f, %f,%f,%f\n",atoi(time_struct.time_min), atoi(time_struct.time_sec), MPU9250_DATA.ACC_fl[0],MPU9250_DATA.ACC_fl[1],MPU9250_DATA.ACC_fl[2], MPU9250_DATA.TEMP_fl,MPU9250_DATA.GYRO_fl[0],MPU9250_DATA.GYRO_fl[1],MPU9250_DATA.GYRO_fl[2], MPU9250_DATA.Magnetometer[0],MPU9250_DATA.Magnetometer[1],MPU9250_DATA.Magnetometer[2]);
	
		}
	}

//	ThreadParams_t *threadParams = (ThreadParams_t *) params.sival_ptr;
    	//printf("imuFunction, Timer %d expired.\n",*((int*) *(threadParams->timer)));
	//printf("imuFunction, Timer %d expired. Thread num which sent %d. Thread %ld. Pthread %ld.\n",
       //*((int*) *(threadParams->timer)), threadParams->threadNum,
       //syscall(SYS_gettid), pthread_self());
//    	wait[threadParams->threadNum] = 0;

	//pthread_exit(NULL);
}


void *imuDataSaveFunction()
{
	int k;
	
	int cnt_data;
	char tm[MAXSTRING];
	static Current_tm time_struct={"0000","000","00","00","00","00"};
	char sys_cmd[MAXSTRING];
	char cmd[MAXSTRING];
	char moDayDir[MAXSTRING];
	char hourDir[MAXSTRING];
	char FOLDER_NAME[MAXSTRING];
	char yrMonDayHrMinSec[MAXSTRING];
	char imuFile[MAXSTRING];
	FILE *fp;
	char file_format[5];
	unsigned char file_interval;
	short output_data[12];

	MPU9250_sensor *psaveMPU9250_DATA;
	
	char min_old;
	char imu_file_tail=0;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	min_old = 0;


	//if(global_options.textFile)
	//    strcpy(file_format,".txt");
	//else {
		strcpy(file_format,".hex");
		file_interval=2;//6;
	//}

	//cnt_data = 0;
	while(1)
	{

		if((global_options.MCU_MODE==2)&(BT_connected==1)&(image_trans_ing==0)) 
		{
			sleep((numFrames/22)+10);///8);//sleep long time when BT transfer is used
		}

		if(flag_newdata)//new data is availible
		{
			printf("Saving imu data...\n");

			if(!imu_file_open){
				psaveMPU9250_DATA = MPU9250_DATA+LEN_SENSOR*(1-flag_sensorbuffer);

				get_time(&time_struct);

				strcpy(moDayDir,PATH);
				sprintf(FOLDER_NAME,"%s%s%s%c%s%c%02d","IMU/","ID",deviceID, '_',time_struct.time_month,'.',atoi(time_struct.time_day));

				strcat(moDayDir,FOLDER_NAME);
				strcpy(sys_cmd,"mkdir -p ");
				strcat(sys_cmd,moDayDir);
				system(sys_cmd);

				//sprintf(moDayDir,"%s%c",moDayDir,'/'); //moDayDir: sdcard/month.day/

				//get_time(&time_struct); //update time

				// create a folder named as the current hour 
				sprintf(hourDir,"%s%c%d%c",moDayDir, '/', psaveMPU9250_DATA->SensorHour,'/');//
				//printf("hourDir is %s\n",hourDir);
				strcpy(cmd,"mkdir -p ");
				strcat(cmd,hourDir);
				system(cmd);


				imu_file_tail = (psaveMPU9250_DATA->SensorMin)/file_interval;//new file every 5 minutes
				sprintf(imuFile,"%s%s%02d%s",hourDir,"imu_",imu_file_tail,file_format);//".txt");
				//sprintf(imuFile,"%s%s%s%02d%s",hourDir,deviceID,"_imu_",imu_file_tail,file_format);//".txt");
				//sprintf(imuFile,"%s%s",moDayDir_sensor,"imu_sensordate.txt");

			
				if((fp = fopen(imuFile, "a")) == NULL){
						perror("Fail to open imu file");
						exit(EXIT_FAILURE);
				}
				imu_file_open = 1;
			}
				//printf("%f,%f,%f,%f,%f,%f,%f, %f,%f,%f\n\r",MPU9250_DATA.ACC_fl[0],MPU9250_DATA.ACC_fl[1],MPU9250_DATA.ACC_fl[2], MPU9250_DATA.TEMP_fl,MPU9250_DATA.GYRO_fl[0],MPU9250_DATA.GYRO_fl[1],MPU9250_DATA.GYRO_fl[2], MPU9250_DATA.Magnetometer[0],MPU9250_DATA.Magnetometer[1],MPU9250_DATA.Magnetometer[2]);
			
			if(imu_file_open)
			{
				for (k=0;k<LEN_SENSOR;k++)
				{
					psaveMPU9250_DATA = MPU9250_DATA+LEN_SENSOR*(1-flag_sensorbuffer)+k;


					/*fprintf(fp,"%02d%02d%02d,%.3f,%.3f,%.3f, %5.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",psaveMPU9250_DATA->SensorHour, psaveMPU9250_DATA->SensorMin, psaveMPU9250_DATA->SensorSec, psaveMPU9250_DATA->ACC_fl[0],psaveMPU9250_DATA->ACC_fl[1],psaveMPU9250_DATA->ACC_fl[2], psaveMPU9250_DATA->TEMP_fl,psaveMPU9250_DATA->GYRO_fl[0],psaveMPU9250_DATA->GYRO_fl[1],psaveMPU9250_DATA->GYRO_fl[2], psaveMPU9250_DATA->Magnetometer[0],psaveMPU9250_DATA->Magnetometer[1],psaveMPU9250_DATA->Magnetometer[2]);*/


					output_data[0] = (short)(psaveMPU9250_DATA->SensorHour);
					output_data[1] = ((short)(psaveMPU9250_DATA->SensorMin)<<8)+(short)(psaveMPU9250_DATA->SensorSec);
					output_data[2] = (short)(1000*psaveMPU9250_DATA->ACC_fl[0]);
					output_data[3] = (short)(1000*psaveMPU9250_DATA->ACC_fl[1]);
					output_data[4] = (short)(1000*psaveMPU9250_DATA->ACC_fl[2]);
					output_data[5] = (short)(10*psaveMPU9250_DATA->TEMP_fl);
					output_data[6] = (short)(10*psaveMPU9250_DATA->GYRO_fl[0]);
					output_data[7] = (short)(10*psaveMPU9250_DATA->GYRO_fl[1]);
					output_data[8] = (short)(10*psaveMPU9250_DATA->GYRO_fl[2]);
					output_data[9] = (short)(10*psaveMPU9250_DATA->Magnetometer[0]);
					output_data[9] = (short)(10*psaveMPU9250_DATA->Magnetometer[1]);
					output_data[11] = (short)(10*psaveMPU9250_DATA->Magnetometer[2]);  

					fwrite(output_data, 1, sizeof(output_data), fp);

					//fflush(fp);
				}
			
				if(min_old!=psaveMPU9250_DATA->SensorMin){ //close the file every minute
				//if(cnt_data>1000){
					fclose(fp);
					min_old = psaveMPU9250_DATA->SensorMin;//cnt_data = 0;//
					imu_file_open = 0;
				}
			}

			flag_newdata=0;

			//system("sync");
		}

	}

	//pthread_exit(NULL);
}


void *msp430_sensor_data_only()
{
	int i,k;
	char file_flag=0;
	struct iio_context *ctx;
	struct iio_device *dev;
	struct iio_channel *chn;
	unsigned int  nb_devices;
	unsigned int nb_channels;
	double channel_value;

	static Current_tm time_struct={"0000","000","00","00","00","00"};
	int imu_samplingrate;
	char hourDir[MAXSTRING];
	char imuFile[MAXSTRING];
	FILE *fp;
	FILE *stream;
	char acc_z[10];
	short *imu_buffer;
	int read_len;

	int delay_cnt;

	printf("Starting capture MSP430 data!!!\n");

	MSP430_DATA MSP430_SENSOR_DATA;
	MSP430_DATA *psaveMCU_DATA;

	unsigned char hist_val,hist_index;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);


	delay_cnt = 0;
	k=0;	
	flag_newdata_mcu = 0;

	hist_index = 0;
	hist_val = 0;
	
	while(1)
	{
		psaveMCU_DATA = MCU_SENSOR_DATA+LEN_SENSOR_MCU*flag_sensorbuffer_mcu+k;
			
		if(!MSP430_sensor_rd(psaveMCU_DATA))
		{
			//printf("%f,%f,%f,%f,%f,%f,%f, %f,%f,%f\n\r",MPU9250_DATA.ACC_fl[0],MPU9250_DATA.ACC_fl[1],MPU9250_DATA.ACC_fl[2], MPU9250_DATA.TEMP_fl,MPU9250_DATA.GYRO_fl[0],MPU9250_DATA.GYRO_fl[1],MPU9250_DATA.GYRO_fl[2], MPU9250_DATA.Magnetometer[0],MPU9250_DATA.Magnetometer[1],MPU9250_DATA.Magnetometer[2]);

			if(k==(LEN_SENSOR_MCU-1))
			{
				k=0;

				if(flag_sensorbuffer_mcu==0)
					flag_sensorbuffer_mcu = 1;
				else if(flag_sensorbuffer_mcu == 1)
					flag_sensorbuffer_mcu = 0;

				flag_newdata_mcu = 1;//new data is ready to be saved into file
			}
			else
				k++;

			
		}

/*
		if(!MSP430_sensor_rd(&MSP430_SENSOR_DATA))
		{
			if(hist_index!=MSP430_SENSOR_DATA.sec)
			{

				printf("sampling rate at %d sec: %d Hz\n", hist_index,hist_val);
				//printf("%d, %d, %d, %d\n\r",MSP430_SENSOR_DATA.sec, MSP430_SENSOR_DATA.MPU_ACC_mG[0],MSP430_SENSOR_DATA.MPU_ACC_mG[1],MSP430_SENSOR_DATA.MPU_ACC_mG[2]);

				hist_index=MSP430_SENSOR_DATA.sec;
				hist_val = 1;
			}
			else
				hist_val++;

			//printf("%d, %d, %d, %d\n\r",MSP430_SENSOR_DATA->sec, MSP430_SENSOR_DATA->MPU_ACC_mG[0],MSP430_SENSOR_DATA->MPU_ACC_mG[1],MSP430_SENSOR_DATA->MPU_ACC_mG[2]);

		}*/
	}

	//pthread_exit(NULL);
}

unsigned char gpiod14_read()
{

	FILE *gpioD14_fd;
	char gpioD14_value[1];
	unsigned char v=0;

	gpioD14_value[0] = 0;
	gpioD14_fd = fopen("/sys/class/gpio/pioD14/value", "rb");
	fread(gpioD14_value, sizeof(char), 1, gpioD14_fd);
	fclose(gpioD14_fd);

	v = atoi(gpioD14_value);
	//if(v==1) printf("%d,",v);
	
	return v;
}

void *msp430_image_trans_only()
{
	int i,k;

	unsigned char BT_send_ready=0;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);


	while(1)
	{
		//printf("BT status: %d...\n", BT_connected);
		if((BT_connected==1)&(image4bt_ready==1)){//connected

			for(i=0;i<100000;i++){;}//delay

			image4bt_ready = 0;
			image_trans_ing = 1;
			printf("BT image is ready!\n");

			printf("BT sending...!\n");

			printf("totally %d frames, numTails=%d\n",numFrames, numTails);

			//send valid frame number, not including this frame
			buffer_bt[0] = 0x3E;
			buffer_bt[1] = 0xFF;
			buffer_bt[2] = 0xFF;
			buffer_bt[3] = 0x75;
			buffer_bt[4] = 0x32;
			buffer_bt[5] = 0x21;
			buffer_bt[6] = numFrames%256;
			buffer_bt[7] = numFrames/256;

			do{
				BT_send_ready = gpiod14_read();
				//BT_connected = MSP430_read_bt_status();
				//if(BT_connected==0) break;
			}while(BT_send_ready==0);
			MSP430_write_bt(buffer_bt, Size_frame+numHeader);
			for(k=0;k<10000;k++)//delay, important
				{;}
			//printf("gpioD14: ");
			//do{BT_send_ready = gpiod14_read();}while(BT_send_ready==0);
			//printf("\n");
			////////////////////////////////////////////////////////////////////////////////
			////////JPEG data
			/*buffer_bt[0] = 0xFF;
			buffer_bt[1] = 0xFF;
			buffer_bt[2] = 0x32;
			buffer_bt[3] = 0x12;*/

			//for test
			/*for (i=0;i<numFrames-1;i++) {
				//generate test data
				buffer_bt[numHeader] = i%256;
				for(k=1;k<Size_frame;k++){buffer_bt[k+numHeader]=test_data+k;}

				MSP430_write_bt(buffer_bt, Size_frame+numHeader);//test data

				printf(".");
				if(i%100==0) printf(".. %d sent!\n", i);

				pImage += Size_frame;
				for(k=0;k<10000;k++)//delay, important
				{;}
				printf("gpioD14: ");
				do{BT_send_ready = gpiod14_read();}while(BT_send_ready==0);
				printf("\n");
			}*/


			printf("pImage[0][1]: 0x%x 0x%x\n", pImage[0],pImage[1]);
			for (i=0;i<numFrames-1;i++) {
				for(k=0;k<Size_frame;k++){buffer_bt[k+numHeader]=pImage[k];}
				
				do{
					BT_send_ready = gpiod14_read();
				}while(BT_send_ready==0);
				MSP430_write_bt(buffer_bt, Size_frame+numHeader);
				
				pImage += Size_frame;

				printf(".");
				if((i%100==0)&(i>0)) {
					//for(k=0;k<2000000;k++)//delay, important
					//{;}
					printf(".. %d sent!\n", i);
				}

				for(k=0;k<10000;k++)//delay, important
				{;}
				//printf("gpioD14: ");
				//do{BT_send_ready = gpiod14_read();}while(BT_send_ready==0);
				//printf("\n");
			}


			//sending tail
			printf("...sending tail...");
			buffer_bt[Size_frame+numHeader-2] = 0x37;
			buffer_bt[Size_frame+numHeader-1] = 0x73;
			for (i=0;i<numTails;i++) 
				buffer_bt[i+numHeader] = pImage[i];

			do{
				BT_send_ready = gpiod14_read();
				//BT_connected = MSP430_read_bt_status();
				//if(BT_connected==0) break;
			}while(BT_send_ready==0);

			MSP430_write_bt(buffer_bt, Size_frame+numHeader);
			/*for(k=0;k<100000;k++)//delay, important
				{;}*/
			//printf("gpioD14: ");
			//do{BT_send_ready = gpiod14_read();}while(BT_send_ready==0);
			//printf("\n");
			printf("totally %d frames were sent! numTails=%d\n",numFrames, numTails);
			/////////////////////////////////////////////////////////////////////////////////////

			image_trans_ing = 0;//transmission done!
			BT_connected = 0; //terminate bt every time after image is sent
			BT_IMG_EN = 0;

			printf("data sending finished!\n");
		}
	}

	//pthread_exit(NULL);
}



void *mcuDataSaveFunction()
{
	int k;

	int cnt_data;
	char tm[MAXSTRING];
	static Current_tm time_struct={"0000","000","00","00","00","00"};
	char sys_cmd[MAXSTRING];
	char cmd[MAXSTRING];
	char moDayDir[MAXSTRING];
	char hourDir[MAXSTRING];
	char FOLDER_NAME[MAXSTRING];
	char yrMonDayHrMinSec[MAXSTRING];
	char imuFile[MAXSTRING];
	char mcuFile[MAXSTRING];
	FILE *fp;
	char file_format[5];
	unsigned char file_interval=10;
	short output_data[21];

	MSP430_DATA *psaveMCU_DATA;
	
	char min_old;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	min_old = 0;
	char mcu_file_tail=0;

	//if(global_options.textFile)
	//    strcpy(file_format,".txt");
	//else {
		strcpy(file_format,".hex");
		file_interval=10;
	//}

	//cnt_data = 0;
	while(1)
	{
		if(flag_newdata_mcu)//new data is availible
		{
			psaveMCU_DATA = MCU_SENSOR_DATA+LEN_SENSOR_MCU*(1-flag_sensorbuffer_mcu);

			if(!mcu_file_open){
				//printf("opening mcuFile......, %d\n", moDayDir_t_flag);

				get_time(&time_struct);

				strcpy(moDayDir,PATH);
				sprintf(FOLDER_NAME,"%s%s%s%c%s%c%02d","MCU/","ID",deviceID, '_',time_struct.time_month,'.',atoi(time_struct.time_day));

				strcat(moDayDir,FOLDER_NAME);
				strcpy(sys_cmd,"mkdir -p ");
				strcat(sys_cmd,moDayDir);
				system(sys_cmd);

				//sprintf(moDayDir,"%s%c",moDayDir,'/'); //moDayDir: sdcard/month.day/

				//get_time(&time_struct); //update time
				
				sprintf(hourDir,"%s%c%d%c",moDayDir, '/', psaveMCU_DATA->hour,'/');//
				//printf("hourDir is %s\n",hourDir);
				strcpy(cmd,"mkdir -p ");
				strcat(cmd,hourDir);
				system(cmd);
	

				mcu_file_tail = (psaveMCU_DATA->min)/file_interval;//new file every 10 minutes
				sprintf(mcuFile,"%s%s%02d%s",hourDir,"mcu_",mcu_file_tail,file_format);//".txt");
				//sprintf(mcuFile,"%s%s",moDayDir_sensor,"imu_sensordate.txt");

				//printf("mcuFile path is: %s\n",mcuFile); 

				if((fp = fopen(mcuFile, "a")) == NULL){
						perror("Fail to open mcu file");
						exit(EXIT_FAILURE);
				}
				mcu_file_open = 1;
			}
				//printf("%f,%f,%f,%f,%f,%f,%f, %f,%f,%f\n\r",MPU9250_DATA.ACC_fl[0],MPU9250_DATA.ACC_fl[1],MPU9250_DATA.ACC_fl[2], MPU9250_DATA.TEMP_fl,MPU9250_DATA.GYRO_fl[0],MPU9250_DATA.GYRO_fl[1],MPU9250_DATA.GYRO_fl[2], MPU9250_DATA.Magnetometer[0],MPU9250_DATA.Magnetometer[1],MPU9250_DATA.Magnetometer[2]);
			
			if(mcu_file_open)
			{
				for (k=0;k<LEN_SENSOR_MCU;k++)
				{
					psaveMCU_DATA = MCU_SENSOR_DATA+LEN_SENSOR_MCU*(1-flag_sensorbuffer_mcu)+k;


					/*fprintf(fp,"%02d%02d%02d,%.3f,%.3f,%.3f, %5.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",psaveMPU9250_DATA->SensorHour, psaveMPU9250_DATA->SensorMin, psaveMPU9250_DATA->SensorSec, psaveMPU9250_DATA->ACC_fl[0],psaveMPU9250_DATA->ACC_fl[1],psaveMPU9250_DATA->ACC_fl[2], psaveMPU9250_DATA->TEMP_fl,psaveMPU9250_DATA->GYRO_fl[0],psaveMPU9250_DATA->GYRO_fl[1],psaveMPU9250_DATA->GYRO_fl[2], psaveMPU9250_DATA->Magnetometer[0],psaveMPU9250_DATA->Magnetometer[1],psaveMPU9250_DATA->Magnetometer[2]);*/


					output_data[0] = (short)(psaveMCU_DATA->hour);
					output_data[1] = ((short)(psaveMCU_DATA->min)<<8)+(short)(psaveMCU_DATA->sec);
					output_data[2] = (short)(psaveMCU_DATA->MPU_ACC_mG[0]);
					output_data[3] = (short)(psaveMCU_DATA->MPU_ACC_mG[1]);
					output_data[4] = (short)(psaveMCU_DATA->MPU_ACC_mG[2]);
					output_data[5] = (short)(psaveMCU_DATA->MPU_GYRO_10[0]);
					output_data[6] = (short)(psaveMCU_DATA->MPU_GYRO_10[1]);
					output_data[7] = (short)(psaveMCU_DATA->MPU_GYRO_10[2]);
					output_data[8] = (short)(psaveMCU_DATA->MPU_MAG_10[0]);
					output_data[9] = (short)(psaveMCU_DATA->MPU_MAG_10[1]);
					output_data[10] = (short)(psaveMCU_DATA->MPU_MAG_10[2]);  
					output_data[11] = (short)(psaveMCU_DATA->LIS_Mag[0]);
					output_data[12] = (short)(psaveMCU_DATA->LIS_Mag[1]);
					output_data[13] = (short)(psaveMCU_DATA->LIS_Mag[2]);
					output_data[14] = (short)(psaveMCU_DATA->BEM280_temp_Cdegree);
					output_data[15] = (short)(psaveMCU_DATA->BEM280_pressure_10Pa);
					output_data[16] = (short)(psaveMCU_DATA->BEM280_humidity_100percent);
					output_data[17] = (short)(psaveMCU_DATA->visiblelight);
					output_data[18] = (short)(psaveMCU_DATA->IRlight);
					output_data[19] = (short)(psaveMCU_DATA->proximity);  
					output_data[20] = (short)(psaveMCU_DATA->proximity);  


					fwrite(output_data, 1, sizeof(output_data), fp);
					cnt_data += LEN_SENSOR_MCU;
 
					//printf("k=%d, flag_newdata_mcu=%d\n", k, flag_newdata_mcu);

					//fflush(fp);
				}
			

				if(min_old!=psaveMCU_DATA->min){ //close the file every minute
				//if(cnt_data>5000){
					fclose(fp);
					min_old = psaveMCU_DATA->min;//cnt_data = 0;//
					mcu_file_open = 0;

					//system("sync");
				}
			}
			
			flag_newdata_mcu=0;

			
		}

	}

	//pthread_exit(NULL);
}

void *camFunction()
{
	int kc;
	
	int i,num_skip;

	//FILE *usb_connect_fd;
	//int usb_connected[1];

	/*char tm[MAXSTRING];
	static Current_tm time_struct={"0000","000","00","00","00","00"};
	char sys_cmd[MAXSTRING];
	char cmd[MAXSTRING];
	char moDayDir[MAXSTRING];
	char hourDir[MAXSTRING];
	char FOLDER_NAME[MAXSTRING];
	char yrMonDayHrMinSec[MAXSTRING];
	char imFile[MAXSTRING];

	get_time(&time_struct);
	sprintf(tm,"%s%c%s",time_struct.time_min,'.',time_struct.time_sec);
	strcpy(pictimecomp.pre_time, tm);*/
	
	//printf("camFunction...\n");	
	
	//open camera
	init_camera_device(camFd);
	start_capture(camFd);

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	kc=0;

	if(low_power_mode==0)
	{
		//set green led in heartbeat mode
		system("echo heartbeat > /sys/class/leds/led_green/trigger");
	}
	else
		system("echo none > /sys/class/leds/led_green/trigger");

	num_skip = 15;
	while (1)//!stop_flag)//
	{		

			if(low_power_mode==1)
			{
				//turn on green led
				//system("echo 1 > /sys/class/leds/led_green/brightness");

				//skip the first several frames to get stable image				

				for(i=0;i<num_skip;i++)
				{
					read_frame(camFd, (i==(num_skip-1)));
				}
	
				stop_capture(camFd);//stop camera

				if((global_options.MCU_MODE==2)&(BT_connected==1)&(image_trans_ing==0)) 
				{
					image4bt_ready = 1;
					sleep((numFrames/22)+10);///8);//sleep long time when BT transfer is used

					BT_connected=0;
					image4bt_ready = 0;
				}
				else
					sleep(2+(6*(global_options.DMIC_ON)));//slow down image frame rate if audio is on

				//sleep(2+((numFrames/20)*(global_options.MCU_MODE)*BT_connected));//

				//sleep long time when BT transfer is used
				/*if((global_options.MCU_MODE==2)&BT_connected)
					while(image_trans_ing)
						sleep(5);*/

				//image4bt_ready = 0;


				//turn off green led
				//system("echo 0 > /sys/class/leds/led_green/brightness");

				init_camera_device(camFd);//reopen camera
				start_capture(camFd);
			}
			else if(low_power_mode==2)//lowest power consumption
			{
				//turn on green led
				//system("echo 1 > /sys/class/leds/led_green/brightness");

				//no skip, image will have color degradation
				read_frame(camFd, 1);
	
				stop_capture(camFd);//stop camera

				init_camera_device(camFd);//reopen camera
				start_capture(camFd);

				//turn off green led
				//system("echo 0 > /sys/class/leds/led_green/brightness");

			}		
			else if(low_power_mode==0)
			{
				//set green led in heartbeat mode
				system("echo heartbeat > /sys/class/leds/led_green/trigger");

				read_frame(camFd, 1);
				usleep(150000);//
			}

			kc++;

			//if(kc==10)
			//{
			//	system("sync");
			//	kc=0;
			//}

			/*if(kc%3==0)
			{
				usb_connected[0] = 0;
				usb_connect_fd = fopen("/sys/class/gpio/pioE9/value", O_RDONLY);
				fread(usb_connect_fd, 1,1, usb_connected);
				printf("usb_connected: %d\n",usb_connected);
				fclose(usb_connect_fd);
				while(usb_connected[0]);//halt camera program
			}*/
	}

	//pthread_exit(NULL);
}
/*	
void camFunction_timer(void)
//void camFunction_timer(union sigval params)
{
	int kc;
	
	int i,num_skip;

	//FILE *usb_connect_fd;
	//int usb_connected[1];
	

	init_camera_device(camFd);
	start_capture(camFd);

	//enable cancellation
//	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	kc=0;

	if(low_power_mode==0)
	{
		//set green led in heartbeat mode
		system("echo heartbeat > /sys/class/leds/led_green/trigger");
	}
	else
		system("echo none > /sys/class/leds/led_green/trigger");

	num_skip = 15;
	//while (1)//!stop_flag)//
	{		

			//if(low_power_mode==1)
			{
				//turn on green led
				//system("echo 1 > /sys/class/leds/led_green/brightness");

				//skip the first several frames to get stable image				

				for(i=0;i<num_skip;i++)
				{
					read_frame(camFd, (i==(num_skip-1)));
				}
	
				stop_capture(camFd);//stop camera

				//close_camera_device(camFd);
			}

	}

//	ThreadParams_t *threadParams = (ThreadParams_t *) params.sival_ptr;
//    	wait[threadParams->threadNum] = 0;

	//pthread_exit(NULL);
}
*/
	
void *dmicFunction()
{	

	unsigned char *audio_buffer_write;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	while (1)
	{

		//buffer switch
		audio_buffer_write = audio_buffer+(new_audio_ppflag)*BUF_SIZE;
		
		//read audio data
		audio_read(audioFd, audio_buffer_write, BUF_SIZE);

		if(new_audio_ppflag==0)
			new_audio_ppflag = 1;
		else
			new_audio_ppflag = 0;

		new_audio_data = 1;
		
	}

}


void *dmicFunction_save()
{

	char tm[MAXSTRING];
	static Current_tm time_struct={"0000","000","00","00","00","00"};
	char sys_cmd[MAXSTRING];
	char cmd[MAXSTRING];
	char moDayDir[MAXSTRING];
	char hourDir[MAXSTRING];
	char FOLDER_NAME[MAXSTRING];
	char yrMonDayHrMinSec[MAXSTRING];
	char auFile[MAXSTRING];

	FILE *fp;

	int i,k;
	unsigned char *audio_buffer_read;
	unsigned char *pcm_bytes;
	int frame_size;
	opus_int16 in[FRAME_SIZE*CHANNELS];
	unsigned char cbits[MAX_PACKET_SIZE];
	int nbBytes;
	int err;

	unsigned char min_old_au,audio_file_tail;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	min_old_au = 0;
	audio_file_tail = 0;
	while (1)
	{
		if(new_audio_data)
		{
			new_audio_data = 0;

			//buffer switch
			audio_buffer_read = audio_buffer+(1-new_audio_ppflag)*BUF_SIZE;

			get_time(&time_struct);

			if(!audio_file_open)
			{
				//************************************************* Path generation **************************************************************************//
				//printf("store data to %s\n",PATH); //PATH="/sdcard/"
				strcpy(moDayDir,PATH);
				//printf("moDayDir is %s\n",moDayDir);

				sprintf(FOLDER_NAME,"%s%s%s%c%s%c%02d","DMIC/", "ID",deviceID, '_',time_struct.time_month,'.',atoi(time_struct.time_day));	 //month.day

				strcat(moDayDir,FOLDER_NAME);
				//printf("folder is %s\n",FOLDER_NAME);
				//printf("moDayDir is %s\n",moDayDir); 
				strcpy(sys_cmd,"mkdir -p ");
				strcat(sys_cmd,moDayDir);
				system(sys_cmd);

				//sprintf(moDayDir,"%s%c",moDayDir,'/'); //moDayDir: sdcard/month.day/

				//get_time(&time_struct); //update time

				// create a folder named as the current hour 
				sprintf(hourDir,"%s%c%d%c",moDayDir, '/', atoi(time_struct.time_hour),'/');// moDayDir: sdard/month.day/; folderHead: hour
				//printf("hourDir is %s\n",hourDir);
				strcpy(cmd,"mkdir -p ");
				strcat(cmd,hourDir);
				system(cmd);
				//******************************************************* end ********************************************************************//

				//get_time(&time_struct);
				//sprintf(yrMonDayHrMinSec,"%s%s%02d%c%s%s%s",time_struct.time_year,time_struct.time_month,atoi(time_struct.time_day),'_',time_struct.time_hour,time_struct.time_min, time_struct.time_sec);
				//strcat(yrMonDayHrMinSec,".dat");

			
				audio_file_tail = (atoi(time_struct.time_min))/1;//new file every 1 minutes
			
				//sprintf(auFile,"/boot/uboot/Data/%d%s",k,".hex");
				//sprintf(auFile,"%s%s%c%s%s",hourDir,deviceID,'_',yrMonDayHrMinSec,".hex");
				sprintf(auFile,"%s%s%s%02d%s",hourDir,deviceID,"_record_",audio_file_tail,".opu");
				printf(auFile);
				printf("\n");

				//open file to write
				if((fp = fopen(auFile, "a")) == NULL){
						perror("Fail to fopen");
						exit(EXIT_FAILURE);
				}

				audio_file_open = 1;
			}

			if(audio_file_open)
			{
				//opus encoding
				for (k=0;k<NUM_FRAMES_PER_SEC*RECORDING_SEC;k++)
				{

					/* Read a 16 bits/sample audio frame. */
					/* Convert from little-endian ordering. */
					pcm_bytes = audio_buffer_read+2*k*CHANNELS*FRAME_SIZE;
					for (i=0;i<CHANNELS*FRAME_SIZE;i++)
						in[i]=pcm_bytes[2*i+1]<<8|pcm_bytes[2*i];

					/* Encode the frame. */
					nbBytes = opus_encode(encoder, in, FRAME_SIZE, cbits, MAX_PACKET_SIZE);
					if (nbBytes<0)
					{
						fprintf(stderr, "encode failed: %s\n", opus_strerror(nbBytes));
						return EXIT_FAILURE;
					}

					//write date to file
					fwrite(&nbBytes, sizeof(int), 1, fp);
					fwrite(cbits, 1, nbBytes, fp);

				}

				if(min_old_au!=(atoi(time_struct.time_min))){ //close the file every minute
					fclose(fp);
					min_old_au = (atoi(time_struct.time_min));//cnt_data = 0;//
					audio_file_open = 0;
				}

			}
			//fflush(fp);

			//usleep(500);
			//fclose(fp);

		}
	}

}

void power_off()
{
	int i;

	time_t time_shutdown;
	FILE *fp_time;

	//blue led
	system("echo none > /sys/class/leds/led_red/trigger");
	system("echo 0 > /sys/class/leds/led_red/brightness");
	system("echo none > /sys/class/leds/led_green/trigger");
	system("echo 0 > /sys/class/leds/led_green/brightness");
	system("echo heartbeat > /sys/class/leds/led_blue/trigger");

	if (global_options.CAMERA_ON)  pthread_cancel(camThread);
	if (global_options.DMIC_ON)	{
		free(audio_buffer);
		/*Destroy the encoder state*/
	   	opus_encoder_destroy(encoder);

		pthread_cancel(dmicThread);
		pthread_cancel(dmicThread_save);

	}
	if (global_options.IMU_ON)	{
		pthread_cancel( imuThread);
		pthread_cancel( imusaveThread);
	}
	if (global_options.MCU_MODE==1)	{
		pthread_cancel( msp430Thread);
		pthread_cancel( msp430saveThread);
	}
	else if(global_options.MCU_MODE==2)	
		pthread_cancel(msp430imageThread);

	system("sync");
	for (i=0;i<1000000;i++)
	{;}

	//mark shutdown time
	if((fp_time = fopen("/boot/uboot/system/time_mark", "w+")) == NULL){
		perror("Fail to open time_mark file");
	}
	else{
		time(&time_shutdown);
		fwrite(&time_shutdown, sizeof(long),1, fp_time);
		//printf ("fwrite done\n");
		fclose(fp_time);
	}

	system("umount /media/lpebutton");
	
	system("shutdown -h now");//system("poweroff");//


}


void *LowBattery_detection()
{

	FILE *nlbo_fd;
	//FILE *user_fd;
	char nlbo[1];
	//char user[1];

  	//interval = 0;


	while (1)
	{

		//low battery
		nlbo[0] = 0;
		nlbo_fd = fopen("/sys/class/gpio/pioE24/value", "rb");
		fread(nlbo, sizeof(char), 1, nlbo_fd);
		//printf("unlbo gpio: %d\n",atoi(nlbo));
		fclose(nlbo_fd);

		if(atoi(nlbo)==0){//check battery, if low, shutdown
			
			power_off();
			
			//shutdown system
			//////system("echo s > /proc/sysrq-trigger");//sync
			//system("echo u > /proc/sysrq-trigger");//umount
			//system("echo o > /proc/sysrq-trigger");//poweroff
		}
		
	}


	return 0;

}

/*
void *soft_poweronoff()
{	

	int keys_fd;
	char ret[2];
	struct input_event t;
	struct timeval start, end;
	int trig;
	int interval;
	
	char *system_poweroff="poweroff";

	keys_fd = open ("/dev/input/event0", O_RDONLY);
	if (keys_fd <= 0)
	{
		printf ("open /dev/input/event0 device error!\n");
		return 0;
	}
	 
	trig=0;
	interval = 0;
	while (1)
	{
	      if (read (keys_fd, &t, sizeof (t)) == sizeof (t))
		{
		  if (t.type == EV_KEY)
			if (t.value == 1)
			{
		        	gettimeofday(&start, NULL);
				printf ("key pressed, time %ld\n", start.tv_sec);			
			}
			else if (t.value == 0)
			{
				gettimeofday(&end, NULL);
				printf ("key released, time %ld\n", end.tv_sec);

				interval = (int)(end.tv_sec - start.tv_sec);
				printf ("interval time %d\n", interval);
				if(interval>=2)
				{
					printf ("pressed time is enough! system will poweroff\n");

					power_off();
				}
				interval = 0;
			}

		}
	}
	close (keys_fd);

}


void *check_battery_vsys(void)
{
	int fd;
	char buf[2];
	char val;

        fd = open( "/dev/i2c-1", O_RDWR );

	if( ioctl( fd, I2C_SLAVE, PMIC_I2C_ADDRESS ) < 0 )
        {
                fprintf( stderr, "Failed to set PMIC slave address: %m\n" );
                return;
        }

	//set syslev = 3.3V
	buf[0] = 0;
	buf[1] = 0xca;
	if (write(fd, buf, 2) != 2)
	{
		printf("PMIC configure error\n");

		return;
	}

	while(1)
	{
		buf[0] = 0;
		write(fd, buf, 1);
		read(fd, &val, 1);
		printf("PMIC Register 0: 0x%02x\n", val);
	}
}
*/


/////timer test
void timer_handler_imu(void)
{
  printf("timer: var_imu is %i\n", var_imu++);
}


//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{

	time_t time_shutdown,time_poweron;
	FILE *fp_time;
	int interval;
	int fd_time;
	//struct rtc_time rt;
	struct tm *current_local_time;
	char buffer[20];

	//timer
/*        timer_t timerid;
        struct sigevent sev;
        struct itimerspec its;
        long long freq_nanosecs;
        sigset_t mask;
        struct sigaction sa;
*/

	//int audio_len;
	//int audio_count;
	int audio_format; 
	int audio_speed;
	
	pthread_attr_t thread_attr_cam, thread_attr_imu, thread_attr_imusave, thread_attr_mcu, thread_attr_mcusave,thread_attr_dmic,thread_attr_dmic_save;
	struct sched_param schedule_param_cam, schedule_param_imu, schedule_param_imusave, schedule_param_mcu, schedule_param_mcusave,schedule_param_dmic;
	
	MSP430_CFG msp430_cfg_data;


	//int ret;
	//int start;
	//int k,i;
	char *config_file="/media/lpebutton/config";//"/boot/uboot/system/config";
	
	//struct v4l2_format codec_fmt;
	struct v4l2_requestbuffers bufrequest;
	//struct v4l2_buffer bufferinfo;
	//struct v4l2_capability codec_cap;

	char sys_cmd[MAXSTRING];
	char rtc_cmd[MAXSTRING];
	//char *deviceIDfile="/boot/uboot/system/DeviceID";
	//FILE *idFid;

	time_t timer;
	static Current_tm time_struct={"0000","000","00","00","00","00"};
	
	char *hwclock_rtc="hwclock -w";

	
	//char sys_cmd[MAXSTRING];
	//char cmd[MAXSTRING];
	//char moDayDir[MAXSTRING];
	//char hourDir[MAXSTRING];
	//char FOLDER_NAME[MAXSTRING];
	//char yrMonDayHrMinSec[MAXSTRING];
	//char imFile[MAXSTRING];
	//char auFile[MAXSTRING];
	
	

	//extern timecomp;
	//timecomp pictimecomp={"000000000",1};
	/*if (( global_options = (option_struct *) malloc( sizeof(option_struct) ) ) == NULL) {
		printf("ERROR ALLOCATING global_options\n");
		exit(1);
	}*/


	//initial leds
	system("echo 1 > /sys/class/leds/led_red/brightness");
	system("echo 1 > /sys/class/leds/led_blue/brightness");
	system("echo 1 > /sys/class/leds/led_green/brightness");
	


	/****Read ID*****/
	/*idFid=fopen(deviceIDfile,"r");
	if (idFid!=NULL)
	{
		fgets(deviceID, 4, idFid);
		fclose(idFid);
	}*/
	//printf("/**************************************************************************/\n");
	//printf("DeviceID = %s\n\n", deviceID);

	/* Initialize global options */
	//initialize_global(&global_options);

	
	/* Parse the command options */
	//cmd_proc(config_file,argc,argv);
	//printf("config_file: %s\n\n", config_file);

	/* Read the configuration file to set the global options */
	get_param(&global_options, config_file);

	sprintf(deviceID, "%04d", global_options.ID);

	low_power_mode = global_options.LOW_POWER_MODE;
	image_encryption_en = global_options.IMAGE_ENCRYPTION;

        ////
	system("date");
	//system("date 101714092012");
	//system("hwclock -w");


	//initial leds
	system("echo 0 > /sys/class/leds/led_red/brightness");
	system("echo 0 > /sys/class/leds/led_blue/brightness");
	system("echo 0 > /sys/class/leds/led_green/brightness");

	//create data folder
	strcpy(sys_cmd,"mkdir -p ");
	strcat(sys_cmd,PATH);
	system(sys_cmd);

	if (global_options.RTC_UPDATE)
	{

		sprintf(rtc_cmd,"%s%c%s","date",' ', global_options.RTC_TIME);
		system(rtc_cmd);
		system(hwclock_rtc);
		system("rm /boot/uboot/system/time_mark");
	}
	else if(global_options.RTC_AUTO_CR)//correct system time
	{
		if((fp_time = fopen("/boot/uboot/system/time_mark", "r")) == NULL){
			perror("Fail to open time_mark file");
		}
		else{
			time(&time_poweron);
			//current_local_time = localtime(&time_poweron);

			printf ("time_poweron: %ld\n", time_poweron);
			fread(&time_shutdown,sizeof(long),1,fp_time); //add the time consumed by shutdown	
			printf ("time_shutdown: %ld\n", time_shutdown);		
			fclose(fp_time);

			time_shutdown += 20; //add time consumed by previous shutdown
			interval = (int) difftime(time_poweron,time_shutdown);
			printf ("interval: %ld\n", interval);

			interval = (int) (((float)interval/32.0)*32.768);
			printf ("corrected interval: %ld\n", interval);
			time_poweron = (time_shutdown+interval);
			printf ("corrected time: %ld\n", time_poweron);
			
			current_local_time = localtime(&time_poweron);

			//printf("local time：%s", asctime(current_local_time));
			strftime(buffer,20,"%m%d%H%M%Y.%S", current_local_time);
			//printf("Formatted date & time : |%s|\n", buffer );

			sprintf(rtc_cmd,"%s%c%s","date",' ', buffer);//sprintf(rtc_cmd,"%s%c%s%c%02d","date",' ', buffer, '.',current_local_time->tm_sec);
			
			//printf("%s\n",rtc_cmd);	
			system(rtc_cmd);
			system(hwclock_rtc);
			printf("time correction done!\n");
		}	
	}

/*	
	//timer test
	if(start_timer_imu(100, &timer_handler_imu)){
	    
		printf("\n imu timer error\n");
	    	return(1);
	}

	while(1)
	{
    		if(var_imu > 100)
    		{
      			break;
    		}
  	}


	//stop_timer_cam();
	stop_timer_imu();

        exit(EXIT_SUCCESS);
*/

	/*if (pthread_mutex_init(&lock, NULL) != 0)
	    {
		printf("\n mutex init failed\n");
		return 1;
	    }*/


	system("echo heartbeat > /sys/class/leds/led_green/trigger");
	if (global_options.CAMERA_ON)
	{

		printf("Open camera...\n");

		camFd = open_camera_device();
		
		//initial video capture way(mmap)
		init_mmap(camFd);
		
		//mainloop(camFd);
		//stop_capture(camFd);
		//close_camera_device(camFd);
		//printf("camera test done!\n");

		if(global_options.DMIC_ON)
			pthread_create( &camThread, NULL, camFunction, NULL);
		else{
			pthread_attr_init(&thread_attr_cam);
			pthread_attr_setinheritsched(&thread_attr_cam, PTHREAD_EXPLICIT_SCHED); 
			pthread_attr_setschedpolicy(&thread_attr_cam,SCHED_RR);
			schedule_param_cam.sched_priority = 1;
			pthread_attr_setschedparam(&thread_attr_cam, &schedule_param_cam); 

			pthread_create( &camThread, &thread_attr_cam, camFunction, NULL);
			//pthread_create( &camThread, NULL, camFunction, NULL);
		}


/*		    	timeToWait[0].it_value.tv_sec = 3;
		    	timeToWait[0].it_value.tv_nsec = 0;
		    	timeToWait[0].it_interval.tv_sec = timeToWait[0].it_value.tv_sec;
		    	timeToWait[0].it_interval.tv_nsec = timeToWait[0].it_value.tv_nsec;

		    	events[0].sigev_notify = SIGEV_THREAD;
		    	events[0].sigev_notify_function = camFunction_timer;
		    	events[0].sigev_value.sival_ptr = &thParams[0];
			if (0 != timer_create(CLOCK_MONOTONIC, &events[0], &timers[0]))
			{
				printf("Failed to create timers. Error %d.\n", errno);
				return 1;
			}

	    		wait[0] = 1;

	    		thParams[0].threadNum = 0;
	    		thParams[0].event = &events[0];
	    		thParams[0].timer = &timers[0];

	    		if (0 != pthread_create(&thParams[0].tid, NULL, threadTask, (void *) &thParams[0]))
	    		{
				printf("Failed to create thread. Error %d.\n", errno);

			    	timer_delete(timers[0]);

				return 1;
	    		}	
*/
/*		if(start_timer(3000, &camFunction_timer)){
	    
			printf("\n imu timer error\n");
	    		return(1);
		}*/
	
	}
	//else
	//	system("echo heartbeat > /sys/class/leds/led_green/trigger");//blink green led for other sensor


	if (global_options.IMU_ON)
	{

		printf("Open imu...\n");

		//(spi_speed_set， gyro_fs, acc_fs)
		//if(global_options.IMU_SPEED>5000000)
		//	global_options.IMU_SPEED = 5000000;
		//if(global_options.IMU_SPEED<600000)
		//	global_options.IMU_SPEED = 600000;
			
		inv_mpu9250_init_config(1000000, global_options.GYRO_FS , global_options.ACC_FS );

		/*pthread_attr_init(&thread_attr_imu);
		pthread_attr_setinheritsched(&thread_attr_imu, PTHREAD_EXPLICIT_SCHED); 
		pthread_attr_setschedpolicy(&thread_attr_imu,SCHED_RR);
		schedule_param_imu.sched_priority = 5;
		pthread_attr_setschedparam(&thread_attr_imu, &schedule_param_imu); 
		pthread_attr_init(&thread_attr_imusave);
		pthread_attr_setinheritsched(&thread_attr_imusave, PTHREAD_EXPLICIT_SCHED); 
		pthread_attr_setschedpolicy(&thread_attr_imusave,SCHED_RR);
		schedule_param_imusave.sched_priority = 5;
		pthread_attr_setschedparam(&thread_attr_imusave, &schedule_param_imusave); 

		pthread_create( &imuThread, &thread_attr_imu, imuFunction, NULL);
		pthread_create( &imusaveThread, &thread_attr_imusave, imuDataSaveFunction, NULL);*/
		

		pthread_create( &imuThread, NULL, imuFunction, NULL);
		pthread_create( &imusaveThread, NULL, imuDataSaveFunction, NULL);

	}

	//////////pthread_create( &poweronoffThread, NULL, soft_poweronoff, NULL);
	pthread_create( &lowbatteryThread, NULL, LowBattery_detection, NULL);//check_battery_vsys, NULL);//


	printf("DMIC_ON = %d\n", global_options.DMIC_ON);
	if (global_options.DMIC_ON)
	{
		printf("opening DSP...\n");
		//open sound device
		if ((audioFd = open(dmic_DEV, O_RDONLY, 0)) < 0) 
		{ 
			// Open of device failed 
			perror(dmic_DEV); 
			exit(1); 
		} 
		printf("DSP opened!\n");

		//format setting
		audio_format = AFMT_S16_LE; 
		printf("format set: %d\n", audio_format);
		if (ioctl(audioFd, SNDCTL_DSP_SETFMT, &audio_format) == -1) 
		{ 
			// fatal error 
			perror("SNDCTL_DSP_SETFMT"); 
			exit(1); 
		} 
		if (audio_format != AFMT_S16_LE) //16bits, little endian
		{ 
			printf("Not supported!\n");
		} 
		 printf("format set done: %d\n", audio_format);

		audio_speed = SAMPLE_RATE;
		printf("Sampling rate initial: %dHz\n", audio_speed);
		if (ioctl(audioFd, SNDCTL_DSP_SPEED, &audio_speed)==-1) 
		{ 
			perror("SNDCTL_DSP_SPEED"); 
			exit(1); 
		} 
		printf("Sampling rate is set: %dHz\n", audio_speed);
		

	   	/************************Create a new encoder state *********************************/
	   	encoder = opus_encoder_create(SAMPLE_RATE, CHANNELS, APPLICATION, &err);
	   	if (err<0)
	   	{
	      		fprintf(stderr, "failed to create an encoder: %s\n", opus_strerror(err));
	      		return EXIT_FAILURE;
	   	}
	   	/* Set the desired bit-rate. You can also set other parameters if needed.
	      	The Opus library is designed to have good defaults, so only set
	      	parameters you know you need. Doing otherwise is likely to result
	      	in worse quality, but better. */
	   	err = opus_encoder_ctl(encoder, OPUS_SET_BITRATE(BITRATE));
	   	if (err<0)
	   	{
	      		fprintf(stderr, "failed to set bitrate: %s\n", opus_strerror(err));
	      		return EXIT_FAILURE;
	   	}

		//pingpong buffer
		audio_buffer = (unsigned char *)malloc( 2*BUF_SIZE * sizeof(unsigned char)); 

		//setup thread
		/*pthread_attr_init(&thread_attr_dmic);
		pthread_attr_setinheritsched(&thread_attr_dmic, PTHREAD_EXPLICIT_SCHED); 
		pthread_attr_setschedpolicy(&thread_attr_dmic,SCHED_RR);
		schedule_param_dmic.sched_priority = 2;
		pthread_attr_setschedparam(&thread_attr_dmic, &schedule_param_dmic); 
		pthread_create( &dmicThread, &thread_attr_dmic, dmicFunction, NULL);*/
		pthread_create( &dmicThread, NULL, dmicFunction, NULL);

		/*pthread_attr_init(&thread_attr_dmic_save);
		pthread_attr_setinheritsched(&thread_attr_dmic_save, PTHREAD_EXPLICIT_SCHED); 
		pthread_attr_setschedpolicy(&thread_attr_dmic_save,SCHED_RR);
		schedule_param_dmic.sched_priority = 2;
		pthread_attr_setschedparam(&thread_attr_dmic_save, &schedule_param_dmic); 
		pthread_create( &dmicThread_save, &thread_attr_dmic_save, dmicFunction_save, NULL);*/
		pthread_create( &dmicThread_save, NULL, dmicFunction_save, NULL);


	}


	//if (global_options.MCU_BARO_ON|global_options.MCU_MOTION_ON|global_options.MCU_LIS_ON|global_options.MCU_OPTIC_ON)
	{

		printf("Open MSP430...\n");

		//MCU_SENSOR_ON = 0;
		//MCU_BT_IMAGE_TRANS_ON=0;

		//get_time(&time_struct);
		timer=time(NULL);
		struct tm time = *localtime(&timer);
		printf("set msp430 time: %04d-%02d-%02d-%02d-%02d-%02d\n",time.tm_year+1900,time.tm_mon+1,time.tm_mday,time.tm_hour,time.tm_min, time.tm_sec);
		
		msp430_cfg_data.rtc_sec = time.tm_sec;
		msp430_cfg_data.rtc_min = time.tm_min;
		msp430_cfg_data.rtc_hr = time.tm_hour;
		msp430_cfg_data.rtc_day = time.tm_mday;
		msp430_cfg_data.rtc_mon = time.tm_mon+1;
		msp430_cfg_data.rtc_yr = time.tm_year+1900-2000;
		msp430_cfg_data.msp430_mode = global_options.MCU_MODE;
		msp430_cfg_data.msp430_BARO_on = global_options.MCU_BARO_ON;
		msp430_cfg_data.msp430_MOTION_on = global_options.MCU_MOTION_ON;
		msp430_cfg_data.msp430_LIS_MAG_on = global_options.MCU_LIS_ON;
		msp430_cfg_data.msp430_OPTIC_on = global_options.MCU_OPTIC_ON;
		msp430_cfg_data.msp430_SAMPLING_RATE = ((global_options.MCU_SAMPLING_RATE)/10)&0x0f;

		MSP430_init_config((1000000+1000000*(2-global_options.MCU_MODE)), msp430_cfg_data );
		//MSP430_init_config((2000000+1000000*(2-global_options.MCU_MODE)), msp430_cfg_data );
	
		//no operation for mode 0, mcu sleep
		/*if(global_options.MCU_MODE==1)
			MCU_SENSOR_ON = 1;
		else if(global_options.MCU_MODE==2)
			MCU_BT_IMAGE_TRANS_ON=1;*/
	}

	if(global_options.MCU_MODE==1)
	{
		/*pthread_attr_init(&thread_attr_mcu);
		pthread_attr_setinheritsched(&thread_attr_mcu, PTHREAD_EXPLICIT_SCHED); 
		pthread_attr_setschedpolicy(&thread_attr_mcu,SCHED_RR);
		schedule_param_mcu.sched_priority = 5;
		pthread_attr_setschedparam(&thread_attr_mcu, &schedule_param_mcu); 
		pthread_attr_init(&thread_attr_mcusave);
		pthread_attr_setinheritsched(&thread_attr_mcusave, PTHREAD_EXPLICIT_SCHED); 
		pthread_attr_setschedpolicy(&thread_attr_mcusave,SCHED_RR);
		schedule_param_mcusave.sched_priority = 5;
		pthread_attr_setschedparam(&thread_attr_mcusave, &schedule_param_mcusave); 

		pthread_create( &msp430Thread, &thread_attr_mcu, msp430_sensor_data_only, NULL);
		pthread_create( &msp430saveThread, &thread_attr_mcusave, mcuDataSaveFunction, NULL);*/
		pthread_create( &msp430Thread, NULL, msp430_sensor_data_only, NULL);
		pthread_create( &msp430saveThread, NULL, mcuDataSaveFunction, NULL);
	}
	else if(global_options.MCU_MODE==2)
	{
		pImage = (unsigned char *)malloc(200*1024);//200KB buffer for image

		pthread_create( &msp430imageThread, NULL, msp430_image_trans_only, NULL);
	}


	//turn off leds
	system("echo 0 > /sys/class/leds/led_red/brightness");
	system("echo 0 > /sys/class/leds/led_blue/brightness");
	system("echo 0 > /sys/class/leds/led_green/brightness");

	/////////////////////////////////////////////////////////////////////////
	//start threads
	///////pthread_join( poweronoffThread, NULL);
	pthread_join( lowbatteryThread, NULL);

	if (global_options.CAMERA_ON)  pthread_join( camThread, NULL);//pthread_join(thParams[0].tid, NULL);//
	if (global_options.DMIC_ON){
		pthread_join( dmicThread, NULL);
		pthread_join( dmicThread_save, NULL);
	}
	if (global_options.IMU_ON)	{
		pthread_join( imuThread, NULL);
		pthread_join( imusaveThread, NULL);//pthread_join(thParams[2].tid, NULL);//
	}

	if(global_options.MCU_MODE==1)
	{
		pthread_join( msp430Thread, NULL); 
		pthread_join( msp430saveThread, NULL); 
	}
	else if(global_options.MCU_MODE==2)
	{
		pthread_join( msp430imageThread, NULL); 
	}

	//pthread_mutex_destroy(&lock);

	pthread_exit(NULL);

	while(1)
	{;}

	return 0;

}

