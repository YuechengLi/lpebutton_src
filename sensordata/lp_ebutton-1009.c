/*
 * lpebutton_cam_App.c
 *
 *  Created on: Jun 25, 2016
 *      Author: root
 */

#include <stdio.h>
#include <fcntl.h>
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

#include "timer.h"

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

char moDayDir_t[MAXSTRING];
char moDayDir_sensor[MAXSTRING];

#define JPG	"image%d.jpg"
int IM_WIDTH=1280;
int IM_HEIGHT=720;

char deviceID[5]={"0"};
int camFd;
int gpsFd;
int proximityFd;
int imuFd;
int audioFd; 
	
timecomp pictimecomp={"000000000",0};

int jpg_ctrl=0;

unsigned char stop_flag=0;

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
#define LEN_SENSOR	512//256
MPU9250_sensor MPU9250_DATA[2*LEN_SENSOR];

// Mandatory variables for audio 
#define Format	16
#define SamplingRate  24000
#define BUF_SIZE SamplingRate*(Format/8)*20 //20 seconds


int cnt_sensor_sampling=0;


int imu_file_open=0;

pthread_mutex_t lock;
pthread_t imuThread,imusaveThread, mcuThread,camThread,gpsThread,dmicThread, poweronoffThread;

option_struct  global_options;

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
	ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
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

int process_image(unsigned char *addr, int length, char *imFile)
{
	FILE *fp;
	
	int i;
	unsigned short valid_bytes_line;
	unsigned char *jpg_file;// = malloc(IM_WIDTH*IM_HEIGHT*sizeof(unsigned char));
	unsigned char *jpg_raw;
	unsigned int jpg_size;
	unsigned char *line_end;



	{
		jpg_raw = (unsigned char *)addr;
		jpg_file = (unsigned char *)addr;
		jpg_size = 0;


		//printf("process_image...\n");
		//if(((jpg_raw[0]==0xff)&&(jpg_raw[1]==0xD8))&&(!((jpg_raw[1280]==0xff)&&(jpg_raw[1281]==0xD8))))//check valid data
		//if((jpg_raw[0]==0xFF)&&(jpg_raw[1]==0xD8))
		{	 
			 
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


			while(flag_newdata)//wait for saving sensor data to be done
				;

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
				addr[0]=0;

			fwrite(addr, 4, ((jpg_size+3)/4), fp);
			//usleep(500);
			fclose(fp);

			printf("ebutton, jpg_size: %d\n", jpg_size);
			printf(imFile);
			printf("\n");

			//system("sync");
		}
		
	}
	
	return 0;
}

int read_frame(int fd, char save_en)
{
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
		
		//multi frames per second
		sprintf(tm,"%s%c%s",time_struct.time_min,'.',time_struct.time_sec);
		if (!strcmp(pictimecomp.pre_time, tm))
		{
			pictimecomp.inx++;
		}
		else
		{
			pictimecomp.inx=0;
			strcpy(pictimecomp.pre_time, tm);
		}

		//printf("store data to %s\n",PATH); //PATH="/sdcard/"
		strcpy(moDayDir,PATH);
		//printf("moDayDir is %s\n",moDayDir);

		sprintf(FOLDER_NAME,"%s%s%c%s%c%02d","ID",deviceID, '_',time_struct.time_month,'.',atoi(time_struct.time_day));	 //month.day

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
		system(cmd);
		//******************************************************* end ********************************************************************//

		//get_time(&time_struct);
		sprintf(yrMonDayHrMinSec,"%s%s%02d%c%s%s%s%c%d",time_struct.time_year,time_struct.time_month,atoi(time_struct.time_day),'_',time_struct.time_hour,time_struct.time_min, time_struct.time_sec,'_',pictimecomp.inx);
		//strcat(yrMonDayHrMinSec,".dat");


		//sprintf(imFile,"/boot/uboot/Data/%d%s",k,".jpg");
		sprintf(imFile,"%s%s%c%s%s",hourDir,deviceID,'_',yrMonDayHrMinSec,".jpg");

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


int audio_read(int audio_fd, char *imFile, unsigned char *audio_buffer, int recording_len)
{
	int audio_count;
	int read_len;
	FILE *fp;

	audio_count = recording_len;
	if ((read_len = read(audio_fd, audio_buffer, audio_count)) == -1) 
	{ 
		perror("audio read"); exit(1); 
	} 

	if((fp = fopen(imFile, "wb")) == NULL){
			perror("Fail to fopen");
			exit(EXIT_FAILURE);
	}
	
	fwrite(audio_buffer, 1, read_len, fp);
	//usleep(500);
	fclose(fp);

	return read_len;
}


void create_HourPath(char *hourDir)
{
	static Current_tm time_struct_t={"0000","000","00","00","00","00"};
	char sys_cmd_t[MAXSTRING];
	char cmd_t[MAXSTRING];
	
	//char hourDir[MAXSTRING];
	char FOLDER_NAME_t[MAXSTRING];
	//char yrMonDayHrMinSec_t[MAXSTRING];

	//************************************************* Path generation **************************************************************************//
	get_time(&time_struct_t);

	//printf("store data to %s\n",PATH); //PATH="/sdcard/"
	strcpy(moDayDir_t,PATH);
	//printf("moDayDir is %s\n",moDayDir);

	sprintf(FOLDER_NAME_t,"%s%s%c%s%c%02d","ID",deviceID, '_',time_struct_t.time_month,'.',atoi(time_struct_t.time_day));	 //month.day

	strcat(moDayDir_t,FOLDER_NAME_t);
	//printf("folder is %s\n",FOLDER_NAME);
	//printf("moDayDir is %s\n",moDayDir); 
	strcpy(sys_cmd_t,"mkdir -p ");
	strcat(sys_cmd_t,moDayDir_t);
	system(sys_cmd_t);

	
	sprintf(moDayDir_t,"%s%c",moDayDir_t,'/'); //moDayDir: sdcard/month.day/
	sprintf(moDayDir_sensor,"%s",moDayDir_t);

	//get_time(&time_struct); //update time

	// create a folder named as the current hour 
	sprintf(hourDir,"%s%d%c",moDayDir_t,atoi(time_struct_t.time_hour),'/');// moDayDir: sdard/month.day/; folderHead: hour
	//printf("hourDir is %s\n",hourDir);
	strcpy(cmd_t,"mkdir -p ");
	strcat(cmd_t,hourDir);
	system(cmd_t);

	//******************************************************* end ********************************************************************//
}


void *imuFunction()
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

	printf("Starting capture imu data!!!\n");

	MPU9250_sensor *pMPU9250_DATA;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

/*
	imu_samplingrate = 80;
	imu_buffer = (short *)malloc( imu_samplingrate * 20 * sizeof(short)); 
	
	

	while (1)
	{
		create_HourPath(hourDir);
		sprintf(imuFile,"%s%s",hourDir,"imu_date.txt");
		//printf(imuFile);
		//printf("\n");

		if((fp = fopen(imuFile, "a")) == NULL){
				perror("Fail to open imu file");
				exit(EXIT_FAILURE);
		}

		if ((read_len = read(imuFd, imu_buffer, imu_samplingrate)) == -1) 
		{ 
			perror("imu device read"); exit(1); 
		} 
		for (i=0;i<imu_samplingrate;i++)
		{
			fprintf(fp, "%d,\n",imu_buffer[i]);
			printf("%d,\n",imu_buffer[i]);
		}
		
		get_time(&time_struct);*/
		/*stream=fopen("/sys/bus/iio/devices/iio:device1/in_accel_z_raw","r+");
		fscanf(stream,"%s", acc_z);
		fprintf(fp,"%s,%s,\n",time_struct.time_sec, acc_z);
		//printf("id0000: %s,%s,\n",time_struct.time_sec, acc_z);
		fclose(stream);*/

		/*//usleep(500);
		fclose(fp);
	}

	free(imu_buffer);*/

	delay_cnt = 0;
	k=0;	
	flag_newdata = 0;
	while(!stop_flag)//1)
	{
		//if(en_sensor_sampling)
		{
			//printf("%d\n",k);
			pMPU9250_DATA = MPU9250_DATA+LEN_SENSOR*flag_sensorbuffer+k;
		
			Motion_MPU9250_REGs_rd(pMPU9250_DATA);
				//printf("%f,%f,%f,%f,%f,%f,%f, %f,%f,%f\n\r",MPU9250_DATA.ACC_fl[0],MPU9250_DATA.ACC_fl[1],MPU9250_DATA.ACC_fl[2], MPU9250_DATA.TEMP_fl,MPU9250_DATA.GYRO_fl[0],MPU9250_DATA.GYRO_fl[1],MPU9250_DATA.GYRO_fl[2], MPU9250_DATA.Magnetometer[0],MPU9250_DATA.Magnetometer[1],MPU9250_DATA.Magnetometer[2]);

			get_time(&time_struct);
		
			pMPU9250_DATA->SensorHour = atoi(time_struct.time_hour);
			pMPU9250_DATA->SensorMin = atoi(time_struct.time_min);
			pMPU9250_DATA->SensorSec = atoi(time_struct.time_sec);
			if(k==(LEN_SENSOR-1))
			{
				k=0;

				if(flag_sensorbuffer==0)
					flag_sensorbuffer = 1;
				else if(flag_sensorbuffer==1)
					flag_sensorbuffer = 0;

				flag_newdata = 1;//new data is ready to be saved into file
			}
			else
				k++;
		
			//slow down sampling rate	
			for(delay_cnt=0;delay_cnt<50000;delay_cnt++)
			{;}

			//en_sensor_sampling=0;

			//printf("%02d%02d,%f,%f,%f,%f\n",pMPU9250_DATA->SensorMin, pMPU9250_DATA->SensorSec, pMPU9250_DATA->ACC_fl[0], pMPU9250_DATA->ACC_fl[1], pMPU9250_DATA->ACC_fl[2],  pMPU9250_DATA->TEMP_fl);
			//fprintf(fp,"%02d%02d,%f,%f,%f,%f,%f,%f,%f, %f,%f,%f\n",atoi(time_struct.time_min), atoi(time_struct.time_sec), MPU9250_DATA.ACC_fl[0],MPU9250_DATA.ACC_fl[1],MPU9250_DATA.ACC_fl[2], MPU9250_DATA.TEMP_fl,MPU9250_DATA.GYRO_fl[0],MPU9250_DATA.GYRO_fl[1],MPU9250_DATA.GYRO_fl[2], MPU9250_DATA.Magnetometer[0],MPU9250_DATA.Magnetometer[1],MPU9250_DATA.Magnetometer[2]);
	
		}
	}

	//pthread_exit(NULL);
}


void *imuDataSaveFunction()
{
	int k;

	char hourDir[MAXSTRING];
	char imuFile[MAXSTRING];
	FILE *fp;

	MPU9250_sensor *psaveMPU9250_DATA;
	
	char min_old;
	char imu_file_tail=0;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	min_old = 0;
	while(!stop_flag)//1)
	{
		if(flag_newdata)//new data is availible
		{
			if(!imu_file_open){
				create_HourPath(hourDir);	
				psaveMPU9250_DATA = MPU9250_DATA+LEN_SENSOR*(1-flag_sensorbuffer);
				sprintf(hourDir,"%s%d%c",moDayDir_t,psaveMPU9250_DATA->SensorHour,'/');//
				imu_file_tail = (psaveMPU9250_DATA->SensorMin)/5;//new file every 5 minutes
				sprintf(imuFile,"%s%s%02d%s",hourDir,"imu_sensordate_",imu_file_tail,".txt");
				//sprintf(imuFile,"%s%s",moDayDir_sensor,"imu_sensordate.txt");

			
				if((fp = fopen(imuFile, "a")) == NULL){
						perror("Fail to open imu file");
						exit(EXIT_FAILURE);
				}
				imu_file_open = 1;
			}
				//printf("%f,%f,%f,%f,%f,%f,%f, %f,%f,%f\n\r",MPU9250_DATA.ACC_fl[0],MPU9250_DATA.ACC_fl[1],MPU9250_DATA.ACC_fl[2], MPU9250_DATA.TEMP_fl,MPU9250_DATA.GYRO_fl[0],MPU9250_DATA.GYRO_fl[1],MPU9250_DATA.GYRO_fl[2], MPU9250_DATA.Magnetometer[0],MPU9250_DATA.Magnetometer[1],MPU9250_DATA.Magnetometer[2]);
			

			for (k=0;k<LEN_SENSOR;k++)
			{
				psaveMPU9250_DATA = MPU9250_DATA+LEN_SENSOR*(1-flag_sensorbuffer)+k;

				fprintf(fp,"%02d%02d%02d,%.3f,%.3f,%.3f, %5.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",psaveMPU9250_DATA->SensorHour, psaveMPU9250_DATA->SensorMin, psaveMPU9250_DATA->SensorSec, psaveMPU9250_DATA->ACC_fl[0],psaveMPU9250_DATA->ACC_fl[1],psaveMPU9250_DATA->ACC_fl[2], psaveMPU9250_DATA->TEMP_fl,psaveMPU9250_DATA->GYRO_fl[0],psaveMPU9250_DATA->GYRO_fl[1],psaveMPU9250_DATA->GYRO_fl[2], psaveMPU9250_DATA->Magnetometer[0],psaveMPU9250_DATA->Magnetometer[1],psaveMPU9250_DATA->Magnetometer[2]);
			}

			if(min_old!=psaveMPU9250_DATA->SensorMin){ //close the file every minute
				fclose(fp);
				min_old = psaveMPU9250_DATA->SensorMin;
				imu_file_open = 0;
			}
			
			flag_newdata=0;

			system("sync");
		}

	}

	//pthread_exit(NULL);
}

void *camFunction()
{
	int kc;
	
	int i;

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
	while (!stop_flag)//1)
	{		
			if(low_power_mode==1)
			{
				//turn on green led
				system("echo 1 > /sys/class/leds/led_green/brightness");

				//skip the first several frames to get stable image
				for(i=0;i<15;i++)
				{
					read_frame(camFd, (i==14));
				}
	
				stop_capture(camFd);//stop camera

				//turn off green led
				system("echo 0 > /sys/class/leds/led_green/brightness");

				init_camera_device(camFd);//reopen camera
				start_capture(camFd);
			}
			else if(low_power_mode==2)//lowest power consumption
			{
				//turn on green led
				system("echo 1 > /sys/class/leds/led_green/brightness");

				//no skip, image will have color degradation
				read_frame(camFd, 1);
	
				stop_capture(camFd);//stop camera

				init_camera_device(camFd);//reopen camera
				start_capture(camFd);

				//turn off green led
				system("echo 0 > /sys/class/leds/led_green/brightness");

			}		
			else if(low_power_mode==0)
			{
				//set green led in heartbeat mode
				system("echo heartbeat > /sys/class/leds/led_green/trigger");

				read_frame(camFd, 1);
			}

			kc++;

			if(kc==10)
			{
				system("sync");
				kc=0;
			}

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
	
	
void *dmicFunction()
{	
	int k;
	
	static Current_tm time_struct={"0000","000","00","00","00","00"};
	//char sys_cmd[MAXSTRING];
	//char cmd[MAXSTRING];
	//char moDayDir[MAXSTRING];
	char hourDir[MAXSTRING];
	//char FOLDER_NAME[MAXSTRING];
	char yrMonDayHrMinSec[MAXSTRING];
	char auFile[MAXSTRING];
	
	unsigned char *audio_buffer;

	//enable cancellation
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

	k = 0;
	while (!stop_flag)//1)
	{
		create_HourPath(hourDir);
		//************************************************* Path generation **************************************************************************//
		/*get_time(&time_struct);

		//printf("store data to %s\n",PATH); //PATH="/sdcard/"
		strcpy(moDayDir,PATH);
		//printf("moDayDir is %s\n",moDayDir);

		sprintf(FOLDER_NAME,"%s%s%c%s%c%02d","ID",deviceID, '_',time_struct.time_month,'.',atoi(time_struct.time_day));	 //month.day

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

		get_time(&time_struct);
		sprintf(yrMonDayHrMinSec,"%s%s%02d%c%s%s%s",time_struct.time_year,time_struct.time_month,atoi(time_struct.time_day),'_',time_struct.time_hour,time_struct.time_min, time_struct.time_sec);
		//strcat(yrMonDayHrMinSec,".dat");

		audio_buffer = (unsigned char *)malloc( BUF_SIZE * sizeof(unsigned char)); 
		//sprintf(auFile,"/boot/uboot/Data/%d%s",k,".hex");
		sprintf(auFile,"%s%s%c%s%s",hourDir,deviceID,'_',yrMonDayHrMinSec,".hex");
		printf(auFile);
		printf("\n");
		audio_read(audioFd, auFile, audio_buffer, BUF_SIZE );
		free(audio_buffer);
		
		k++;

		//system("sync");
		if(k==10)
		{
			system("sync");
			k=0;
		}
	}

	//pthread_exit(NULL);

}


/*
void timer_handler_sensor(void)
{
	cnt_sensor_sampling++;
	en_sensor_sampling=1;

	//printf("timer triggered!\n");
}*/


void *soft_poweronoff()
{	

  	//int keys_fd;
  	//char ret[2];
  	//struct input_event t;
  	//struct timeval start, end;
  	int interval;

	static Current_tm time_struct_start={"0000","000","00","00","00","00"};
	static Current_tm time_struct_end={"0000","000","00","00","00","00"};

	FILE *nlbo_fd, *user_fd, *usb_fd;
	char nlbo[1], user[1], usb[1];

  	interval = 0;

	
	stop_flag = 0;

	while (1)
	{
		
/*
		usb[0] = 0;
	  	usb_fd = fopen("/sys/class/gpio/pioE9/value", "rb");
	  	fread(usb, sizeof(char), 1, usb_fd);
	  	printf("usb gpio: %d\n",atoi(usb));
	  	fclose(usb_fd);

		if(atoi(usb)==1){//usb connected, stop lp_ebutton

		
			//system("pkill lp_ebutton"); //kill ebutton app
			stop_flag = 1;

			//blue led
			system("echo none > /sys/class/leds/led_red/trigger");
			system("echo 0 > /sys/class/leds/led_red/brightness");
			system("echo none > /sys/class/leds/led_green/trigger");
			system("echo 0 > /sys/class/leds/led_green/brightness");
			system("echo none > /sys/class/leds/led_blue/trigger");
			system("echo 1 > /sys/class/leds/led_blue/brightness");

			system("sync");

			//load USB mass storage
			system("insmod /boot/uboot/system/gadget/libcomposite.ko");
			system("insmod /boot/uboot/system/gadget/function/usb_f_mass_storage.ko");
			system("insmod /boot/uboot/system/gadget/legacy/g_mass_storage.ko file=/dev/loop0 stall=0 removable=1");


			while(atoi(usb)==1)
			{

				//blue led
				system("echo none > /sys/class/leds/led_red/trigger");
				system("echo 0 > /sys/class/leds/led_red/brightness");
				system("echo none > /sys/class/leds/led_green/trigger");
				system("echo 0 > /sys/class/leds/led_green/brightness");
				system("echo heartbeat > /sys/class/leds/led_blue/trigger");

				usb[0] = 0;
			  	usb_fd = fopen("/sys/class/gpio/pioE9/value", "rb");
			  	fread(usb, sizeof(char), 1, usb_fd);
			  	printf("usb gpio: %d\n",atoi(usb));
			  	fclose(usb_fd);
			}

		
			system("poweroff");
			
		}
		else{
			//check battery, if low, shutdown

			//remove usb module
			//system("rmmod /boot/uboot/system/gadget/legacy/g_mass_storage.ko");
			//system("rmmod /boot/uboot/system/gadget/function/usb_f_mass_storage.ko");
			//system("rmmod /boot/uboot/system/gadget/libcomposite.ko");

			nlbo[0] = 0;
		  	nlbo_fd = fopen("/sys/class/gpio/pioE24/value", "rb");
		  	fread(nlbo, sizeof(char), 1, nlbo_fd);
		  	printf("unlbo gpio: %d\n",atoi(nlbo));
		  	fclose(nlbo_fd);
			//low battery
			if(atoi(nlbo)==0){
				
				//blue led
				system("echo none > /sys/class/leds/led_red/trigger");
				system("echo 0 > /sys/class/leds/led_red/brightness");
				system("echo none > /sys/class/leds/led_green/trigger");
				system("echo 0 > /sys/class/leds/led_green/brightness");
				system("echo heartbeat > /sys/class/leds/led_blue/trigger");

				//system("pkill lp_ebutton"); //kill ebutton app
				stop_flag = 1;

				system("sync");
				system("poweroff");
			}
		}*/

		//detect poweroff button
		user[0] = 0;
  		user_fd = fopen("/sys/class/gpio/pioD20/value", "rb");
  		fread(user, sizeof(char),1, user_fd);
  		printf("user gpio: %d\n",atoi(user));
  		fclose(user_fd);
		
		//gettimeofday(&start, NULL);//start time
		get_time(&time_struct_start);

		interval = 0;
		if(atoi(user)==1)
		{

			pthread_mutex_lock(&lock);

			//printf ("key pressed, time %ld\n", start.tv_sec);
			printf ("key pressed, time %d\n", atoi(time_struct_start.time_sec));

			do{
				user[0] = 0;
		  		user_fd = fopen("/sys/class/gpio/pioD20/value", "rb");
		  		fread(user, sizeof(char),1, user_fd);
		  		printf("user gpio: %d\n",atoi(user));
		  		fclose(user_fd);
				
				//gettimeofday(&end, NULL);	
				get_time(&time_struct_end);	

				interval = abs(atoi(time_struct_end.time_sec) - atoi(time_struct_start.time_sec));//(int)(end.tv_sec - start.tv_sec);
				printf ("interval time %d\n", interval);
				if(interval>=5)
				{
					printf ("pressed time is enough! system will poweroff\n");


					//if(lp_ebutton_running) 
					//system("pkill lp_ebutton"); //kill ebutton app
					stop_flag = 1;

					if (global_options.CAMERA_ON)  pthread_cancel(camThread);
					if (global_options.DMIC_ON)	pthread_cancel(dmicThread);
					if (global_options.IMU_ON)	{
						pthread_cancel( imuThread);
						pthread_cancel( imusaveThread);
					}

					//turn off leds
					//blue led
					system("echo none > /sys/class/leds/led_red/trigger");
					system("echo 0 > /sys/class/leds/led_red/brightness");
					system("echo none > /sys/class/leds/led_green/trigger");
					system("echo 0 > /sys/class/leds/led_green/brightness");
					system("echo heartbeat > /sys/class/leds/led_blue/trigger");

					//system("rmmod /boot/uboot/system/gadget/legacy/g_mass_storage.ko");
					//system("rmmod /boot/uboot/system/gadget/function/usb_f_mass_storage.ko");
					//system("rmmod /boot/uboot/system/gadget/libcomposite.ko");

					//shutdown system//
					system("sync");
					system("shutdown -h now");//system("poweroff");
				}
				interval = 0;
			}while(atoi(user)==1);

			
			pthread_mutex_unlock(&lock);
		}
		
	}

}

int main(int argc, char *argv[])
{
	//int audio_len;
	//int audio_count;
	int audio_format; 
	int audio_speed;
	
		

	//int ret;
	//int start;
	//int k,i;
	char *config_file="/media/lpebutton/app/config";//"/boot/uboot/system/config";
	
	//struct v4l2_format codec_fmt;
	struct v4l2_requestbuffers bufrequest;
	//struct v4l2_buffer bufferinfo;
	//struct v4l2_capability codec_cap;

	char rtc_cmd[MAXSTRING];
	//char *deviceIDfile="/boot/uboot/system/DeviceID";
	//FILE *idFid;

	//static Current_tm time_struct={"0000","000","00","00","00","00"};
	
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
	
	//backup system app and config
	//system("cp /media/lpebutton/app/lp_ebutton /boot/uboot/system/");
	//system("cp /media/lpebutton/app/config /boot/uboot/system/");

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
	initialize_global(&global_options);

	
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


	/*if (global_options.DEBUG)
	{
		//free(global_options);
		printf("exiting program\n");
		exit(0);
	}*/

	//100HZ
	/*if(start_timer(3, &timer_handler_sensor))
	{
		printf("\n timer error\n");
		return(1);
	}*/


	if (global_options.RTC_UPDATE)
	{

		sprintf(rtc_cmd,"%s%c%s","date",' ', global_options.RTC_TIME);
		system(rtc_cmd);
		system(hwclock_rtc);
	}


	if (pthread_mutex_init(&lock, NULL) != 0)
	    {
		printf("\n mutex init failed\n");
		return 1;
	    }

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

		pthread_create( &camThread, NULL, camFunction, NULL);
	}
	else
		system("echo heartbeat > /sys/class/leds/led_green/trigger");//blink green led for other sensor

	if (global_options.IMU_ON)
	{

		printf("Open imu...\n");

		//(spi_speed_set， gyro_fs, acc_fs)
		if(global_options.IMU_SPEED>5000000)
			global_options.IMU_SPEED = 5000000;
		if(global_options.IMU_SPEED<600000)
			global_options.IMU_SPEED = 600000;		
		inv_mpu9250_init_config(global_options.IMU_SPEED, global_options.GYRO_FS , global_options.ACC_FS );

		pthread_create( &imuThread, NULL, imuFunction, NULL);
		pthread_create( &imusaveThread, NULL, imuDataSaveFunction, NULL);
	}

	pthread_create( &poweronoffThread, NULL, soft_poweronoff, NULL);

/*
	if (global_options -> LIGHT_SENSOR_ON)//if (global_options -> LIGHT_SENSOR_ON || global_options -> ACCELE_ON)
	{
		adcFd=openADC(adc_DEV);
	}


	if (global_options	-> GPS_ON)
	{
		gpsFd=openSPI(gps_DEV);

		///////////////////////////////////////////////////////////////////
		// 		          start
		//	gps standby demo         added by byc 10/17/2012
		///////////////////////////////////////////////////////////////////
		int fd_gps_normal;
		fd_gps_normal = open("/dev/gpsnormal",0);
		if (fd_gps_normal < 0)
		{
			perror("open gpsstandby device:");
			return 1;
		}
		fprintf(stderr,"\ngpsnormal device is opened!\n");
		close(fd_gps_normal);
		///////////////////////////////////////////////////////////////////
		//    			   end
		//	gps standby demo         added by byc 10/17/2012
		///////////////////////////////////////////////////////////////////
	}

        if (global_options	-> INERTIAL_ON)
	{
		inertialFd=openINERTIAL(inertial_DEV);
	}

	if (global_options	-> GYROSCOPE_ON)
	{
		gyroFd=openGYROSCOPE(gyro_DEV);
	}

	if (global_options	-> BAROMETER_ON)
	{
		baroFd=openBARO(baro_DEV);
	}

	if (global_options	-> PROXIMITY_ON)
	{
		proximityFd=openPROXIMITY(proximity_DEV);
	}

	// retrieve device ID by reading file "/root/DeviceID"

	idFid=fopen(deviceIDfile,"r");
	if (idFid!=NULL)
	{
		fgets(deviceID, 4, idFid);
		fclose(idFid);
	}

*/

/*
		//open imu device
		if ((imu_fd = open(imu_DEV, O_RDONLY, 0)) < 0) 
		{ 
			perror(imu_DEV); 
			exit(1); 
		} 
		printf("imu opened!\n");
*/
	if (global_options.DMIC_ON)
	{
		//open sound device
		if ((audioFd = open(dmic_DEV, O_RDONLY, 0)) < 0) 
		{ 
			/* Open of device failed */ 
			perror(dmic_DEV); 
			exit(1); 
		} 
		printf("DSP opened!\n");

		//format setting
		audio_format = AFMT_S16_LE; 
		printf("format set: %d\n", audio_format);
		if (ioctl(audioFd, SNDCTL_DSP_SETFMT, &audio_format) == -1) 
		{ 
			/* fatal error */ 
			perror("SNDCTL_DSP_SETFMT"); 
			exit(1); 
		} 
		if (audio_format != AFMT_S16_LE) //16bits, little endian
		{ 
			printf("Not supported!\n");
		} 
		 printf("format set done: %d\n", audio_format);

		audio_speed = SamplingRate;
		printf("Sampling rate initial: %dHz\n", audio_speed);
		if (ioctl(audioFd, SNDCTL_DSP_SPEED, &audio_speed)==-1) 
		{ 
			perror("SNDCTL_DSP_SPEED"); 
			exit(1); 
		} 
		printf("Sampling rate is set: %dHz\n", audio_speed);
		
		//setup thread
		pthread_create( &dmicThread, NULL, dmicFunction, NULL);
	}


	//turn off
	system("echo 0 > /sys/class/leds/led_red/brightness");
	system("echo 0 > /sys/class/leds/led_blue/brightness");
	system("echo 0 > /sys/class/leds/led_green/brightness");

	pthread_join( poweronoffThread, NULL);

	if (global_options.CAMERA_ON) pthread_join( camThread, NULL);
	if (global_options.DMIC_ON)	pthread_join( dmicThread, NULL);
	if (global_options.IMU_ON)	{
		pthread_join( imuThread, NULL);
		pthread_join( imusaveThread, NULL);
	}


	pthread_mutex_destroy(&lock);

	/*k=1;
	while (1)
	{*/

		//************************************************* Path generation **************************************************************************//
		/*get_time(&time_struct);

		//printf("store data to %s\n",PATH); //PATH="/sdcard/"
		strcpy(moDayDir,PATH);
		//printf("moDayDir is %s\n",moDayDir);

		sprintf(FOLDER_NAME,"%s%s%c%s%c%02d","ID",deviceID, '_',time_struct.time_month,'.',atoi(time_struct.time_day));	 //month.day

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

/*		get_time(&time_struct);
		sprintf(yrMonDayHrMinSec,"%s%s%02d%c%s%s%s",time_struct.time_year,time_struct.time_month,atoi(time_struct.time_day),'_',time_struct.time_hour,time_struct.time_min, time_struct.time_sec);
		//strcat(yrMonDayHrMinSec,".dat");



		if (global_options -> CAMERA_ON)
		{	
			//sprintf(imFile,"/boot/uboot/Data/%d%s",k,".jpg");
			sprintf(imFile,"%s%s%c%s%s",hourDir,deviceID,'_',yrMonDayHrMinSec,".jpg");
			
			//printf(imFile);
			//printf("\n");
			read_frame(camFd, imFile);
		}
			
		if (global_options -> DMIC_ON)
		{
			audio_buffer = (unsigned char *)malloc( BUF_SIZE * sizeof(unsigned char)); 
			//sprintf(auFile,"/boot/uboot/Data/%d%s",k,".hex");
			sprintf(auFile,"%s%s%c%s%s",hourDir,deviceID,'_',yrMonDayHrMinSec,".hex");
			audio_read(audio_fd, auFile, audio_buffer, BUF_SIZE/3 );
			free(audio_buffer);
		}
*/
		

/*		retrieve_data(deviceID, global_options, IM_WIDTH, IM_HEIGHT, adcFd,gpsFd, camFd, gyroFd, baroFd, inertialFd, magFd, proximityFd, &pictimecomp);
		printf("inerval is %d\n",global_options	-> CAMERA_INTERVAL);

		if (global_options	-> CAMERA_INTERVAL)
		{
			sleep(global_options	-> CAMERA_INTERVAL);
		}

		if (global_options -> SLEEP)
		{
			if (global_options -> CAMERA_ON)
			{
				camFd=openCAM(cam_DEV);
				ret = ioctl(camFd , VIDIOC_S_FMT, &codec_fmt);
				if (ret < 0) {
					printf("V4L2 : ioctl on VIDIOC_S_FMT failled\n");
					exit(1);
				}

				start=1;
				if(ioctl(camFd, VIDIOC_STREAMON, &start)<0)
				{
					printf("V4L2 : ioctl on VIDIOC_STREAMON failed\n");
					exit(1);
				}
			}
			if (global_options-> LIGHT_SENSOR_ON || global_options-> ACCELE_ON)
			{
				adcFd=openADC(adc_DEV);
			}
			if (global_options-> GPS_ON)
			{
				gpsFd=openSPI(gps_DEV);

			}
			if (global_options	-> INERTIAL_ON)
			{
				inertialFd=openINERTIAL(inertial_DEV);
			}

			if (global_options	-> GYROSCOPE_ON)
			{
				gyroFd=openGYROSCOPE(gyro_DEV);
			}

			if (global_options	-> BAROMETER_ON)
			{
				baroFd=openBARO(baro_DEV);
			}
			if (global_options	-> PROXIMITY_ON)
			{
				proximityFd=openPROXIMITY(proximity_DEV);
			}
		}
*/
/*		k++;

		if(k%150==0)
		{
			system("sync");
			k=1;
		}
	}

	if (global_options -> CAMERA_ON)
	{

		start = 0;
		stop_capture(camFd);
		close_camera_device(camFd);
	}
	
	free(global_options);
	close(adcFd);
	close(gpsFd);
        close(inertialFd);
	close(camFd);
	close(baroFd);
	close(gyroFd);
	close(proximityFd);*/
	return 0;
}

