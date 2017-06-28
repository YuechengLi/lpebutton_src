#include <stdio.h>  
#include <sys/time.h>  
#include <linux/input.h>  
#include <stdlib.h>  
#include <sys/stat.h> 
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include <stdlib.h>
#include <string.h>

//#include "get_time.h"

int main ()
{
  	//int keys_fd;
  	//char ret[2];
  	//struct input_event t;
  	//struct timeval start, end;
  	//int interval;

	//static Current_tm time_struct_start={"0000","000","00","00","00","00"};
	//static Current_tm time_struct_end={"0000","000","00","00","00","00"};

	FILE *nlbo_fd, *usb_fd;
	//FILE *user_fd;
	char nlbo[1], usb[1];
	//char user[1];

  	//interval = 0;
	int i;


	while (1)
	{
		//stop_flag = 0;

		
		for (i=0;i<10000;i++)
		{;}


		//USB connection
		usb[0] = 0;
	  	usb_fd = fopen("/sys/class/gpio/pioE9/value", "rb");
	  	fread(usb, sizeof(char), 1, usb_fd);
	  	printf("usb gpio: %d\n",atoi(usb));
	  	fclose(usb_fd);

		//low battery
		nlbo[0] = 0;
		nlbo_fd = fopen("/sys/class/gpio/pioE24/value", "rb");
		fread(nlbo, sizeof(char), 1, nlbo_fd);
		printf("unlbo gpio: %d\n",atoi(nlbo));
		fclose(nlbo_fd);

		if(atoi(usb)==1){//usb connected, stop lp_ebutton

			//green led
			system("echo none > /sys/class/leds/led_red/trigger");
			system("echo 0 > /sys/class/leds/led_red/brightness");
			system("echo none > /sys/class/leds/led_green/trigger");
			system("echo 1 > /sys/class/leds/led_green/brightness");
			system("echo none > /sys/class/leds/led_blue/trigger");
			system("echo 0 > /sys/class/leds/led_blue/brightness");
		
			system("pkill -9 lp_ebutton"); //kill ebutton app
			//stop_flag = 1;


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
				system("echo none > /sys/class/leds/led_blue/trigger");
				system("echo 1 > /sys/class/leds/led_blue/brightness");


				usb[0] = 0;
			  	usb_fd = fopen("/sys/class/gpio/pioE9/value", "rb");
			  	fread(usb, sizeof(char), 1, usb_fd);
			  	printf("usb gpio: %d\n",atoi(usb));
			  	fclose(usb_fd);
			}


			//confirm USB disconnection and powerdown
			usb[0] = 0;
		  	usb_fd = fopen("/sys/class/gpio/pioE9/value", "rb");
		  	fread(usb, sizeof(char), 1, usb_fd);
		  	printf("usb gpio: %d\n",atoi(usb));
		  	fclose(usb_fd);
			
			if(atoi(usb)==0)
			{
				//blue led
				system("echo none > /sys/class/leds/led_red/trigger");
				system("echo 0 > /sys/class/leds/led_red/brightness");
				system("echo none > /sys/class/leds/led_green/trigger");
				system("echo 0 > /sys/class/leds/led_green/brightness");
				system("echo heartbeat > /sys/class/leds/led_blue/trigger");
		
				system("poweroff");
			}
			
		}
		else if(atoi(nlbo)==0){//check battery, if low, shutdown

			system("pkill -9 lp_ebutton"); //kill ebutton app
			
			//wait for a while then double check
			for (i=0;i<1000;i++)
			{;}

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
			if(atoi(nlbo)==0)
			{

				//blue led
				system("echo none > /sys/class/leds/led_red/trigger");
				system("echo 0 > /sys/class/leds/led_red/brightness");
				system("echo none > /sys/class/leds/led_green/trigger");
				system("echo 0 > /sys/class/leds/led_green/brightness");
				system("echo none > /sys/class/leds/led_blue/trigger");
				system("echo 0 > /sys/class/leds/led_blue/brightness");

				system("sync");
				system("shutdown -h now");//

				//shutdown system
				//system("echo s > /proc/sysrq-trigger");//sync
				//system("echo u > /proc/sysrq-trigger");//umount
				//system("echo o > /proc/sysrq-trigger");//poweroff

			}
		}

		//detect poweroff button
/*		user[0] = 0;
  		user_fd = fopen("/sys/class/gpio/pioD20/value", "rb");
  		fread(user, sizeof(char),1, user_fd);
  		printf("user gpio: %d\n",atoi(user));
  		fclose(user_fd);
		
		//gettimeofday(&start, NULL);//start time
		get_time(&time_struct_start);

		interval = 0;
		if(atoi(user)==1)
		{
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
				if(interval>=3)
				{
					printf ("pressed time is enough! system will poweroff\n");

					//if(lp_ebutton_running) 
					system("pkill lp_ebutton"); //kill ebutton app
					//stop_flag = 1;
			
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

		}*/
		
	}


	return 0;
}
