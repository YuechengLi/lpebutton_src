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

void main ()
{
	time_t time_shutdown;
	FILE *fp_time;

	int keys_fd;
	char ret[2];
	int i;
	struct input_event t;
	time_t start, end;
	//struct timeval start, end;
	int trig;
	int interval;

	//wait for main app to start
	for (i=0;i<100000;i++)
	{;}

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
		        	time(&start);//gettimeofday(&start, NULL);
				printf ("key pressed, time %ld\n", start);//.tv_sec);			
			}
			else if (t.value == 0)
			{
				time(&end);//gettimeofday(&end, NULL);
				printf ("key released, time %ld\n", end);//.tv_sec);

				interval = difftime(end,start);//(int)(end.tv_sec - start.tv_sec);
				printf ("interval time %d\n", interval);
				if(interval>=2)
				{
					printf ("pressed time is enough! system will poweroff\n");


					//turn off leds
					//blue led
					system("echo none > /sys/class/leds/led_red/trigger");
					system("echo 0 > /sys/class/leds/led_red/brightness");
					system("echo none > /sys/class/leds/led_green/trigger");
					system("echo 0 > /sys/class/leds/led_green/brightness");
					system("echo heartbeat > /sys/class/leds/led_blue/trigger");

					//if(lp_ebutton_running) 
					system("pkill -f lp_ebutton"); //kill ebutton app
					//stop_flag = 1;

					//turn off leds
					system("echo none > /sys/class/leds/led_red/trigger");
					system("echo 1 > /sys/class/leds/led_red/brightness");
					system("echo none > /sys/class/leds/led_green/trigger");
					system("echo 0 > /sys/class/leds/led_green/brightness");

					//system("rmmod /boot/uboot/system/gadget/legacy/g_mass_storage.ko");
					//system("rmmod /boot/uboot/system/gadget/function/usb_f_mass_storage.ko");
					//system("rmmod /boot/uboot/system/gadget/libcomposite.ko");

					//shutdown system//
					//system("sync");
					system("umount /media/lpebutton");
					for (i=0;i<10000;i++)
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

					system("poweroff");//system("shutdown -h now");//
				}
				interval = 0;
			}
		}
	}
	close (keys_fd);


}
