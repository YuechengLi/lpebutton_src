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

int main ()
{
  	int keys_fd;
  	char ret[2];
  	struct input_event t;
  	struct timeval start, end;
  	int trig;
  	int interval;
  	char poweroff_pressed=0;

  	char *system_poweroff="poweroff";

  	int cnt;

  	cnt = 0;

	FILE *nlbo_fd, *user_fd;
	char nlbo[1], user[1];


  	trig=0;
  	interval = 0;

	while (1)
	{
		nlbo[0] = 0;
	  	nlbo_fd = fopen("/sys/class/gpio/pioE24/value", "rb");
	  	fread(nlbo, sizeof(char), 1, nlbo_fd);
	  	printf("unlbo gpio: %d\n",atoi(nlbo));
	  	fclose(nlbo_fd);
		//low battery
		if(atoi(nlbo)==0){
			system("pkill lp_ebutton"); //kill ebutton app
			system("sync");
			system("poweroff");
		}


		//detect poweroff button
		user[0] = 0;
  		user_fd = fopen("/sys/class/gpio/pioD20/value", "rb");
  		fread(user, sizeof(char),1, user_fd);
  		printf("user gpio: %d\n",atoi(user));
  		fclose(user_fd);
		
		gettimeofday(&start, NULL);//start time

		interval = 0;
		if(atoi(user)==1)
		{
			printf ("key pressed, time %ld\n", start.tv_sec);

			do{
				user[0] = 0;
		  		user_fd = fopen("/sys/class/gpio/pioD20/value", "rb");
		  		fread(user, sizeof(char),1, user_fd);
		  		printf("user gpio: %d\n",atoi(user));
		  		fclose(user_fd);
				
				gettimeofday(&end, NULL);		

				interval = (int)(end.tv_sec - start.tv_sec);
				printf ("interval time %d\n", interval);
				if(interval>=3)
				{
					printf ("pressed time is enough! system will poweroff\n");
					system("pkill lp_ebutton"); //kill ebutton app
					for(cnt=0;cnt<1000;cnt++)
					{;}
					system("sync");
			
					//turn off leds
					system("echo none > /sys/class/leds/led_red/trigger");
					system("echo 0 > /sys/class/leds/led_red/brightness");
					system("echo none > /sys/class/leds/led_blue/trigger");
					system("echo 0 > /sys/class/leds/led_blue/brightness");
					system("echo heartbeat > /sys/class/leds/led_green/trigger");

					//system("rmmod /boot/uboot/system/gadget/legacy/g_mass_storage.ko");
					//system("rmmod /boot/uboot/system/gadget/function/usb_f_mass_storage.ko");
					//system("rmmod /boot/uboot/system/gadget/libcomposite.ko");

					//shutdown system
					system("poweroff");
				}
				interval = 0;
			}while(atoi(user)==1);

		}
		/*else{//delay
			
			while(interval<2)
			{
				gettimeofday(&end, NULL);		

				interval = (int)(end.tv_sec - start.tv_sec);
				printf ("interval time %d\n", interval);
			}
		}*/

	}


	return 0;
}
