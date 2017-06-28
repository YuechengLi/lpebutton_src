#include <stdio.h>  
#include <sys/time.h>  
#include <linux/input.h>  
#include <stdlib.h>  
#include <sys/stat.h> 
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>



int main ()
{
  int keys_fd;
  char ret[2];
  struct input_event t;
  struct timeval start, end;
  int trig;
  int interval;
  
  char *system_poweroff="poweroff";

  int cnt;

  cnt = 0;

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
			if(interval>=3)
			{
				printf ("pressed time is enough! system will poweroff\n");
				system("killall lp_ebutton");
				for(cnt=0;cnt<1000;cnt++)
				{;}

				system("sync");
				system("poweroff");
			}
			interval = 0;
		}

            	/*if (t.value == 0 || t.value == 1)
		{
		      printf ("key %d %s\n", t.code,
		              (t.value) ? "Pressed" : "Released");
		  if(t.code==KEY_ESC)
		      break;
		}*/
        }
    }
  close (keys_fd);

  return 0;
}
