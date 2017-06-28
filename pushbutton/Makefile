.SUFFIXES : .c .o

OBJECTS =	usb_detect.o\
		lb_detect.o\
		softp.o

SRCS = 		usb_detect.c\
		lb_detect.c\
		softp.c

#DEPENDENCY =   	get_time.h

CFLAGS = -Wall -I ../ -I /home/ycli/work/A5/lpebutton_A5_2016/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/usr/include #-g #-Os -Wall
##INC += -I$(KERNEL_PATH)/include     ## peter added

LDFLAGS = -L ../ -L /home/ycli/work/A5/lpebutton_A5_2016/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/usr/lib 



all: usb_detect lb_detect softp

TARGET = usb_detect lb_detect softp


usb_detect:usb_detect.c
	$(CC)gcc $(CFLAGS) $(LDFLAGS) -lm -o usb_detect usb_detect.c

lb_detect:lb_detect.c
	$(CC)gcc $(CFLAGS) $(LDFLAGS) -lm -o lb_detect lb_detect.c

softp:softp.c
	$(CC)gcc $(CFLAGS) $(LDFLAGS) -lm -o softp softp.c

#$(TARGET) : $(SRCS)
#		$(CC)gcc $(CFLAGS) $(LDFLAGS) -lm -o $(TARGET) $(SRCS) #$(CC)gcc -lm -o $(TARGET) $(SRCS) #$(OBJECTS)

.PHONY: all clean 

clean :
		rm -rf $(OBJECTS) $(TARGET) core
