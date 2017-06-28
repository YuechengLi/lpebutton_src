.SUFFIXES : .c .o

OBJECTS =  	initialize_global.o	\
		cmd_proc.o	\
		get_param.o	\
		mpu9250_spi.o	\
		msp430_spi.o	\
		lp_ebutton.o#	\
		#timer.o
		  #retrieve_data.o 	\
		#readSensor.o	\
#LogMsg.o

SRCS = 		initialize_global.c	\
		cmd_proc.c	\
		get_param.c	\
		mpu9250_spi.c	\
		msp430_spi.c	\
		lp_ebutton.c	#\
		#timer.c
 #retrieve_data.c 	\
#		readSensor.c	\
#LogMsg.c	\

DEPENDENCY =   	get_time.h	\
		global_def.h	\
		mpu9250_spi.h	\
		msp430_spi.h	\
		#timer.h		\
		./opus-1.1.3_2/include/opus.h

#		LogMsg.h	
#		lp_ebutton.h	
#		readSensor.h


#videodev2.h 	
#retrieve_data.h						

#CROSS_COMPILE = /usr/local/arm/4.2.2-eabi/usr/bin/arm-linux-
#CC = $(CROSS_COMPILE)gcc -lm 
#AR = $(CROSS_COMPILE)ar

#KERNEL_PATH = /home/fusq/idea6410/urbetter-linux2.6.28
CFLAGS = -Wall -I ../ -I /home/ycli/work/A5/lpebutton_A5_2016/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/usr/include -I ./opus-1.1.3_2/include/ #-g #-Os -Wall
##INC += -I$(KERNEL_PATH)/include     ## peter added

LDFLAGS = -L ../ -L /home/ycli/work/A5/lpebutton_A5_2016/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/usr/lib

LIBS = ./opus-1.1.3_2/.libs/libopus.a ./opus-1.1.3_2/.libs/libopus.so -pthread -lm -lrt

TARGET = lp_ebutton

.PHONY: all clean 

$(TARGET) : $(SRCS)
		$(CC)gcc $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(TARGET) $(SRCS) #$(OBJECTS)
		#$(CC)gcc -lm -o $(TARGET) $(SRCS) #$(OBJECTS)
#		cp -rf ./app_tv /home/fusq/idea6410/app/UT_MultiMedia_test

#.c.o :
#		$(CC) $(INC) $(CFLAGS) $<

clean :
		rm -rf $(OBJECTS) $(TARGET) core
