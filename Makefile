SHELL = /bin/sh

CC = g++
CFLAGS = -std=c++11 -Wall -g

ifeq ($(shell uname -s), Darwin)
    INCS = -I src -I ~/LIBS/opencv40/include
    PLIBS = ~/LIBS/opencv40/lib/
    PLATFORM = MAC
endif
ifeq  ($(shell uname -m),armv7l)
    INCS = -I src -I ~/LIBS/opencv40/include/opencv4
    PLIBS = ~/LIBS/opencv40/lib/
    PLATFORM = RPI
endif
ifeq  ($(shell uname -m),x86_64)
    INCS = -I src -I ~/LIBS/opencv40/include
    PLIBS = ~/LIBS/opencv40/lib/
    PLATFORM = LAPTOP
endif
INCS = -I src -I /home/denis/LIBS/opencv40/include/opencv4
PLIBS = /home/denis/LIBS/opencv40/lib/
#PLATFORM = LAPTOP

LIBS = -L $(PLIBS) -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lpthread -lpigpio

all:  main 

INCDIR=src acceptImage
SRCDIR=src acceptImage
OBJDIR=obj
EXEDIR=.
vpath           %.h $(SRCDIR)
vpath           %.c $(SRCDIR)
vpath           %.cc $(SRCDIR)
vpath           %.cpp $(SRCDIR)

.c.o:
	$(CC) $(CFLAGS)  ${INCS} -c $<  
.cc.o:
	$(CC) $(CFLAGS)  ${INCS} -c $<   
.cpp.o:
	$(CC) $(CFLAGS)  ${INCS} -c $<  

MAIN_OBJ = main.o tcp_thread.o opencvPnP.o server.o videoserver.o pid.o
main:   ${MAIN_OBJ} src/servoController.h
	${CC} ${CFLAGS} ${INCS} -o $@.exe ${MAIN_OBJ}  ${LIBS}
	@echo "export LD_LIBRARY_PATH=$(PLIBS)" > env.sh
	@echo "stty -F /dev/ttyUSB0 115200" >> env.sh
	@echo "stty -F /dev/ttyUSB0 -hupcl" >> env.sh
	@echo "stty -F /dev/ttyUSB1 115200" >> env.sh

clnt:   clnt.o
	${CC} ${CFLAGS} -o $@.exe clnt.o 

client:   client.o
	${CC} ${CFLAGS} ${INCS} -o $@.exe client.o  ${LIBS}


run:
	@echo " run  program " 
	LD_LIBRARY_PATH=$(PLIBS) ./main.exe


clean:
	-rm -f *.o core *.core *.exe
help:
	@echo "   usage: make ;  make run"
test:
	@echo " BUILD for $(PLATFORM) : $(shell uname -m) CFLAGS=$(CFLAGS)"
	@echo " INCS=$(INCS) "
	@echo " LIBS=$(LIBS) "
