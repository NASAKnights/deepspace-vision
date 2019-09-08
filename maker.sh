#!/bin/bash

#link_frcLib=/home/denis/FRC/opencv/build/lib/
link_rpiLib=/usr/local/lib/
link_compLib=/home/denis/LIBS/lib64/

#run_frcLib=$link_frcLib
run_rpiLib=$link_rpiLib
run_compLib=$link_compLib


FILE=main

NAME=${1-rpi}
COMM=${2-link}

if [[ ${NAME} == help ]]
then
    echo ""
    echo "Usage  : $0 <frc/rpi/comp> <run>"
    echo "Deafult: $0 <rpi> <link>"
    echo ""
    exit 1
fi


export LD_LIBRARY_PATH=


if [[ ${COMM} == run ]]; then
    if [[ ${NAME} == rpi ]]
    then
	export LD_LIBRARY_PATH=$run_rpiLib
#    elif [[ ${NAME} == frc ]]
#    then
#	export LD_LIBRARY_PATH=$run_frcLib
    elif [[ ${NAME} == comp ]]
    then
	export LD_LIBRARY_PATH=$run_compLib
    else
	echo "error, wrong lib..."
	exit 1
    fi
    ./${FILE}.exe
    


#-------------------------------------------------
elif [[ ${COMM} == link ]]
then
    if [[ ${NAME} == rpi ]]
    then
	export LD_LIBRARY_PATH=$link_rpiLib
	g++ -std=c++11 -I. -I/usr/local/include/opencv4/ -L /usr/local/lib/ -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lpthread  src/main.cpp src/tcplib.c src/server.cpp -o ${FILE}.exe -g
	./${FILE}.exe
#    elif [[ ${NAME} == frc ]]
#    then
#	export LD_LIBRARY_PATH=$link_frcLib
#	g++ -std=c++11 -I. -I/home/denis/LIBS/include/opencv4/ -L/home/denis/FRC/opencv/build/lib/ -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lpthread  src/main.cpp src/tcplib.c src/server.cpp  -o ${FILE}.exe -g
#	./${FILE}.exe
    elif [[ ${NAME} == comp ]]
    then
	export LD_LIBRARY_PATH=$link_compLib
	g++ -std=c++11 -I. -I/home/denis/LIBS/include/opencv4 -L /home/denis/LIBS/lib64 -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lpthread src/main.cpp src/tcplib.c src/server.cpp  -o ${FILE}.exe -g
	./${FILE}.exe
    elif [[ ${NAME} == stream ]]
    then
	g++ -I. `pkg-config --cflags opencv` src/stream.cpp src/tcplib.c src/server.cpp  -o ${FILE}.exe `pkg-config --libs opencv` -lpthread
    else
	echo "error, wrong lib..."
	exit 1
    fi
fi
