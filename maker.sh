#!/bin/bash

link_frcLib=/home/denis/FRC/opencv/build/lib/
link_rpiLib=/usr/local/lib/
linkComputerLib=/home/denis/LIBS/lib64/

runLib=$link_rpiLib
run_frcLib=$link_frcLib

FILE=${1-main}
NAME=${2-frc}
COMM=${3-run}

if [[ ${FILE} == help ]]
then
    echo "Usage: $0 <exe file> <frc/rpi> <run/link>"
    exit 1
fi


export LD_LIBRARY_PATH=


if [[ ${COMM} == run ]]; then
    if [[ ${NAME} == frc ]]
    then
	export LD_LIBRARY_PATH=$run_frcLib
    elif [[ ${NAME} == rpi ]]
    then
	export LD_LIBRARY_PATH=$runLib
    else
	echo "error, wrong lib..."
	exit 1
    fi
    ./${FILE}.exe
    
elif [[ ${COMM} == link ]]
then
    if [[ ${NAME} == frc ]]
    then
	export LD_LIBRARY_PATH=$link_frcLib
	g++ -std=c++11 -I. -I/home/denis/LIBS/include/opencv4/ -L/home/denis/FRC/opencv/build/lib/ -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lpthread  src/main.cpp src/tcplib.c src/server.cpp  -o ${FILE}.exe -g
    elif [[ ${NAME} == rpi ]]
    then
	export LD_LIBRARY_PATH=$link_rpiLib
	g++ -std=c++11 -I. -I/usr/local/include/opencv4/ -L /usr/local/lib/ -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lpthread  src/main.cpp src/tcplib.c src/server.cpp -o ${FILE}.exe -g
    elif [[ ${NAME} == comp ]]
    then
	export LD_LIBRARY_PATH=$linkComputerLib
	g++ -std=c++11 -I. -I/home/denis/LIBS/include/opencv4 -L
/home/denis/LIBS/lib64 -lopencv_core -lopencv_ml -lopencv_calib3d
-lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc
-lpthread  src/main.cpp src/tcplib.c src/server.cpp  -o ${FILE}.exe -g
    elif [[ ${NAME} == stream ]]
    then
	g++ -I. `pkg-config --cflags opencv` src/stream.cpp src/tcplib.c src/server.cpp  -o ${NAME}.exe `pkg-config --libs opencv` -lpthread
    else
	echo "error, wrong lib..."
	exit 1
    fi
    
fi

    















    
