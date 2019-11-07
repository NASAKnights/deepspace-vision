#!/bin/bash

RPI=true;


#link_frcLib=/home/denis/FRC/opencv/build/lib/
link_rpiLib=/home/denis/LIBS/opencv40/lib/
link_compLib=/home/denis/LIBS/lib64/

#run_frcLib=$link_frcLib
run_rpiLib=$link_rpiLib
run_compLib=$link_compLib


FILE=main

COMM=${1-auto}
PAR1=${2}
PAR2=${3}
PAR3=${4}
PARA=""
if [[ ${COMM} == -h || ${COMM} == --help || ${COMM} == -help ]]
then
    echo ""
    echo -e "Usage  : $0 \n-l compiles (must be alone) \n-c color\n-t trackbars\n-s server\n-o origial\n-h HSV\n-b thresholded"
    echo ""
    exit 1
fi


if [[ ${COMM} != -l && ${COMM} != auto ]]
then
    PARA=${COMM}
    COMM=auto
fi
    

export LD_LIBRARY_PATH=


if [[ ${COMM} == auto ]]; then


    
    if [[ "$RPI" == true ]]
    then
	export LD_LIBRARY_PATH=$run_rpiLib


	
    elif [[ "$RPI" == false ]]
    then
	export LD_LIBRARY_PATH=$run_compLib
    fi
    
    ./${FILE}.exe ${PARA} ${PAR1} ${PAR2} ${PAR3}
    


    #-------------------------------------------------
elif [[ ${COMM} == -l ]]
then
    if [[ -f "${FILE}.exe" ]]
    then
	rm ${FILE}.exe
    fi


    
    if [[ "$RPI" == true ]]
    then
	export LD_LIBRARY_PATH=$link_rpiLib
	g++ -std=c++11 -I. -I/home/denis/LIBS/opencv40/include/opencv4 -L /home/denis/LIBS/opencv40/lib/ -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lpthread  src/main.cpp  src/tcp_thread.cc src/server.cpp src/videoserver.cpp -o ${FILE}.exe -g


	
	
    elif [[ "$RPI" == false ]]
    then
	export LD_LIBRARY_PATH=$link_compLib
	g++ -std=c++11 -I. -I/home/denis/LIBS/include/opencv4 -L /home/denis/LIBS/lib64 -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lpthread src/main.cpp  src/server.cpp src/videoserver.cpp  -o ${FILE}.exe -g



	
    fi
fi
