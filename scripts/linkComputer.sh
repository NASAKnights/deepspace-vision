export LD_LIBRARY_PATH=/home/denis/LIBS/lib64
g++ -std=c++11 -I. -I/home/denis/LIBS/include/opencv4 -L /home/denis/LIBS/lib64 \
-lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs \
-lopencv_highgui  -lopencv_imgproc -lpthread  main.cpp tcplib.c server.cpp  -o main.exe -g

