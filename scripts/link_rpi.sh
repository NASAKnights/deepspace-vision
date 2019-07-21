export LD_LIBRARY_PATH=/usr/local/lib/
g++ -std=c++11 -I. -I/usr/local/include/opencv4/ -L /usr/local/lib/ \
-lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs \
-lopencv_highgui  -lopencv_imgproc -lpthread  main.cpp tcplib.c server.cpp \
    -o main.exe -g
