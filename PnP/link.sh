#INCS=/home/denis/LIBS/opencv40/include/opencv4
INCS=/home/denis/LIBS/include/opencv4

#LIBS=/home/denis/LIBS/opencv40/lib/
LIBS=/home/denis/LIBS/lib64

g++ -std=c++11 -I$INCS -L $LIBS -lopencv_core -lopencv_ml -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui  -lopencv_imgproc -lopencv_core opencvPnP.cpp -o PnP.exe -g
