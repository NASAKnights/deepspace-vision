#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <unistd.h>
#include <pthread.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/time.h>
#include <pigpio.h>
typedef struct {
  double x;
  double y;
  double z;
  double dist;
  double angle;
  double angle2;
  double OffSetx;
  double speed;
  double turn;
  double gyro;
  double P;
  double I;
  double D;
}Position;


class Targets{
public:
  Targets(){NullTargets();};
  int status;
  std::string found;
  cv::Point center;
  double height;
  double width;
  double area;
  char LorR;
  int number;
  std::string reason;
  double angle;
  double ratio;
  std::vector<cv::Point> corners;
  int offby;
  cv::RotatedRect rect;
  cv::Rect boundingRect;
  cv::Point2d points[4];
  void NullTargets()
  {
    status = 0;
    found = "";
    center.x = 0;
    center.y = 0;
    height = 0;
    width = 0;
    area = 0;
    LorR = ' ';
    number = 0;
    reason = "";
    angle = 0;
    ratio = 0;
    offby = 0;
    for(int i=0;i<4;i++){
      points[i].x=0;
      points[i].y=0;
    }
  }
};
