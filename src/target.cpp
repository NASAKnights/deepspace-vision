#include "main.h"
#include "pid.h"
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <fcntl.h>
 
using namespace cv;
/*
  removed old math
  check if ontrackbar() is needed, or can be replaced by NULL
  check and remove clnt.c from /src/
  check and maybe shorten / remove tcp_thread.*, tcplib.*, videoserver.cpp, and server.cpp
*/



#define MAXTARGETS 50
#define ASIZE 10

bool firstTimeTest = true;
int printTime = 0;
double minMotor = 0;

//Editable
double MinRatio = 0.1;
double MaxRatio = 0.65;
int MaxDiff = 1000;
//colors
int minH = 0;//81
int maxH = 255;
int minS = 10;//41
int maxS = 255; // 5 without filter
int minV = 219;//219
int maxV = 241;
//booleans
struct Switches {
  bool SHOWORIG;
  bool SHOWHUE;
  bool SHOWTRESH;
  bool SHOWTRACK;
  bool USESERVER;
  bool USECOLOR;
  bool DOPRINT;
  //bool SERVO;
  //bool TURNCAM;
};
int addr = 0x04;
int file_i2c;
int length; 
char *filename = (char*)"/dev/i2c-1";

//frame counter
int counter = 0, counter2=0, counter_old=0;
int missFR = 0;
struct timeval t1, t2;
struct timeval timeTest, lostAtTime;
struct timeval timenow, timestart;
int qdebug = 0;
//frame
bool newFrame = false;
double FrameWidth;
double FrameHeight;
//threads
pthread_t MJPEG;
void* VideoCap(void* arg);

pthread_t tcpserver;
void* opentcp(void* arg);

//pthread_t moveServoThread;
//void* moveServo(void* arg);

pthread_t videoServerThread;
void* videoServer(void* arg);

pthread_t USBSlaveThread;
void* USBSlave(void* arg);

pthread_t PIDThread;
void* movePID(void* arg);

pthread_t DriveThread;
void* drive(void* arg);

pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t positionMutex = PTHREAD_MUTEX_INITIALIZER;
Mat frame;

const std::string trackbarWindowName = "Trackbars";
std::vector <Point>totalfound;
const Scalar BLUE = Scalar(255, 0, 0), RED = Scalar(0,0,255), YELLOW = Scalar(0,255,255), GREEN = Scalar(0,255,0);

int remoteSocket = 0;
int videoPort;
int videoError = 0;
double gyroAngle = 0;
double driveAngle = 0;
double alphaGlobal = 0;
bool updated;
int buttonPress;


//Targets *tLeft;
//Targets *tRight;
Targets target;
void findAnglePnP(cv::Mat im, Targets target, Position *position);

double P, I, D;
double PIDInit[] = {-1,-1,-1};
double turn;
bool move;

bool funcX (cv::Point2f p1, cv::Point2f p2){
  return (p1.x<p2.x);
}
bool funcY (cv::Point2f p1, cv::Point2f p2){
  return (p1.y<p2.y);
}


class Clock{
private:
    struct timeval time1, time2;
public:
  Clock(){
    gettimeofday(&time1,NULL);
  }
  void restart(){
    gettimeofday(&time1,NULL);
  }
  double getTimeAsMillis(){
    gettimeofday(&time2,NULL);
    return (time2.tv_usec-time1.tv_usec+1000000 * (time2.tv_sec - time1.tv_sec))*0.001;
  }
  double getTimeAsMicros(){
    gettimeofday(&time2,NULL);
    return (time2.tv_usec-time1.tv_usec + 1000000 * (time2.tv_sec - time1.tv_sec)) * 0.000001;
  }

};


//FUNCTIONS----------------------------------------------------------------------
void on_trackbar(int, void*)
{
}

//--------------------------------------------------------------------------------

void morphOps(Mat &thresh)
{
  Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3,3));
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
  //dilate(thresh,thresh,dilateElement);
  //dilate(thresh,thresh,dilateElement);
}

//--------------------------------------------------------------------------------

void createTrackbars()
{
  namedWindow(trackbarWindowName, 0);
  /*
  char TrackbarName[50];
  sprintf( TrackbarName, "H_MIN", minH);
  sprintf( TrackbarName, "H_MAX", maxH);
  sprintf( TrackbarName, "S_MIN", minS);
  sprintf( TrackbarName, "S_MAX", maxS);
  sprintf( TrackbarName, "V_MIN", minV);
  sprintf( TrackbarName, "V_MAX", maxV);
  */
  createTrackbar( "H_MIN", trackbarWindowName, &minH, 255, on_trackbar );
  createTrackbar( "H_MAX", trackbarWindowName, &maxH, 255, on_trackbar );
  createTrackbar( "S_MIN", trackbarWindowName, &minS, 255, on_trackbar );
  createTrackbar( "S_MAX", trackbarWindowName, &maxS, 255, on_trackbar );
  createTrackbar( "V_MIN", trackbarWindowName, &minV, 255, on_trackbar );
  createTrackbar( "V_MAX", trackbarWindowName, &maxV, 255, on_trackbar );
}

//--------------------------------------------------------------------------------

int findTarget(Mat original, Mat thresholded, Targets *targets){
  bool printTimeTarget = false;
  if(printTime==2)
    printTimeTarget=true;
  Clock clock;
  Clock between;
  
  double qualityLevel = 0.05;
  double minDistance = 50;
  int blockSize = 3;
  bool useHarisDetector = true;
  double k = 0.04;//0.04
  int maxCorners = 4;

  /*
  double qualityLevel = 0.05;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarisDetector = true;
  double k = 0.04;//0.04
  int maxCorners = 100;
  */
  if(printTimeTarget){
    clock.restart();
    between.restart();
    printf("begin: \n");
  }
  
  std::vector<Vec4i> hierarchy;
  std::vector<std::vector<Point> > contours;
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  std::vector<cv::Point2f> corners;
  if(printTimeTarget){
    printf(" init         %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }
  //erode(thresholded,thresholded,erodeElement);
  if(printTimeTarget){
    printf(" eroding      %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }
  //dilate(thresholded,thresholded,dilateElement);
  if(printTimeTarget){
    printf(" dialating    %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }
  //GaussianBlur(thresholded,thresholded,Size(3,3),0);
  if(printTimeTarget){
    printf(" gaus blur    %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }
  //findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  //findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  findContours(thresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  if(printTimeTarget){
    printf(" findContours %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }
  //------------- preselect by perimeter ------------------
  
  for (std::vector<std::vector<Point> >::iterator it = contours.begin(); it != contours.end();){
    if (it->size() < 100){//min contour
      it = contours.erase(it);
      if (qdebug>4)
	std::cout << "erased perim" << std::endl;
    } else
      ++it;
  }
  

  //-------------- select by number of contours ------------

  int ntargets=0;
  
  if (contours.size() > MAXTARGETS ) {
    //if (qdebug>2)
      std::cout <<  " too many targets found = " << contours.size() << std::endl;
    return -1;
  } else if (1 > contours.size() ){
    //if (qdebug>2)
      std::cout << "too little targets found" << std::endl;
    return -1;
  }
  
  //--------------------------------------------------------
  if(printTimeTarget){
    printf(" fltr prm&num %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }
  std::vector<RotatedRect> minRect(contours.size());
  std::vector<std::vector<Point>> hull(contours.size());
  std::vector<std::vector<Point>> art;
  //std::vector<std::vector<Point>> squeezedArt;
  std::vector<Point> approx;
  //Mat workingImage(480,640,CV_8UC1,Scalar(0));
  Mat workingImage(FrameHeight,FrameWidth,CV_8UC1,Scalar(0));
  Mat  workingImageSq;
  int num = -1;
  if(printTimeTarget){
    printf(" init#2       %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }
  if (!contours.empty() && !hierarchy.empty()) {

    for (int i = 0; i < (int) contours.size(); i++){
      if(printTimeTarget){
	printf("i: %d\n",i);
      }
      targets[i].NullTargets();
      if (hierarchy[i][2]!=-1) {if (qdebug>2) printf("failed hierarchy\n"); continue;}
      minRect[i] = minAreaRect(Mat(contours[i]));
      cv::Point2f rect_points[4];
      minRect[i].points(rect_points);
      std::copy(rect_points,rect_points+4,targets[i].points);
      targets[i].rect = minRect[i];
      targets[i].boundingRect = minRect[i].boundingRect();
      if(printTimeTarget){
	printf(" findRct      %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }
      bool flag= false;
      int bounding = 20;
      for(int k=0;k<4;k++){
	//if(abs(rect_points[k].x-320) > (320-bounding) || abs(rect_points[k].y-240) > (240-bounding))
	if(abs(rect_points[k].x-FrameWidth/2) > (FrameWidth/2-bounding) || abs(rect_points[k].y-FrameHeight/2) > (FrameHeight/2-bounding))
	  flag = true;
      }
      if(flag){
	if(printTimeTarget)printf("  SKIP: edge too close\n");
	continue;
      }
      
      for(int j=0;j<4;j++){
	line(original,rect_points[j],rect_points[(j+1)%4],BLUE,1,8);
	circle(original, rect_points[j],3,RED,-1,8,0);
      }
      if(printTimeTarget){
	printf(" drawRct      %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }
      convexHull(contours[i],hull[i]);
      if(printTimeTarget){
	printf(" convexHull   %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }
      double ratioTest = contourArea(hull[i])/contourArea(contours[i]);
      if(ratioTest < 4){
	  if(printTimeTarget) printf("  SKIP: Area-Ratio: %.2f\n",ratioTest);
	continue;
      }
      if(printTimeTarget){
	printf(" ratio test   %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }
      approxPolyDP(hull[i],approx,arcLength(hull[i],true)*0.015,true);//0.015
      if(printTimeTarget){
	printf(" after poly   %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }
      art.push_back(approx);
      
      //Mat(targets[i].boundingRect).copyTo(workingImage);
      //drawContours(original,art,i,GREEN,4);
      //squeezedArt
      num = i;
      ntargets++;
    }//---end contour loop i
    if(printTimeTarget) printf("end:\n");
    for(unsigned int j=0;j<art.size();j++){
      drawContours(workingImage,art,j,Scalar(255));
      drawContours(original,art,j,GREEN,4);
    }
    if(num!=-1)
      workingImage(targets[num].boundingRect).copyTo(workingImageSq);
    if(printTimeTarget){
      printf(" drw on Mat   %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
      between.restart();
    }
    if(ntargets==1){
      cv::goodFeaturesToTrack(workingImageSq,corners,maxCorners,qualityLevel,minDistance,Mat(),blockSize,useHarisDetector,k);
      for(unsigned int i=0;i<corners.size();i++){
	corners[i].x=corners[i].x+targets[num].boundingRect.x;
	corners[i].y=corners[i].y+targets[num].boundingRect.y;
      }
      if(printTimeTarget){ 
	printf(" goodFeatures %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }
      //cv::cornerHarris(workingImage,corners,); // test this out instead?
      target.corners.clear();
      target.corners.push_back(corners[0]);
      target.corners.push_back(corners[1]);
      target.corners.push_back(corners[2]);
      target.corners.push_back(corners[3]);
    }
  }
  /*
  if(ntargets==2){
    line(original,tLeft->center, tRight->center, YELLOW, 1);
    line(original,tLeft->center, tLeft->center, RED, 3);
    line(original,tRight->center, tRight->center, RED, 3);
  }
  */
  if(printTimeTarget)
    printf("--finalTime: %.2f\n==---------------------------==\n",clock.getTimeAsMillis());
  return ntargets;
}

//--------------------------------------------------------------------------------

Mat ThresholdImage(Mat original)
{
  Mat thresholded;
  inRange(original, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), thresholded);
  return thresholded;
}

//-Threads-----------------------------------------------------------------------


void* drive(void* arg){
  Position* pos = (Position*) arg;
  bool init = true;
  Clock clock;
  double angle2Local;
  int counter12 = 0;
  while(true){
    if(buttonPress == 0)
      init = true;
    if(buttonPress == 1 && init){ //initialize loop when first button pressed;
      init = false;
      printf("button press: INIT!\n");
    }
    if(updated){
      pthread_mutex_lock(&targetMutex);
      angle2Local = pos->angle2;
      pthread_mutex_unlock(&targetMutex);
    }
    if(buttonPress == 1){
      pos->speed = 0.0;
      if(turn>=0)
	pos->turn = std::min(sqrt(abs(turn/1.5)),0.7);
      else
	pos->turn = std::max(-sqrt(abs(turn/1.5)),-0.7);
      
      //pos->turn = turn;
    }else{
      pos->speed = 0.0;
      pos->turn = 0.0;
    }
    if(clock.getTimeAsMillis()>100){
      printf("turn: %5.2f angle: %6.2f ; P %5.2f, I %5.2f, D %5.2f                     gyro: %7.2f\n",pos->turn,pos->angle,P,I,D,gyroAngle);
      clock.restart();
    }
    if(updated){
      driveAngle = alphaGlobal+(-gyroAngle); // calculate the needed position for the turn
      updated = false;
    }
    move = (pos->dist > 75) ? true : false;
    usleep(50);
  }
}

void* movePID(void* arg){
  PID* drivePID;
  Clock clock;
  double turnLoc, dt;
  if(PIDInit[0]==-1)
    PIDInit[0]=0.008;
  if(PIDInit[1]==-1)
    PIDInit[1]=0.0;
  if(PIDInit[2]==-1)
    PIDInit[2]=0.002;
  
  drivePID = new PID(0.0,1,-1, PIDInit[0],PIDInit[1],PIDInit[2]); // 0.01,0.005,0.0
  
  while(true){
    dt = clock.getTimeAsMicros();
    clock.restart();
    //printf("dt: %f\n",dt);
    drivePID->button(buttonPress);
    //turnLoc = 0;
    turnLoc = drivePID->calculate(driveAngle,-gyroAngle,dt,&P,&I,&D);
    turn = turnLoc;
    usleep(10*1000);
  }
}

void* VideoCap(void *args){
  cv::VideoCapture vcap;
  
  while (!vcap.open(0)){
    std::cout << "cant connect" << std::endl;
    usleep(10000000);
  }
  printf("setting brightness\n");
  vcap.set(cv::CAP_PROP_BRIGHTNESS,100);
  printf("setting auto exposure\n");
  vcap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);//255
  printf("setting exposure\n");
  vcap.set(cv::CAP_PROP_EXPOSURE,85);
  
  vcap.set(cv::CAP_PROP_FRAME_WIDTH,640);//1920//1280
  vcap.set(cv::CAP_PROP_FRAME_HEIGHT,480);//1080//720

  FrameWidth = vcap.get(cv::CAP_PROP_FRAME_WIDTH);
  FrameHeight = vcap.get(cv::CAP_PROP_FRAME_HEIGHT);
  while (true){
    pthread_mutex_lock(&frameMutex);
    vcap.read(frame);
    pthread_mutex_unlock(&frameMutex);
    newFrame = true;
    usleep(33000);//33000
  }
}
//-----------------------------------------------------------------------------
inline void nullifyStruct(Position &pos){
  pos.x=0;
  pos.z=0;
  pos.dist=0;
  pos.angle=0;
  pos.angle2=0;
  pos.OffSetx=0;
  pos.speed=0;
  pos.turn=0;
  pos.gyro=0;
  pos.P=0;
  pos.I=0;
  pos.D=0;
}
void* USBSlave(void* arg){
  printf("enter gyro slave\n");
  int ttyFid = open("/dev/ttyUSB0", O_RDWR);
  if (ttyFid == -1){
    printf( "Error unable to open port\n");
  }
  printf("enter readBus\n");
  char line[256];
  while(true){
    for(int ii=0;ii<200;ii++){
      int nb = read(ttyFid,&line[ii],1);
      if(nb!=1)
	printf("nb=%d\n",nb);
      if(nb<0){sleep(1); ii=11; continue;}
      if(line[ii]==',') { line[ii]=' ';}
      if(line[ii]=='\n') {line[ii+1]=0; break;}
    }
    //printf("line=%s\n",line);
    float roll, pitch, yaw;
    float ACCX,ACCY,GYROZ,AAZ;
    //sscanf(line,"%f,%f,%f",&roll,&pitch,&yaw);
    sscanf(line,"%f %f %f %f %f %f %f",&ACCX,&ACCY,&roll,&pitch,&yaw,&GYROZ,&AAZ);
    if(GYROZ > 100)
      printf("yaw: %.2f; GYROZ: %.2f\n",yaw,GYROZ);
    gyroAngle=yaw;
  }
}

int main(int argc, const char* argv[]){
  Clock clock, between;
  bool printTimeMain = false;
  //double deltaGyro, prevGyro = 0;
  Switches switches;
  switches.SHOWORIG = false;
  switches.SHOWHUE = false;
  switches.SHOWTRESH = false;
  switches.SHOWTRACK = false;
  switches.USESERVER = false;
  switches.USECOLOR = false;
  switches.DOPRINT = false;
  if(argc>=2){
    std::string args = argv[1];
    std::vector<char> token;
    if(args.compare("--help")==0){
      printf("-c color\n");
      printf("-t Trackbars\n");
      printf("-s Server on\n");
      printf("-o Original\n");
      printf("-h HSV\n");
      printf("-b black and white\n");
      printf("-p print stuff\n");
      printf("param 2:\n");
      printf("qdebug=\n");
      printf("ptime=\n");
      printf("  1=main loop\n  2=findTarget loop\n");
      return 0;
    }
    for(int i=1;i<(int)args.size();i++)
      token.push_back(args.at(i));
    for(int i=0;i<(int)token.size();i++){
      switch(token[i]){
      case 'c':
	switches.USECOLOR = true; break;
      case 't':
	switches.SHOWTRACK = true; break;
      case 's':
	switches.USESERVER = true; break;
      case 'o':
	switches.SHOWORIG = true; break;
      case 'h':
	switches.SHOWHUE = true; break;
      case 'b':
	switches.SHOWTRESH = true; break;
      case 'p':
	switches.DOPRINT = true; break;
      }  
    }
    if(argc>=3){
      for(int a=3;a<=argc;a++){
	std::string arg2(argv[a-1]);
	if(arg2.substr(0,7).compare("qdebug=")==0 && arg2.size()>=8){
	  qdebug = std::stoi(arg2.substr(7));
	  printf("set qdebug to %d\n",qdebug);
	}
	else if(arg2.substr(0,6).compare("ptime=")==0 && arg2.size()>=7){
	  printTime = std::stoi(arg2.substr(6));
	  printf("set printTime to %d\n",printTime);
	}
	else if(arg2.substr(0,2).compare("P=")==0 && arg2.size()>=3){
	  PIDInit[0] = std::stof(arg2.substr(2));
	  printf("set P to %f\n",PIDInit[0]);
	}
	else if(arg2.substr(0,2).compare("I=")==0 && arg2.size()>=3){
	  PIDInit[1] = std::stof(arg2.substr(2));
	  printf("set I to %f\n",PIDInit[1]);
	}
	else if(arg2.substr(0,2).compare("D=")==0 && arg2.size()>=3){
	  PIDInit[2] = std::stof(arg2.substr(2));
	  printf("set D to %f\n",PIDInit[2]);
	}
	else if(arg2.substr(0,5).compare("mMin=")==0 && arg2.size()>=6){
	  minMotor = std::stof(arg2.substr(6));
	  printf("set minMotor to %f\n",minMotor);
	}
      }
    }
  }
  if(printTime==1)
    printTimeMain=true;

  
  if(printTimeMain){
    printf(" getting input %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }
  
  if ((file_i2c = open(filename, O_RDWR)) < 0){
    printf("Failed to open the i2c bus");
    return false;
  }
  if (ioctl(file_i2c, I2C_SLAVE, addr) < 0){
    printf("Failed to acquire bus access and/or talk to slave.\n");
    return false;
  }

  gettimeofday(&lostAtTime,NULL);
  
  Mat img, HSV, thresholded, output;
  gyroAngle = 0;
  driveAngle = 0;
  gettimeofday(&t1, NULL);
  gettimeofday(&timeTest,NULL);
  videoPort=4097;
  Position position, positionAV;
  std::vector<Position>::iterator it;
  std::vector<Position> posA;
  Targets targets[MAXTARGETS];
  pthread_create(&tcpserver, NULL, opentcp, &positionAV);
  pthread_create(&MJPEG, NULL, VideoCap, NULL);
  pthread_create(&USBSlaveThread, NULL, USBSlave,NULL);
  pthread_create(&PIDThread, NULL, movePID, NULL);
  pthread_create(&DriveThread, NULL, drive, &positionAV);
  int rc = pthread_setname_np(MJPEG, "MJPEG Thread");
  if (rc != 0)
    printf("MJPEG thread fail%d\n",rc);
  rc = pthread_setname_np(tcpserver, "tcpserver");
  if (rc != 0)
    printf("tcp thread fail%d\n",rc);
  rc = pthread_setname_np(USBSlaveThread,"GyroThread");
  if (rc != 0)
    printf("gyro thread fail%d\n",rc);
  
  if(switches.USESERVER) {
    pthread_create(&videoServerThread, NULL, videoServer, NULL);
    rc = pthread_setname_np(videoServerThread,"VideoServerThread");
    if (rc != 0)
      printf("video thread fail%d\n",rc);
  }
  
  
  if(switches.SHOWTRACK) createTrackbars();
  if(!img.isContinuous()) img = img.clone();
  //set all values to 0 before program begins in loop forever.
  nullifyStruct(position);
  nullifyStruct(positionAV);
  if(printTimeMain){
    printf(" initThreads %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
    between.restart();
  }

  while (true){
    clock.restart();
    between.restart();
    pthread_mutex_lock(&frameMutex);
    if(!frame.empty() && newFrame){ //check if new frame is available
      frame.copyTo(img);
      if(printTimeMain){
	printf(" get frame   %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }
      
      pthread_mutex_unlock(&frameMutex);
      cv::cvtColor(img, HSV, CV_BGR2HSV);
      if(printTimeMain){
	printf(" to HSV      %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }

      thresholded = ThresholdImage(HSV);
      if(printTimeMain){
	printf(" thresh      %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }

      morphOps(thresholded);
      if(printTimeMain){
	printf(" Morph       %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }

      pthread_mutex_lock(&targetMutex);
      int nt = findTarget(img, thresholded, targets);
      if(printTimeMain){
	printf(" findTarget  %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }

      nullifyStruct(position);
      //printf("checking %d==1?\n",nt);
      if (nt==1){
	//printf("updated %.2f\n",clock.getTimeAsMillis());
	findAnglePnP(img,target,&position);
	if(printTimeMain){
	  printf(" solvePnP    %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	  between.restart();
	}

	//put latest values into avaraging struct, delete old one.
	
	posA.push_back(position);
	if(posA.size()>ASIZE)
	  posA.erase(posA.begin());
	//MUTEX HERE
	//resetting avarage struct
	nullifyStruct(positionAV);
	int cntr = 0;
	for(it = posA.end()-3; it != posA.end(); it++){
	  cntr++;
	  //std::cout<< i << ": x = " << (*it).x << std::endl;
	  positionAV.x+=(*it).x;
	  positionAV.z+=(*it).z;
	  positionAV.angle+=(*it).angle;
	  //positionAV.angle2+=(*it).angle2;
	  positionAV.dist+=(*it).dist;
	  positionAV.OffSetx+=(*it).OffSetx;	      
	}
	for(it = posA.begin(); it != posA.end(); it++){
	  positionAV.angle2+=(*it).angle2;
	}
	positionAV.x/=cntr;
	positionAV.z/=cntr;
	positionAV.angle/=cntr;
	positionAV.angle2/=posA.size();
	positionAV.dist/=cntr;
	positionAV.OffSetx/=cntr;
	positionAV.gyro=gyroAngle;
	if(printTimeMain){
	  printf(" avaraging   %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	  between.restart();
	}
	

	//====UPDATES====
	//setting global alpha Value
	alphaGlobal = positionAV.angle;

	// set PID args to send to server;
	positionAV.P=P;
	positionAV.I=I;
	positionAV.D=D;
	
	//all values and fr has been updated;
	updated = true;

	
	if(qdebug > 3){
	  std::cout << "" << std::endl;
	  std::cout << "/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*" << std::endl;
	}
	if(printTimeMain){
	  printf(" settingGlob %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	  between.restart();
	}

      }else{
	//nullifyStruct(positionAV);
	positionAV.z=-1;
	missFR++;


      }
      //f(firstTimeTest){
      between.restart();
      if(switches.SHOWORIG)
	imshow("Original", img);
      if(switches.SHOWTRESH)
	imshow("Thresholded", thresholded);
      if(switches.SHOWHUE)
	imshow("HSV" , HSV);
      if(printTimeMain){
	printf(" imShow      %.2f tot: %.2f\n",between.getTimeAsMillis(),clock.getTimeAsMillis());
	between.restart();
      }
      
	/*
	firstTimeTest = false;
      } else {
	if(switches.SHOWORIG)
	  updateWindow("Original");
	if(switches.SHOWTRESH)
	  updateWindow("Thresholded");
	if(switches.SHOWHUE)
	  updateWindow("HSV");
      }
	*/

      
      if(switches.DOPRINT)
	printf("x=%6.2f, z=%6.2f, dist=%6.2f, angle=%6.2f, angle2=%6.2f, speed=%4.2f, turn=%5.2f, gyro=%7.2f\n",position.x,position.z,position.dist,position.angle,position.angle2,position.speed,position.turn,position.gyro);
      pthread_mutex_unlock(&targetMutex);
      totalfound.clear();
      counter++;
      if(printTimeMain){
	printf("--finalTime: %.2f\n",clock.getTimeAsMillis());
      }
    }//end of check for new frame
    else
      pthread_mutex_unlock(&frameMutex);

    
    counter2++;
    if(counter%10==0 && counter != counter_old){
      counter_old = counter;
      //could be replaced by Clock
      gettimeofday(&t2,NULL);
      double dt = t2.tv_usec-t1.tv_usec+1000000 * (t2.tv_sec - t1.tv_sec);
      t1=t2;
      
      //printf("------ Frame rate: %f fr/s (%f) \n",10./dt*1e6,counter2/dt*1e6); counter2=0;
      //printf("------ Miss Frame: %d fr \n",missFR);
      missFR=0;
      if(switches.USESERVER && remoteSocket>0){
	int bytes = 0;
	if(switches.USECOLOR && remoteSocket>0){
	  int imgSize = img.total() * img.elemSize();
	  if (!img.isContinuous()) {
	    img = img.clone();
	  }
	  if((bytes = send(remoteSocket,img.data,imgSize, 0)) < 0){
	    printf("error");
	    videoError=1;
	  }
	}else{	    
	  int imgSize = thresholded.total() * thresholded.elemSize();
	  if (!thresholded.isContinuous()) {
	    thresholded = thresholded.clone();
	  }
	  if((bytes = send(remoteSocket,thresholded.data,imgSize,0)) <0){
	    printf("error");
	    videoError=1;
	  }
	}
      }
    }

    
    if(switches.SHOWORIG || switches.SHOWHUE || switches.SHOWTRESH || switches.SHOWTRACK)
      waitKey(5);
    newFrame = false;
    usleep(100);
  }

  pthread_join(MJPEG, NULL);

  return 0;
  
}
