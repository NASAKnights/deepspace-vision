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
#include "main.h"
#include <include/constants_c.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/time.h>
//#include <signal.h>



using namespace cv;
using namespace std;

#define MAXTARGETS 20
#define ASIZE 10


//Editable
double MinRatio = 0.1;
double MaxRatio = 0.65;
int MaxDiff = 1000;

/*
  int minH = 133;//131
  int maxH = 255;
  int minS = 255;//128
  int maxS = 255;
  int minV = 169;//92
  int maxV = 255;
*/
int minR = 211; //127//41
int maxR = 230;//132//255
int minG = 219;//131//200
int maxG = 255;//132//255
int minB = 174; //131//0
int maxB = 231;//132//125

bool USEIPCAM = false;
bool SHOWO = false;
bool SHOWT = false; 
bool SHOWTR = false;
bool USESERVER = false;
bool USECOLOR = false;

int counter = 0;
struct timeval t1, t2;

bool SideShow = true;
bool AngleShow = true;
bool AreaShow = true;
bool RatioShow = true;
int qdebug = 9;

// char* outFile = "./out.mjpg";

//vals
double alpha;

//non-Editable
bool newFrame = false;
double FrameWidth;
double FrameHeight;
pthread_t MJPEG;
pthread_t tcpserver;
pthread_t videoServerThread;
Point centerob;
float dist;
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
const string trackbarWindowName = "Trackbars";
Mat frame;

vector <Point>totalfound;
const Scalar BLUE = Scalar(255, 0, 0), RED = Scalar(0,0,255), YELLOW = Scalar(0,255,255);

void *opentcp(void *arg);
void *videoServer(void *arg);

int remoteSocket = 0;
int videoPort;
int videoError = 0;
//#PARAMS

//Functions
/*
  void createTrackbars();
  void*VideoCap(void *args);
*/

//Classes
class Targets
{
public:
  Targets(){NullTargets();};
  ~Targets(){};
  int status;
  string found;
  Point center;
  double height;
  double width;
  double area;
  char LorR;
  int number;
  string reason;
  double angle;
  double ratio;
  int offby;
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
  }
  void Show(){
    int W = width;
    int H = height;
    cout << "Number: " << number << endl;
    cout << "\tTarget found?: " << found;
    if (found == "no"){
      cout << "--" << reason << endl;
    } else {
      cout << "" << endl;
    }
    if (RatioShow) cout << "--Ratio: " << ratio << endl;
    if (SideShow) cout << "--Width: " << W << " || Height: " << H << endl;
    if (AngleShow) cout << "--Angle: " << angle << endl;
    if (AreaShow) cout << "--Area: " << height*width << endl;
    
  }
};
Targets *tLeft;
Targets *tRight;

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
  char TrackbarName[50];
  sprintf( TrackbarName, "H_MIN", minR);
  sprintf( TrackbarName, "H_MAX", maxR);
  sprintf( TrackbarName, "S_MIN", minG);
  sprintf( TrackbarName, "S_MAX", maxG);
  sprintf( TrackbarName, "V_MIN", minB);
  sprintf( TrackbarName, "V_MAX", maxB);
  createTrackbar( "H_MIN", trackbarWindowName, &minR, 255, on_trackbar );
  createTrackbar( "H_MAX", trackbarWindowName, &maxR, 255, on_trackbar );
  createTrackbar( "S_MIN", trackbarWindowName, &minG, 255, on_trackbar );
  createTrackbar( "S_MAX", trackbarWindowName, &maxG, 255, on_trackbar );
  createTrackbar( "V_MIN", trackbarWindowName, &minB, 255, on_trackbar );
  createTrackbar( "V_MAX", trackbarWindowName, &maxB, 255, on_trackbar );
}

//--------------------------------------------------------------------------------

int findTarget(Mat original, Mat thresholded, Targets *targets)
{
  vector<Vec4i> hierarchy;
  vector<vector<Point> > contours;
  findContours(thresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  //------------- preselect by perimetr ------------------
  unsigned int contourMin = 10;
  //cout << "initial targets = " << contours.size() << endl;
  for (vector<vector<Point> >::iterator it = contours.begin(); it != contours.end();){
    if (it->size() < contourMin){
      it = contours.erase(it);
      if (qdebug==2) cout << "erased" << endl;
    } else
      ++it;
  }

  //-------------- select by number of contours ------------

  int ntargets=0;
  if (contours.size() > MAXTARGETS ) {
    if (qdebug==2) cout <<  " too many targets found = " << contours.size() << endl;
    return -1;
  } else if (2 > contours.size() ){
    if (qdebug==2) cout << "too little targets found" << endl;
    return -1;
  }
  //--------------------------------------------------------

  vector<RotatedRect> minRect(contours.size());
  Mat drawing = Mat::zeros(original.size(), CV_8UC3);

  if (!contours.empty() && !hierarchy.empty()) {

    for (int i = 0; i < contours.size(); i++){
      targets[i].NullTargets();

      //cout << " hierarchi = " << hierarchy[i][2] << endl;
      if (hierarchy[i][2]!=-1) {if (qdebug==2) printf("failed hierarchy\n"); continue;}

      minRect[i] = minAreaRect(Mat(contours[i]));
      Point2f rect_points[4];
      minRect[i].points(rect_points);
      double w1 = sqrt(pow((rect_points[0].x-rect_points[1].x),2)
		       + pow((rect_points[0].y-rect_points[1].y),2));
      double w2 = sqrt(pow((rect_points[2].x-rect_points[3].x),2)
		       + pow((rect_points[2].y-rect_points[3].y),2));
      double h1 = sqrt(pow((rect_points[1].x-rect_points[2].x),2)
		       + pow((rect_points[1].y-rect_points[2].y),2));
      double h2 = sqrt(pow((rect_points[3].x-rect_points[0].x),2)
		       + pow((rect_points[3].y-rect_points[0].y),2));

      double Width = (w1+w2)/2;
      double Height = (h1+h2)/2;
      targets[i].height = Height;
      targets[i].width = Width;
      if (w1>h1){
	targets[i].height = Width;
	targets[i].width = Height;
	Width=targets[i].width;
	Height=targets[i].height;
      }

      double area =  Width * Height;
      targets[i].area = area;

      for (int j = 0; j < 4; j++) { 
	line(original,rect_points[j], rect_points[(j + 1) % 4], BLUE, 1, 8);
      }
      Rect box = minRect[i].boundingRect();
      targets[i].found = "no";
      Point center(box.x + box.width / 2, box.y + box.height / 2);
      targets[i].center=center;
      
      double angle = minRect[i].angle;
      targets[i].angle=angle;

      double RRatio = Width / Height;
      targets[i].ratio=RRatio;
 
      //if (area<200) {if (qdebug == 0)printf("failed area\n");continue;}

      if ( MinRatio >  RRatio || RRatio > MaxRatio ){if (qdebug==1) printf("failed ratio : %f \n",RRatio); continue;}
 
      //if (abs(angle)<10) continue; 

      double ang1=15, ang2=75;
      if ( abs(abs(angle)-ang1)>20 && abs(abs(angle)-ang2)>20 ){if (qdebug==1) printf("failed angle\n"); continue;}

      targets[i].number = i;
      targets[i].status=1; // --- preselected target ----	
      int ttt = 0;
      for ( int k = 0; k < i; k++) {

	if (targets[i].status==0) {if (qdebug==1) printf("failed status\n");continue;}
	
	double xi = targets[i].center.x;
	double xk = targets[k].center.x;
	double yi = targets[i].center.y;
	double yk = targets[k].center.y;

	double dx = xi - xk;
	double dy = yi - yk;

	//cout << " i,k = " << i << " " << k <<  " dx = " << dx << " dy = " << dy << endl;
	//cout << " area  = " << targets[i].area << " " << targets[k].area << endl;
	ttt++;
	if ( abs(dy)>40 ) {if (qdebug==1) printf("failed distance: %f: %d\n",dy,ttt);continue;}
	double arr = targets[i].area/targets[k].area;
	if (arr>1) arr=1./arr;
	if (qdebug > 1)cout << " area " << targets[i].area << " " << targets[k].area << " arr= " << arr << endl;
	if (arr<0.5) {if (qdebug==1) printf("failed area ratio: %f: %d\n",arr,ttt);continue;}
	//------  end selection -------

	if (targets[i].status==1) { targets[i].status=2; targets[i].found = "yes"; ntargets++ ; }
	if (targets[k].status==1) { targets[k].status=2; targets[k].found = "yes"; ntargets++ ; }

	if (dx>0) { tLeft=&targets[k]; tRight=&targets[i]; }
	else      { tLeft=&targets[i]; tRight=&targets[k]; }


      } //---end contour loop k
    }//---end contour loop i

  }

  if(ntargets==2){
    line(original,tLeft->center, tRight->center, YELLOW, 1);
    line(original,tLeft->center, tLeft->center, RED, 3);
    line(original,tRight->center, tRight->center, RED, 3);
  }
  return ntargets;
}

//--------------------------------------------------------------------------------

Mat ThresholdImage(Mat original)
{
  Mat thresholded;
  inRange(original, Scalar(minR, minG, minB), Scalar(maxR, maxG, maxB), thresholded);
  //blur(thresholded, thresholded, Size(3,3));
  return thresholded;
}

//--------------------------------------------------------------------------------

void *VideoCap(void *args)
{
  cv::VideoCapture vcap;
  
  if(USEIPCAM){
    //const char* webcam="http://localhost:1181/?action=stream";
    const char* webcam="http://127.0.0.1:1181/?action=stream";
    //const char* webcam="http://10.1.22.20:1181/";
    cout << "use webcam at " << webcam << endl;	
    while (!vcap.open(webcam)){
      cout << "cant connect to " << webcam << endl;
      usleep(10000000);
    }
    cout << "webcam:onnected to " << webcam << endl;
  }else{
    while (!vcap.open(0)){
      cout << "cant connect" << endl;
      usleep(10000000);
    }
  }
  
  vcap.set(CV_CAP_PROP_BRIGHTNESS, 100);
  //vcap.set(CV_CAP_PROP_CONTRAST, 100);
  vcap.set(CV_CAP_PROP_AUTO_EXPOSURE, 255);
  //vcap.set(CV_CAP_PROP_EXPOSURE, 255);
  //vcap.set(CV_CAP_PROP_SATURATION, 100);
  //vcap.set(CV_CAP_PROP_GAIN,0);
  FrameWidth = vcap.get(CV_CAP_PROP_FRAME_WIDTH);
  FrameHeight = vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
  
  
  cout << "success!" << endl;
  
  while (true){
    pthread_mutex_lock(&frameMutex);
    vcap.read(frame);
    pthread_mutex_unlock(&frameMutex);
    newFrame = true;
    usleep(33000);
  }
}

//--------------------------------------------------------------------------------

void calcTarget()
{
  if (qdebug > 1){
    if (tLeft->area > tRight->area)
      cout << "turn left" << endl;
    else 
      cout << "turn right" << endl;
    if ((tLeft->center.x > FrameWidth/2-160 && tLeft->center.x < FrameWidth/2) && (tRight->center.x < FrameWidth/2+160 && tRight->center.x > FrameWidth/2))
      cout << "stay on track" << endl;
    else if (tLeft->center.x < FrameWidth/2)
      cout << "move left" << endl;
    else if (tRight->center.x > FrameWidth/2)
      cout << "move right" << endl;
  }
  centerob.x = (tRight->center.x-tLeft->center.x)/2+tLeft->center.x;
  centerob.y = (tRight->center.y-tLeft->center.y)/2+tLeft->center.y;
  
  int testq;
  
}


//-----------------------------------------------------------------------------




int main(int argc, const char* argv[])
{
  //sigaction(SIGPIPE, &(struct sigaction){SIG_IGN}, NULL);
  string args = argv[1];
  vector<string> token;
  for(int i=1;i<4;i++)
    token.push_back(args.substr(i,1));
  for(int i=0;i<token.size();i++){
    if(token[i]=="c")
      USECOLOR = true;
    if(token[i]=="t")
      SHOWTR = true;
    if(token[i]=="s")
      USESERVER = true;
  }//add port arg
  printf("State:\nUSECOLOR=%d\nSHOWTR=%d\nUSESERVER=%d\n",USECOLOR,SHOWTR,USESERVER);

  gettimeofday(&t1, NULL);

  videoPort=4097;
  
  if (SHOWTR) createTrackbars();
  Position position;
  Position positionAV;
  vector<Position>::iterator it;
  vector <Position>posA;
  Targets targets[MAXTARGETS];
  pthread_create(&tcpserver, NULL, opentcp, &positionAV);
  pthread_create(&MJPEG, NULL, VideoCap, NULL);
  if(USESERVER) pthread_create(&videoServerThread, NULL, videoServer, NULL);
  Mat img, HSV, thresholded, output;
  //#remove
  Mat imgGray;

  if(!img.isContinuous()){
    img = img.clone();
    imgGray = img.clone();
  }

  position.x=0;
  position.y=0;
  position.z=0;
  position.angle=0;
  position.dist=0;
  position.OffSetx=0;
  position.OffSety=0;
  while (true)
    {
      pthread_mutex_lock(&frameMutex);
      if(!frame.empty() && newFrame) //check it
	{
	  frame.copyTo(img);

	  pthread_mutex_unlock(&frameMutex);
	  
	  thresholded = ThresholdImage(img);
	  morphOps(thresholded);
	  pthread_mutex_lock(&targetMutex);
	  int nt = findTarget(img, thresholded, targets);
	  cout << " found targets = " << nt << endl;
	  if (nt==2) { 
	    
	    //tLeft->Show();
	    //tRight->Show();
	    //====================================================================
	    //   calculations 
	    //====================================================================

	    double KL = 60.; // 
	    double H0 = 62*KL; // size in pix to cm
	    double W0 =  33*KL; // size in pix to cm
	    float d1 = H0/tLeft->height;
	    float d2 = H0/tRight->height;
	    
	    if (qdebug > 1 ) cout << "\n---------------------------" << endl;
	    //------ area version -----

	    double KS=60;   // inches
	    double S0=1480; // area at this distance 
	    double s1=tLeft->area;
	    double s2=tRight->area;
	    double cs1 = s1/(S0*KS*KS/(d1*d1)); if (cs1>1.) cs1=0.9999;
	    double cs2 = s2/(S0*KS*KS/(d2*d2)); if (cs2>1.) cs2=0.9999;
	    double z0=(d1*cs1+d2*cs2)/2.;
	    double sn1=sqrt(1-cs1*cs1);
	    double sn2=sqrt(1-cs2*cs2);
	    double x0=(d1*sn1+d2*sn2)/2.;
	    double d00=sqrt(x0*x0+z0*z0);
	    alpha = atan2(x0,z0);// 180./3.1415;
	    double offsetX=(FrameWidth/2-centerob.x);
	    double KX=4./3.;
	    double shiftX=offsetX*KX/d00;
	    if (d1<d2) x0=-x0;
	    if (qdebug == -1){
	      printf("==>  s= %.2f %.2f cs= %.2f %.2f  d^2=%.2f s0= %.2f %.2f\n"
		     ,s1,s2,cs1,cs2,d1*d1,S0*KS*KS/(d1*d1),S0*KS*KS/(d1*d1));
	      cout << "==>  x0= " << x0 << " z0= " << z0 << " d00= " << d00 << " al2= " << alpha << endl;
	      printf("\n offset = %.2f  shift = %.2f \n",offsetX,shiftX);
	    }
	    dist = d00;
	    if (d1<d2) 
	      alpha=-alpha;
	    position.z=dist*cos(alpha);
	    position.x=dist*sin(alpha);
	    position.y=tLeft->center.y;
	    position.angle=alpha*180./3.1415;
	    position.dist=dist;
	    position.OffSetx=shiftX; //in inch
	    position.OffSety=   (FrameHeight/2-centerob.y)/dist*2;
	    
	    posA.push_back(position);
	    if(posA.size()>ASIZE){
	      int i= 0;
	      /*
		for(it = posA.begin(); it != posA.end(); it++,i++){
		cout<< i << ": x = " << (*it).x << endl;
		}
	      */
	      //cout << "erase" << endl;
	      posA.erase(posA.begin());
	      i = 0;
	      /*
		for(it = posA.begin(); it != posA.end(); it++,i++){
		cout<< i << ": x = " << (*it).x << endl;
		}
	      */
	    }
	    positionAV.x=0;
	    positionAV.y=0;
	    positionAV.z=0;
	    positionAV.angle=0;
	    positionAV.dist=0;
	    positionAV.OffSetx=0;
	    positionAV.OffSety=0;
	    
	    
	    for(it = posA.begin(); it != posA.end(); it++){
	      //cout<< i << ": x = " << (*it).x << endl;
	      positionAV.x+=(*it).x;
	      positionAV.y+=(*it).y;
	      positionAV.z+=(*it).z;
	      positionAV.angle+=(*it).angle;
	      positionAV.dist+=(*it).dist;
	      positionAV.OffSetx+=(*it).OffSetx;
	      positionAV.OffSety+=(*it).OffSetx;
	      
	    }
	    positionAV.x/=posA.size();
	    positionAV.y/=posA.size();
	    positionAV.z/=posA.size();
	    positionAV.angle/=posA.size();
	    positionAV.dist/=posA.size();
	    positionAV.OffSetx/=posA.size();
	    positionAV.OffSety/=posA.size();
	    

	    if (qdebug == -1){
	      cout << "\nx= " << position.x 
		   << " y= " << position.y 
		   << " z= " << position.z 
		   << " angle= " << position.angle 
		   << " Offset= " << position.OffSetx
		   << endl;
	    }
	    calcTarget();
	    double qtest;
	    double wtest;
	    if(qdebug == -1){
	      cout << "" << endl;
	      cout << "/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*" << endl;
	    }
	  }else{
	    //if (qdebug == -1) 
	    cout << "failed nt = " << nt << endl;
	    positionAV.x=-1;
	    positionAV.y=0;
	    positionAV.z=-1;
	    positionAV.angle=0;
	    positionAV.dist=0;
	    positionAV.OffSetx=0;
	    positionAV.OffSety=0;
	    if (qdebug == -1){
	      cout << "\nx= " << position.x 
		   << " y= " << position.y 
		   << " z= " << position.z 
		   << " angle= " << position.angle 
		   << " Offset= " << position.OffSetx
		   << endl;
	    }
	  }
	  if(SHOWO)
	    imshow("Original", img);
	  if(SHOWT)
	    imshow("Thresholded", thresholded);
	  if (qdebug > 2){
	  printf("------------------------------------------------------------\n");
	  printf("X:%.2f, Y:%.2f, Z:%.2f, ang:%.2f, dist:%.2f, OffX:%.2f, OffY:%.2f\n",position.x, position.y, position.z, position.angle, position.dist, position.OffSetx, position.OffSety);
	  printf("X:%.2f, Y:%.2f, Z:%.2f, ang:%.2f, dist:%.2f, OffX:%.2f, OffY:%.2f\n",positionAV.x, positionAV.y, positionAV.z, positionAV.angle, positionAV.dist, positionAV.OffSetx, positionAV.OffSety);
	  if(position.x>10 || position.x<-10)
	    printf("dist: %f alpha: %f\n",dist,alpha);
	  }
	  pthread_mutex_unlock(&targetMutex);
	  totalfound.clear();
	}
      
      pthread_mutex_unlock(&frameMutex);
      counter++;
      if(counter%10==0){
	gettimeofday(&t2,NULL);
	double dt = t2.tv_usec-t1.tv_usec+1000000 * (t2.tv_sec - t1.tv_sec);
	t1=t2;
	printf("------ Frame rate: %f fr/s \n",10./dt*1e6);
	if(USESERVER && remoteSocket>0){
	  //cvtColor(img,imgGray,6);
	  int bytes = 0;
	  

	  if(USECOLOR && remoteSocket>0){
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
	} // server
	
      } ///-- counter 10
      waitKey(5);

      newFrame = false;
      usleep(10000);
    }

  pthread_join(MJPEG, NULL);

  return 0;
  
}
