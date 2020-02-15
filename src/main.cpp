#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <fcntl.h>
#include "main.h"

using namespace cv;
/*
  removed old math
  check if ontrackbar() is needed, or can be replaced by NULL
  remove commented out main.h sections.. and check the used values it stores.
  check and remove clnt.c from /src/
  check and maybe shorten / remove tcp_thread.*, tcplib.*, videoserver.cpp, and server.cpp
*/



#define MAXTARGETS 20

//Editable
double MinRatio = 0.1;
double MaxRatio = 0.65;
int MaxDiff = 1000;
//colors
int minH = 0;
int maxH = 255;
int minS = 0;
int maxS = 255; // 5 without filter
int minV = 219;
int maxV = 241;
//booleans
struct Switches {
  bool SHOWORIG;
  bool SHOWTRESH;
  bool SHOWTRACK;
};
//qdebug
int debug1 = 0;
Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));
Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));

bool funcX (cv::Point2f p1, cv::Point2f p2){
  return (p1.x<p2.x);
}
bool funcY (cv::Point2f p1, cv::Point2f p2){
  return (p1.y<p2.y);
}

int qdebug = 0;
//frame
bool newFrame = false;
double FrameWidth;
double FrameHeight;
//threads
pthread_t MJPEG;
void* VideoCap(void* arg);
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
Mat frame;

const std::string trackbarWindowName = "Trackbars";
std::vector <Point>totalfound;
const Scalar BLUE = Scalar(255, 0, 0), RED = Scalar(0,0,255), YELLOW = Scalar(0,255,255), GREEN = Scalar(0,255,0);



//FUNCTIONS----------------------------------------------------------------------
void on_trackbar(int, void*)
{
}

//--------------------------------------------------------------------------------

void createTrackbars(){
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
  Mat test;
  std::vector<Vec4i> hierarchy;
  std::vector<std::vector<Point> > contours;
  
  std::vector<cv::Point2f> corners;
  double qualityLevel = 0.05;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarisDetector = true;
  double k = 0.04;//0.04
  int maxCorners = 100;  
  //dilate(thresholded,thresholded,dilateElement); // makes targets bigger
  //erode(thresholded,thresholded,erodeElement); // makes targets smaller
  //cv::Canny(thresholded,thresholded,50,100,3); // takes outlines
  //GaussianBlur(thresholded,thresholded,Size(3,3),0); // blurrs image, maybe better than blur
  //blur(thresholded,thresholded,Size(2,2)); ^ 

  
  erode(thresholded,thresholded,erodeElement);
  dilate(thresholded,thresholded,dilateElement);
  GaussianBlur(thresholded,thresholded,Size(3,3),0);
  //cv::Canny(thresholded,thresholded,50,100,3); 
  //findContours(thresholded, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE); // simple contours
  findContours(thresholded, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
  std::vector<Vec2f> lines;
  //HoughLinesP(thresholded,lines,1,3.141592/180,75,0,100); // diff type of Houghlines
  //HoughLines(thresholded,lines,1,3.141592/180,65,0,0);
  //cv::goodFeaturesToTrack(thresholded,corners,maxCorners,qualityLevel,minDistance,Mat(),blockSize,useHarisDetector,k);
  
  /* center dot
     std::vector<cv::Moments> mu(contours.size());
     for(int i=0;i<contours.size();i++)
     mu[i]=cv::moments(contours[i],false);
     std::vector<cv::Point2f> mc(contours.size());
     for(int i=0;i<contours.size();i++)
     mc[i]=cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
  */
  /*
  for(int i=0;i<lines.size();i++){
    float rho = lines[i][0],theta = lines[i][1];
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    //line(original,pt1,pt2,YELLOW,3);
    //cv::Vec4i l = lines[i];
    //line(original,Point(l[0],l[1]), Point(l[2] , l[3]), YELLOW, 3);
  }
  */
  if(debug1>0) printf("test in findTarget1\n");

  //------------- preselect by perimetr ------------------
  for (std::vector<std::vector<Point> >::iterator it = contours.begin(); it != contours.end();){
    if (it->size() < 10){//min contour
      it = contours.erase(it);
      if (qdebug>4) std::cout << "erased" << std::endl;
    } else
      ++it;
  }

  //-------------- select by number of contours ------------

  int ntargets=0;
  if (contours.size() > MAXTARGETS ) {
    if (qdebug>2) std::cout <<  " too many targets found = " << contours.size() << std::endl;
    return -1;
  } else if (2 > contours.size() ){
    if (qdebug>2) std::cout << "too little targets found" << std::endl;
    return -1;
  }
  //--------------------------------------------------------
  int asdf = 0;
  std::vector<RotatedRect> minRect(contours.size());
  Mat drawing = Mat::zeros(original.size(), CV_8UC3);
  if(debug1>0) printf("test in findTarget2\n");
  std::vector<std::vector<Point>> hull(contours.size());
  std::vector<std::vector<Point>> art;
  std::vector<Point> approx;
  Mat workingImage(480,640,CV_8UC1,Scalar(0));

  if (!contours.empty() && !hierarchy.empty()) {

    for (int i = 0; i < contours.size(); i++){
      targets[i].NullTargets();
      if (hierarchy[i][2]!=-1) {if (qdebug>2) printf("failed hierarchy\n"); continue;}
      minRect[i] = minAreaRect(Mat(contours[i]));
      cv::Point2f rect_points[4];
      minRect[i].points(rect_points);
      std::copy(rect_points,rect_points+4,targets[i].points);
      targets[i].rect = minRect[i];

      //Mat croppedThresh;
      //thresholded(targets[i].rect.boundingRect()).copyTo(croppedThresh);
      
      
      for (int j = 0; j < 4; j++) {
	line(original,rect_points[j], rect_points[(j + 1) % 4], BLUE, 1, 8);
	circle(original, rect_points[j],3,RED,-1,8,0);
      }
      if(debug1>0) printf("test in contour loop\n");
      //std::vector<std::vector<Point>> contr;
      //std::vector<Vec4i> harch;
      //findContours(croppedThresh, contr, harch, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
      //for(int j=0;j<contr.size();j++)
      //drawContours(original,contr,j,RED);
      //std::vector<std::vector<Point>> hull(contr.size());
      //Mat workingImage(480,640,CV_8UC3,Scalar(0,0,0));
      //std::vector<std::vector<Point>> hull(contours.size());
      //for(int j=0;j<contours.size();j++)
      //convexHull(contours[j],hull[j]);
      //std::vector<std::vector<Point>> art;
      //std::vector<Point> approx;

      convexHull(contours[i],hull[i]);
      //for(int j=0;j<contours.size();j++){
	approxPolyDP(hull[i],approx,arcLength(hull[i],true)*0.05,true);
	art.push_back(approx);

	//}
      //printf("before drawing\n");
      //for(int j=0;j<art.size();j++){
	//const Point* p = &art[j][0];
	//int n = (int)art[i].size();
	//polylines(original,&p,&n,1,true,RED,3,LINE_AA);
	//asdf++;
	//drawContours(original,art,j,RED);  
	//drawContours(workingImage,art,j,RED);
	//imshow("ART",workingImage);
	//}
      //printf("after drawing\n");
      
    }
    for(int j=0;j<art.size();j++){
      drawContours(workingImage,art,j,Scalar(255));
      drawContours(original,art,j,RED);
    }
    //imshow("ART",workingImage);
    cv::goodFeaturesToTrack(workingImage,corners,maxCorners,qualityLevel,minDistance,Mat(),blockSize,useHarisDetector,k);
    for(int i=0;i<corners.size();i++){
      circle(original, corners[i],6,GREEN,-1,8,0);
    }
  }
  if(debug1>0) printf("test end of findTarget\n");
  return ntargets;
}

//--------------------------------------------------------------------------------

Mat ThresholdImage(Mat original)
{
  Mat thresholded;
  inRange(original, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), thresholded);
  return thresholded;
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
  vcap.set(cv::CAP_PROP_EXPOSURE,100);
  FrameWidth = vcap.get(cv::CAP_PROP_FRAME_WIDTH);
  FrameHeight = vcap.get(cv::CAP_PROP_FRAME_HEIGHT);
  while (true){
    pthread_mutex_lock(&frameMutex);
    vcap.read(frame);
    //frame = imread("BlueGoal-084in-Center.jpg");
    pthread_mutex_unlock(&frameMutex);
    newFrame = true;
    usleep(33000);//33000
  }
}


int main(int argc, const char* argv[]){
  Switches switches;
  switches.SHOWORIG = false;
  switches.SHOWTRESH = false;
  switches.SHOWTRACK = false;
  if(argc==2){
    std::string args = argv[1];
    std::vector<std::string> token;
    if(args.compare("--help")==0){
      printf("-t Trackbars\n");
      printf("-o Original\n");
      printf("-b black and white\n");
      return 0;
    }
    printf("args size: %d\n",args.size());
    for(int i=1;i<args.size();i++)
      token.push_back(args.substr(i,1));
    for(int i=0;i<token.size();i++){
      if(token[i]=="t")
	switches.SHOWTRACK = true;
      if(token[i]=="o")
	switches.SHOWORIG = true;
      if(token[i]=="b")
	switches.SHOWTRESH = true;
    }//#FIX
  }
        if(debug1>0) printf("test1\n");

  Mat img, HSV, thresholded, output;
  Targets targets[MAXTARGETS];
  pthread_create(&MJPEG, NULL, VideoCap, NULL);
  int rc = pthread_setname_np(MJPEG, "MJPEG Thread");
  if (rc != 0)
    printf("MJPEG thread fail%d\n",rc);

  if(switches.SHOWTRACK) createTrackbars();
  if(!img.isContinuous()) img = img.clone();
  if(debug1>0) printf("test before loop\n");
  while (true){
    pthread_mutex_lock(&frameMutex);
    if(!frame.empty() && newFrame){ //check if new frame is available
      frame.copyTo(img);
      pthread_mutex_unlock(&frameMutex);
      //dilate(thresholded,thresholded,dilateElement);
      if(debug1>0) printf("test loop\n");
      cv::cvtColor(img, HSV, CV_BGR2HSV);
      thresholded = ThresholdImage(HSV);
      pthread_mutex_lock(&targetMutex);
      int nt = findTarget(img, thresholded, targets);
      if(switches.SHOWORIG)
	imshow("Original", img);
      if(switches.SHOWTRESH)
	imshow("Thresholded", thresholded);
      pthread_mutex_unlock(&targetMutex);
      totalfound.clear();
    }//end of check for new frame
    else pthread_mutex_unlock(&frameMutex);
    waitKey(50);
    newFrame = false;
    //usleep(10000);
  }
  pthread_join(MJPEG, NULL);

  return 0;
  
}
