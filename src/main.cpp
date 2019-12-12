#include "main.h"
#include "servoController.h"

/**
 * #FIX // why does that say counter2?
 *
 */


using namespace cv;

#define MAXTARGETS 20
#define ASIZE 10

//Editable
double MinRatio = 0.1;
double MaxRatio = 0.65;
int MaxDiff = 1000;

int minH = 0;
int maxH = 255;//175
int minS = 0;
int maxS = 255;//14
int minV = 219;//181
int maxV = 241;//255

//booleans
bool USEIPCAM = false;
bool SHOWO = false;
bool SHOWH = false;
bool SHOWT = false; 
bool SHOWTR = false;
bool USESERVER = false;
bool USECOLOR = false;
bool DOPRINT = false;
/*
bool SideShow = false;
bool AngleShow = false;
bool AreaShow = false;
bool RatioShow = false;
*/
//frame counter
int counter = 0, counter2=0, counter_old=0;
struct timeval t1, t2;
int qdebug = 0;

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
const std::string trackbarWindowName = "Trackbars";
Mat frame;
std::vector <Point>totalfound;
const Scalar BLUE = Scalar(255, 0, 0), RED = Scalar(0,0,255), YELLOW = Scalar(0,255,255);

//threads
void *opentcp(void *arg);
void *videoServer(void *arg);
int remoteSocket = 0;
int videoPort;
int videoError = 0;

Targets *tLeft;
Targets *tRight;
void findAnglePnP(cv::Mat im, Targets *tLeft, Targets *tRight);

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
  sprintf( TrackbarName, "H_MIN", minH);
  sprintf( TrackbarName, "H_MAX", maxH);
  sprintf( TrackbarName, "S_MIN", minS);
  sprintf( TrackbarName, "S_MAX", maxS);
  sprintf( TrackbarName, "V_MIN", minV);
  sprintf( TrackbarName, "V_MAX", maxV);
  createTrackbar( "H_MIN", trackbarWindowName, &minH, 255, on_trackbar );
  createTrackbar( "H_MAX", trackbarWindowName, &maxH, 255, on_trackbar );
  createTrackbar( "S_MIN", trackbarWindowName, &minS, 255, on_trackbar );
  createTrackbar( "S_MAX", trackbarWindowName, &maxS, 255, on_trackbar );
  createTrackbar( "V_MIN", trackbarWindowName, &minV, 255, on_trackbar );
  createTrackbar( "V_MAX", trackbarWindowName, &maxV, 255, on_trackbar );
}

//--------------------------------------------------------------------------------

int findTarget(Mat original, Mat thresholded, Targets *targets)
{
  std::vector<Vec4i> hierarchy;
  std::vector<std::vector<Point> > contours;
  findContours(thresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  //------------- preselect by perimetr ------------------
  //std::cout << "initial targets = " << contours.size() << std::endl;
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

  std::vector<RotatedRect> minRect(contours.size());
  Mat drawing = Mat::zeros(original.size(), CV_8UC3);

  if (!contours.empty() && !hierarchy.empty()) {

    for (int i = 0; i < contours.size(); i++){
      targets[i].NullTargets();
      if (hierarchy[i][2]!=-1) {if (qdebug>2) printf("failed hierarchy\n"); continue;}
      minRect[i] = minAreaRect(Mat(contours[i]));
      cv::Point2f rect_points[4];
      minRect[i].points(rect_points);
      std::copy(rect_points,rect_points+4,targets[i].points);
      if(qdebug>2)
	std::cout<<*targets[i].points<<"\n";
      double w1 = sqrt(pow((rect_points[0].x-rect_points[1].x),2)+
		       pow((rect_points[0].y-rect_points[1].y),2));
      double w2 = sqrt(pow((rect_points[2].x-rect_points[3].x),2)+
		       pow((rect_points[2].y-rect_points[3].y),2));
      double h1 = sqrt(pow((rect_points[1].x-rect_points[2].x),2)+
		       pow((rect_points[1].y-rect_points[2].y),2));
      double h2 = sqrt(pow((rect_points[3].x-rect_points[0].x),2)+
		       pow((rect_points[3].y-rect_points[0].y),2));

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

      if ( MinRatio >  RRatio || RRatio > MaxRatio ){if (qdebug>2) printf("failed ratio : %f \n",RRatio); continue;}
 
      //if (abs(angle)<10) continue; 

      double ang1=15, ang2=75;
      if ( abs(abs(angle)-ang1)>20 && abs(abs(angle)-ang2)>20 ){if (qdebug>2) printf("failed angle\n"); continue;}

      targets[i].number = i;
      targets[i].status=1; // --- preselected target ----	
      int ttt = 0;
      for ( int k = 0; k < i; k++) {

	if (targets[i].status==0) {if (qdebug>2) printf("failed status\n");continue;}
	
	double xi = targets[i].center.x;
	double xk = targets[k].center.x;
	double yi = targets[i].center.y;
	double yk = targets[k].center.y;

	double dx = xi - xk;
	double dy = yi - yk;

	//std::cout << " i,k = " << i << " " << k <<  " dx = " << dx << " dy = " << dy << std::endl;
	//std::cout << " area  = " << targets[i].area << " " << targets[k].area << std::endl;
	ttt++;
	if ( abs(dy)>40 ) {if (qdebug>3) printf("failed distance: %f: %d\n",dy,ttt);continue;}
	double arr = targets[i].area/targets[k].area;
	if (arr>1) arr=1./arr;
	if (qdebug > 3)std::cout << " area " << targets[i].area << " " << targets[k].area << " arr= " << arr << std::endl;
	if (arr<0.5) {if (qdebug>2) printf("failed area ratio: %f: %d\n",arr,ttt);continue;}
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
  //inRange(original, Scalar(minR, minG, minB), Scalar(maxR, maxG, maxB), thresholded);
  inRange(original, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), thresholded);
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
    std::cout << "use webcam at " << webcam << std::endl;	
    while (!vcap.open(webcam)){
      std::cout << "cant connect to " << webcam << std::endl;
      usleep(10000000);
    }
    std::cout << "webcam:onnected to " << webcam << std::endl;
  }else{
    while (!vcap.open(0)){
      std::cout << "cant connect" << std::endl;
      usleep(10000000);
    }
  }
  //vcap.set(CV_CAP_PROP_CONTRAST, 100);
  //vcap.set(CV_CAP_PROP_EXPOSURE, 255);
  //vcap.set(CV_CAP_PROP_SATURATION, 100);
  //vcap.set(CV_CAP_PROP_GAIN,0);

  vcap.set(cv::CAP_PROP_BRIGHTNESS,100);
  vcap.set(cv::CAP_PROP_AUTO_EXPOSURE, 255);
  FrameWidth = vcap.get(cv::CAP_PROP_FRAME_WIDTH);
  FrameHeight = vcap.get(cv::CAP_PROP_FRAME_WIDTH);
  
  
  std::cout << "success!" << std::endl;
  
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
      std::cout << "turn left" << std::endl;
    else 
      std::cout << "turn right" << std::endl;
    if ((tLeft->center.x > FrameWidth/2-160 && tLeft->center.x < FrameWidth/2) && (tRight->center.x < FrameWidth/2+160 && tRight->center.x > FrameWidth/2))
      std::cout << "stay on track" << std::endl;
    else if (tLeft->center.x < FrameWidth/2)
      std:: cout << "move left" << std::endl;
    else if (tRight->center.x > FrameWidth/2)
      std::cout << "move right" << std::endl;
  }
  centerob.x = (tRight->center.x-tLeft->center.x)/2+tLeft->center.x;
  centerob.y = (tRight->center.y-tLeft->center.y)/2+tLeft->center.y;
  int testq; 
}
//-----------------------------------------------------------------------------
inline void nullifyStruct(Position &pos){
  pos.x=0;
  pos.y=0;
  pos.z=0;
  pos.angle=0;
  pos.dist=0;
  pos.OffSetx=0;
  pos.OffSety=0;
}



int main(int argc, const char* argv[])
{
  bool RANDOM = false;
  if(argc>1){
    std::string args = argv[1];
    std::vector<std::string> token;
    if(args.compare("--help")==0){
      printf("-c color\n");
      printf("-t Trackbars\n");
      printf("-s Server on\n");
      printf("-o Original\n");
      printf("-h HSV\n");
      printf("-b black and white\n");
      printf("-p print stuff\n");
      printf("-r random motor move\n");
      return 0;
    }
    printf("args size: %d\n",args.size());
    for(int i=1;i<args.size();i++)
      token.push_back(args.substr(i,1));
    for(int i=0;i<token.size();i++){
      if(token[i]=="c")
	USECOLOR = true;
      if(token[i]=="t")
	SHOWTR = true;
      if(token[i]=="s")
	USESERVER = true;
      if(token[i]=="o")
	SHOWO = true;
      if(token[i]=="h")
	SHOWH = true;
      if(token[i]=="b")
	SHOWT = true;
      if(token[i]=="p")
	DOPRINT = true;
      if(token[i]=="r")
	RANDOM = true;
    }//#FIX
  }
  printf("State:\nUSECOLOR=%d\nSHOWTR=%d\nUSESERVER=%d\n",USECOLOR,SHOWTR,USESERVER);
  printf("DOPRINT=%d\n",DOPRINT);
  Mat img, HSV, thresholded, output;
  LX16ABus * bus = new LX16ABus();
  
  bus->openBus("/dev/ttyUSB0");
  LX16AServo * servo = new LX16AServo(bus,1); // 254=broadcast
  srand(time(NULL));
  if(RANDOM){
    servo->setAngle((int) rand()%1000);
  }
  servo->setAngle(500);
  int missFR = 0;
  gettimeofday(&t1, NULL);
  videoPort=4097;
  Position position, positionAV;
  std::vector<Position>::iterator it;
  std::vector<Position> posA;
  Targets targets[MAXTARGETS];
  pthread_create(&tcpserver, NULL, opentcp, &positionAV);
  pthread_create(&MJPEG, NULL, VideoCap, NULL);
  if(USESERVER) pthread_create(&videoServerThread, NULL, videoServer, NULL);
  if(SHOWTR) createTrackbars();
  if(!img.isContinuous()) img = img.clone();
  //set all values to 0 before program begins in loop forever.
  nullifyStruct(position);
  nullifyStruct(positionAV);
  while (true){
    pthread_mutex_lock(&frameMutex);
    if(!frame.empty() && newFrame){ //check if new frame is available
      frame.copyTo(img);
      pthread_mutex_unlock(&frameMutex);
      cv::cvtColor(img, HSV, CV_BGR2HSV);
      thresholded = ThresholdImage(HSV);
      morphOps(thresholded);
      pthread_mutex_lock(&targetMutex);
      int nt = findTarget(img, thresholded, targets);
      if (nt==2) findAnglePnP(img,tLeft,tRight);
      //else       findAnglePnP(img,NULL,NULL); //-- for test only
      if (qdebug > 4)std::cout << " found targets = " << nt << std::endl;
      if (nt==2) { //found 2 targets, now to calculations to find distance from them.
	    
	//tLeft->Show();
	//tRight->Show();
	//====================================================================
	//   calculations 
	//====================================================================

	double ScalingDist = 60.; // 
	double HeightScalar = 62*ScalingDist; // size in pix to cm
	double WidthScalar =  33*ScalingDist; // size in pix to cm
	float d1 = HeightScalar/tLeft->height;
	float d2 = HeightScalar/tRight->height;
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
	if(qdebug > 2){
	  printf("Height: %.2f, %.2f\n"
		 "Width: %.2f, %.2f\n"
		 "==>  s= %.2f %.2f cs= %.2f %.2f s0= %.2f %.2f\n"
		 "==>  x0 = %.2f z0= %.2f d00= %.2f al2= %.2f\n"
		 "==>  offset = %.2f shift %.2f\n"
		 ,tLeft->height,tRight->height,tLeft->width,tRight->width,s1,s2,cs1,cs2,S0*KS*KS/(d1*d1),S0*KS*KS/(d1*d1),x0,z0,d00,alpha,offsetX,shiftX);
	}
	    
	dist = d00;
	if (d1<d2) 
	  alpha=-alpha;

	//enter values into struct
	position.z=dist*cos(alpha);
	position.x=dist*sin(alpha);
	position.y=tLeft->center.y;
	position.angle=alpha*180./3.1415;
	position.dist=dist;
	position.OffSetx=shiftX; //in inch
	position.OffSety=   (FrameHeight/2-centerob.y)/dist*2;

	//put latest values into avaraging struct, delete old one.
	posA.push_back(position);
	if(posA.size()>ASIZE)
	  posA.erase(posA.begin());

	//resetting avarage struct
	nullifyStruct(positionAV);
	    
	    
	for(it = posA.begin(); it != posA.end(); it++){
	  //std::cout<< i << ": x = " << (*it).x << std::endl;
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
	    

	if (qdebug > 0){
	  printf("x= %f y= %f z= %f angle= %f Offset= %f\n",position.x,position.y,position.z,position.angle,position.OffSetx);
	}
	calcTarget();
	double qtest;
	double wtest;
	if(qdebug > 4){
	  std::cout << "" << std::endl;
	  std::cout << "/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*" << std::endl;
	}
      }else{
	//if (qdebug == -1) 
	if(qdebug>1) std::cout << "failed nt = " << nt << std::endl;
	nullifyStruct(positionAV);
	positionAV.x=-1;
	positionAV.z=-1;
	if (qdebug > 0){
	  printf("x= %f y= %f z= %f angle= %f Offset= %f\n",position.x,position.y,position.z,position.angle,position.OffSetx);
	}
	missFR++;


	    
      }
      if(SHOWO)
	imshow("Original", img);
      if(SHOWT)
	imshow("Thresholded", thresholded);
      if(SHOWH)
	imshow("HSV" , HSV);
      if (qdebug > 0){
	printf("------------------------------------------------------------\n");
	if(DOPRINT){
	  printf("Current: X:%.2f, Y:%.2f, Z:%.2f, ang:%.2f, dist:%.2f, OffX:%.2f, OffY:%.2f\n",position.x, position.y, position.z, position.angle, position.dist, position.OffSetx, position.OffSety);
	  printf("Avarage: X:%.2f, Y:%.2f, Z:%.2f, ang:%.2f, dist:%.2f, OffX:%.2f, OffY:%.2f\n",positionAV.x, positionAV.y, positionAV.z, positionAV.angle, positionAV.dist, positionAV.OffSetx, positionAV.OffSety);
	if(position.x>10 || position.x<-10)
	  printf("dist: %f alpha: %f\n",dist,alpha);
	}
	  
      }
      pthread_mutex_unlock(&targetMutex);
      totalfound.clear();
      counter++;
    }//end of check for new frame
    pthread_mutex_unlock(&frameMutex);
    counter2++;
    if(counter%10==0 && counter != counter_old){
      counter_old = counter;
      gettimeofday(&t2,NULL);
      double dt = t2.tv_usec-t1.tv_usec+1000000 * (t2.tv_sec - t1.tv_sec);
      t1=t2;
      if(qdebug > 3){
	printf("------ Frame rate: %f fr/s (%f) \n",10./dt*1e6,counter2/dt*1e6); counter2=0;//#FIX
	printf("------ Miss Frame: %d fr/s \n",missFR);
      }
      missFR=0;
      if(USESERVER && remoteSocket>0){
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
