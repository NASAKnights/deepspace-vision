#include "main.h"
#include "pid.h"
#include "servoController.h"

using namespace cv;
/*
  removed old math
  check if ontrackbar() is needed, or can be replaced by NULL
  remove commented out main.h sections.. and check the used values it stores.
  check and remove clnt.c from /src/
  check and maybe shorten / remove tcp_thread.*, tcplib.*, videoserver.cpp, and server.cpp
*/



#define MAXTARGETS 20
#define ASIZE 10

int buttonPress;

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
  bool SHOWHUE;
  bool SHOWTRESH;
  bool SHOWTRACK;
  bool USESERVER;
  bool USECOLOR;
  bool DOPRINT;
  bool SERVO;
  bool TURNCAM;
};
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
pthread_t moveServoThread;
void* moveServo(void* arg);
pthread_t videoServerThread;
void* videoServer(void* arg);
pthread_t i2cSlaveThread;
void* i2cSlave(void* arg);
pthread_t PIDThread;
void* movePID(void* arg);
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
Mat frame;

const std::string trackbarWindowName = "Trackbars";
std::vector <Point>totalfound;
const Scalar BLUE = Scalar(255, 0, 0), RED = Scalar(0,0,255), YELLOW = Scalar(0,255,255);

//threads
void *opentcp(void *arg);
void *videoServer(void *arg);
void *moveServo(void *arg);
void *i2cSlave(void *arg);
int remoteSocket = 0;
int videoPort;
int videoError = 0;
float angleGyro = 0;
float fixedAngle = 0;

Targets *tLeft;
Targets *tRight;
void findAnglePnP(cv::Mat im, Targets *tLeft, Targets *tRight, Position *position);

struct ServoArgs{
  LX16AServo* servo;
  int angle;
  int readAngle;
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

      if(targets[i].center.y < 100)
	continue;
      if(targets[i].center.y > 380)
	continue;
      
      double angle = minRect[i].angle;
      targets[i].angle=angle;

      double RRatio = Width / Height;
      targets[i].ratio=RRatio;
      if ( MinRatio >  RRatio || RRatio > MaxRatio ){if (qdebug>2) printf("failed ratio : %f \n",RRatio); continue;}
      
      double ang1=15, ang2=75;
      if ( abs(abs(angle)-ang1)>10 && abs(abs(angle)-ang2)>10 ){if (qdebug>2) printf("failed angle\n"); continue;}

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

	ttt++;
	if ( abs(dy)>40 ) {if (qdebug>3) printf("failed distance: %f: %d\n",dy,ttt);continue;}
	double arr = targets[i].area/targets[k].area;
	if (arr>1) arr=1./arr;
	if (qdebug > 3)std::cout << " area " << targets[i].area << " " << targets[k].area << " arr= " << arr << std::endl;
	if (arr<0.5) {if (qdebug>2) printf("failed area ratio: %f: %d\n",arr,ttt);continue;}
	if (abs(dx)<targets[i].height/2.0 || abs(dx) > targets[i].height*2.0){continue;}
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
  inRange(original, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), thresholded);
  return thresholded;
}

//-Threads-----------------------------------------------------------------------

void* moveServo(void *arg){
  while(true){
    ServoArgs* obj = (ServoArgs*) arg;
    obj->servo->setAngle(obj->angle * -1);
    usleep(50*1000);
  }
}

void* movePID(void* arg){
  PIDArgs* args = (PIDArgs*) arg;
  PID* drivePID;
  struct timeval tnew, told;
  double turn, dt;
  //drivePID = new PID(0.1,1,-1, Pc, Ic, Dc);  // -- init PID P=0.015
  drivePID = new PID(0.0,1,-1, 0.01,0.001,0.007); // 0.007 // 0 // 0.008
  while(true){
    gettimeofday(&tnew,NULL);
    dt = (tnew.tv_usec-told.tv_usec+1000000 * (tnew.tv_sec - told.tv_sec))*1e-6;
    if(told.tv_sec==0)
      dt = 0.1;
    turn = 0;
    drivePID->button(buttonPress);
    turn = drivePID->calculate(args->driveAngle,-angleGyro,dt,&args->P,&args->I,&args->D);
    
    //printf(", PnP:%.2f\n",args->alpha);
    args->turn = turn;
    /*
    if(buttonPress == 1 && args->move){
      args->turn = turn;
    } else
      args->turn = 0;
    */
    told = tnew;
    //printf("PID: turn: %.2f, driveAngle: %.2f, gyro: %.2f, dt: %f button: %d\n",turn, args->driveAngle, -angleGyro,dt,buttonPress);
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
  vcap.set(cv::CAP_PROP_EXPOSURE,100);
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
void* i2cSlave(void* arg){
  float *angleGyro = (float*) arg;
  printf("enter gyro slave\n");
  int ttyFid = open("/dev/ttyUSB0", O_RDWR);
  if (ttyFid == -1){
    printf( "Error unable to open port\n");
  }
  printf("enter readBus\n");
  unsigned char ch;
  int ret;
  char line[256];
  int counterGyro = 0;
  while(true){
    for(int ii=0;ii<200;ii++){
      int nb = read(ttyFid,&line[ii],1);
      if(nb!=1)
	printf("nb=%d %c\n",nb,ch);
      if(nb<0){sleep(1); ii=11; continue;}
      if(line[ii]=='\n') {line[ii+1]=0; break;}
    }
    //printf("line=%s\n",line);
    float roll, pitch, yaw;
    sscanf(line,"%f,%f,%f",&roll,&pitch,&yaw);
    if(!(counterGyro++ % 60))
      printf("angles: %f,%f,%f\n",roll/100.,pitch/100.,yaw/100.);
    *angleGyro=yaw/100.0f;
  }
}

int main(int argc, const char* argv[]){
  int frFound = 0;
  int frLost = 0;
  double deltaGyro, prevGyro = 0;
  Switches switches;
  switches.SERVO = false;
  switches.SHOWORIG = false;
  switches.SHOWHUE = false;
  switches.SHOWTRESH = false;
  switches.SHOWTRACK = false;
  switches.USESERVER = false;
  switches.USECOLOR = false;
  switches.DOPRINT = false;
  switches.TURNCAM = false;
  if(argc==2){
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
      printf("-r use the servo\n");
      printf("-a turn cam");
      return 0;
    }
    printf("args size: %d\n",args.size());
    for(int i=1;i<args.size();i++)
      token.push_back(args.substr(i,1));
    for(int i=0;i<token.size();i++){
      if(token[i]=="c")
	switches.USECOLOR = true;
      if(token[i]=="t")
	switches.SHOWTRACK = true;
      if(token[i]=="s")
	switches.USESERVER = true;
      if(token[i]=="o")
	switches.SHOWORIG = true;
      if(token[i]=="h")
	switches.SHOWHUE = true;
      if(token[i]=="b")
	switches.SHOWTRESH = true;
      if(token[i]=="p")
	switches.DOPRINT = true;
      if(token[i]=="r")
	switches.SERVO = true;
      if(token[i]=="a")
        switches.TURNCAM = true;
    }//#FIX
  }
  printf("tr : %d\n",switches.SHOWTRACK);
    
  Mat img, HSV, thresholded, output;
  LX16ABus * bus = new LX16ABus();
  bool switchBool = true;
  bool resetCam = true;
  angleGyro = 0;
  fixedAngle = 0;
  bus->openBus("/dev/ttyUSB1");
  LX16AServo * servo = new LX16AServo(bus,1);
  PIDArgs PIDargs;
  PIDargs.move = false;
  PIDargs.alpha = 0;
  PIDargs.servoAngle = 0;
  PIDargs.driveAngle = 0;
  srand(time(NULL));
  ServoArgs servoArgs;
  if(switches.SERVO){
    servoArgs.angle = 0;
    servoArgs.servo = servo;
    pthread_create(&moveServoThread,NULL,moveServo, &servoArgs);
    pthread_setname_np(moveServoThread,"MoveServoThread");
  }
  gettimeofday(&t1, NULL);
  gettimeofday(&timeTest,NULL);
  videoPort=4097;
  Position position, positionAV;
  std::vector<Position>::iterator it;
  std::vector<Position> posA;
  Targets targets[MAXTARGETS];
  pthread_create(&tcpserver, NULL, opentcp, &positionAV);
  pthread_create(&MJPEG, NULL, VideoCap, NULL);
  pthread_create(&i2cSlaveThread, NULL, i2cSlave,&angleGyro);
  pthread_create(&PIDThread, NULL, movePID, &PIDargs);
  int rc = pthread_setname_np(MJPEG, "MJPEG Thread");
  if (rc != 0)
    printf("MJPEG thread fail%d\n",rc);
  rc = pthread_setname_np(tcpserver, "tcpserver");
  if (rc != 0)
    printf("tcp thread fail%d\n",rc);
  rc = pthread_setname_np(i2cSlaveThread,"GyroThread");
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
      nullifyStruct(position);
      printf("nt: %d\n",nt);
      if (nt==2)
	findAnglePnP(img,tLeft,tRight,&position);
      if (nt==2) {
	position.gyro=angleGyro;
	//printf("angleGyro: %f, fixed: %f, PnP: %f\n",position.gyro,fixedAngle,position.angle);
	//put latest values into avaraging struct, delete old one.
	posA.push_back(position);
	if(posA.size()>ASIZE)
	  posA.erase(posA.begin());
	
	//resetting avarage struct
	nullifyStruct(positionAV);
	
	for(it = posA.begin(); it != posA.end(); it++){
	  //std::cout<< i << ": x = " << (*it).x << std::endl;
	  positionAV.x+=(*it).x;
	  positionAV.z+=(*it).z;
	  positionAV.angle+=(*it).angle;
	  positionAV.angle2+=(*it).angle2;
	  positionAV.dist+=(*it).dist;
	  positionAV.OffSetx+=(*it).OffSetx;
	      
	}
	positionAV.x/=posA.size();
	positionAV.z/=posA.size();
	positionAV.angle/=posA.size();
	positionAV.angle2/=posA.size();
	positionAV.dist/=posA.size();
	positionAV.OffSetx/=posA.size();
	positionAV.speed=position.speed;
	positionAV.turn=position.turn;
	positionAV.gyro=position.gyro;
	positionAV.P=position.P;
	positionAV.I=position.I;
	positionAV.D=position.D;

	//====  PID =====
	
	deltaGyro = angleGyro - prevGyro;
	std::cout << "gyro: " << angleGyro << " , " <<deltaGyro << std::endl;
	if (abs(positionAV.angle)>20) 
	  servoArgs.angle += deltaGyro/10.; //-- move camera to ...
	//int servoAngle = servo->readAngle2(); // use instead of servoAgrs belo
	//printf("angle diff: read: %d, angle: %d, pos angle: %.2f\n",servoAngle,servoArgs.angle,positionAV.angle);
	fixedAngle = positionAV.angle+(-angleGyro)+servoArgs.angle+(-positionAV.angle2); // calculate the needed position for the turn
	prevGyro = angleGyro;

	// set PID args to send to server;
	positionAV.P=PIDargs.P;
	positionAV.I=PIDargs.I;
	positionAV.D=PIDargs.D;

	// send PID the required values;
	PIDargs.alpha = positionAV.angle;
	PIDargs.servoAngle = servoArgs.angle;

	if(PIDargs.move == false)
	  PIDargs.driveAngle = fixedAngle;
	positionAV.turn = PIDargs.turn;
	if(buttonPress == 1){
	  servoArgs.angle = positionAV.angle+positionAV.angle2;
	  printf("reset \n");
	  PIDargs.move = true;
	}
	//pid was here

	// set speed to constant (any values above 0.0 works) iff the target is within 200 && -200 bounds, and the distance to target is >65cm
	//                       (since the robot calculates the speed based on turn by its self)
	if(position.OffSetx < 200 && position.OffSetx > -200)
	  if(position.dist>65)
	    positionAV.speed=0.5;//0.25

	
	if(PIDargs.move && switchBool){
	  for(int i=0;i<10;i++){
	    positionAV.turn = PIDargs.turn;
	    //usleep(100*1000);
	  }
	  switchBool = false;
	} else if(PIDargs.move)
	  PIDargs.driveAngle = fixedAngle;
	
	if(qdebug > 4){
	  std::cout << "" << std::endl;
	  std::cout << "/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*" << std::endl;
	}
	if(switches.TURNCAM){
	  frFound++;
	  if(frFound >= 5){
	    frLost = 0;
	    frFound = 6;
	    resetCam = false;
	  }
	}
	
	//printf("found: frFound: %d, frLost:%d\n",frFound, frLost);
      }else{
	if(switches.TURNCAM){
	if(resetCam){
	  resetCam = false;
	  servoArgs.angle=-90;
	  gettimeofday(&lostAtTime,NULL);
	}
	
	if(frLost >= 15 && frFound >= 5){
	  resetCam = true;
	  frFound = 0;
	  frLost = 0;
	}
	if(frLost >= 20){
	  frFound = 0;
	  frLost = 21;
	}
	}
	nullifyStruct(positionAV);
	positionAV.z=-1;
	if(switches.TURNCAM){
	frLost++;
	missFR++;
	gettimeofday(&timeTest,NULL);
	double dt = (timeTest.tv_usec-lostAtTime.tv_usec+1000000 * (timeTest.tv_sec - lostAtTime.tv_sec))*1e-6;
	//printf("lost: frFound: %d, frLost:%d\n",frFound, frLost);
	if(dt > .5){
	  if(servoArgs.angle<60){
	    if(frFound < 5 && frLost > 3){
	      servoArgs.angle += 30;
	      printf("servo incrment Angle: %d\n",servoArgs.angle);
	    }
	  }
	  else
	  resetCam = true;
	  gettimeofday(&lostAtTime,NULL);
	}
	}
      }
      if(switches.SHOWORIG)
	imshow("Original", img);
      if(switches.SHOWTRESH)
	imshow("Thresholded", thresholded);
      if(switches.SHOWHUE)
	imshow("HSV" , HSV);
      if(switches.DOPRINT)
	printf("x=%.2f, z=%.2f, dist=%.2f, angle=%.2f, angle2=%.2f, OffSetx=%.2f, speed=%.2f, turn=%.2f, gyro=%.2f\n",position.x,position.z,position.dist,position.angle,position.angle2,position.OffSetx,position.speed,position.turn,position.gyro);
      pthread_mutex_unlock(&targetMutex);
      totalfound.clear();
      counter++;
    }//end of check for new frame
    else pthread_mutex_unlock(&frameMutex);
    counter2++;
    if(counter%10==0 && counter != counter_old){
      counter_old = counter;
      gettimeofday(&t2,NULL);
      double dt = t2.tv_usec-t1.tv_usec+1000000 * (t2.tv_sec - t1.tv_sec);
      t1=t2;
      printf("------ Frame rate: %f fr/s (%f) \n",10./dt*1e6,counter2/dt*1e6); counter2=0;
      printf("------ Miss Frame: %d fr/s \n",missFR);
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
    //waitKey(5);

    newFrame = false;
    usleep(10000);
  }

  pthread_join(MJPEG, NULL);

  return 0;
  
}
