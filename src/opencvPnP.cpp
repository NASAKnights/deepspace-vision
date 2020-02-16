#include "main.h"
#include <opencv2/calib3d.hpp>
#define PI 3.141592
static int model=0;
std::vector<cv::Point3d> mod3d;
std::vector<cv::Point2d> mod2d;

int debugPnP = 1;
bool showImg = false;

#define USE_LEFT_T
#define USE_RIGHT_T
//#define USE_OUTER_T
bool useBoth = true;
// (inches)
//39 top
//19.5 strip
//18 height
double i2cm = 2.54;
double tTop = 39.0* i2cm;
double tStrip = 19.5 * i2cm;
double tHeight = 18.0 * i2cm;

void findAnglePnP(cv::Mat im, Targets target,Position* position){
  std::vector<cv::Point2d> img2dpoints;
  //std::vector<cv::Point2d> img2dpointstest;
  std::vector<cv::Point2d> tBox;
  /*
  img2dpoints.push_back(target.corners[0]);
  img2dpoints.push_back(target.corners[1]);
  img2dpoints.push_back(target.corners[2]);
  img2dpoints.push_back(target.corners[3]);
  */
  mod3d.clear();
  mod3d.push_back(cv::Point3d(+tTop/2.0,-tHeight,0.0));//top right
  mod3d.push_back(cv::Point3d(-tTop/2.0,-tHeight,0.0));//top left
  mod3d.push_back(cv::Point3d(-tStrip/2.0,0.0,0.0));//bottom left
  mod3d.push_back(cv::Point3d(+tStrip/2.0,0.0,0.0));// bottom right
  
  //std::vector<cv::Mat> mod2dm;
  cv::Point2d center = cv::Point2d(im.cols/2,im.rows/2);//use the found center

  while(true){
    int change = 0;
    for(int i=0;i<3;i++){
      if(target.corners[i].y > target.corners[i+1].y){
	change = 1;
	cv::Point2d tmp = target.corners[i];
	target.corners[i]=target.corners[i+1];
	target.corners[i+1]=tmp;
      }
      
    }
    if(change==0) break;
  }
  if(target.corners[0].x < target.corners[1].x){
    img2dpoints.push_back(target.corners[1]);
    img2dpoints.push_back(target.corners[0]);
  } else {
    img2dpoints.push_back(target.corners[0]);
    img2dpoints.push_back(target.corners[1]);
  }
  if(target.corners[2].x < target.corners[3].x){
    img2dpoints.push_back(target.corners[2]);
    img2dpoints.push_back(target.corners[3]);
  } else {
    img2dpoints.push_back(target.corners[3]);
    img2dpoints.push_back(target.corners[2]);
  }
  /*for(int i=0; i < (int) mod3d.size(); i++) {
    circle(im, mod2d[i], i, cv::Scalar(0,255,0), 2);
    circle(im, cv::Point2d(mod3d[i].x*3+center.x,mod3d[i].y*3+center.y+20), i*5, cv::Scalar(255,255,0), 2);  
  }*/
  //-----------  initialization of camera -------------
   
  double focal_length = im.cols;
  //double focal_length = im.cols*1.13;
  cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type);

  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat rMat;

  //cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec); // default
  cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_P3P); // test out
  //cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, true); //test this out!? plug in old r and t vec values
  
  //cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  //cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_DLS);
  //cv::solvePnP(mod3d, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_UPNP); 
  cv::Rodrigues(rvec,rMat);
  cv::Mat rotationVecTest;
  cv::Rodrigues(rMat.t(),rotationVecTest);
  cv::Mat tvecT = -rMat.t()*tvec;
  double* transvec = tvec.ptr<double>();
  //transvec is the transposing vector of target, x=0 y=1 z=2; x=dir y=height z=depth; care for rotated camera
  double distance = sqrt(transvec[0]*transvec[0]+transvec[2]*transvec[2]);
  double angle = atan2(transvec[0],transvec[2]);
  //double angle3 = atan2(transvec[1],distance);
  cv::Mat xWorldd = -rMat.t()*tvec;
  double* xWorld = xWorldd.ptr<double>();
  //xWorld is the irl coords of cam vs target
  double angle2 = atan2(xWorld[0],-xWorld[2]);
  double dist3D = sqrt(distance*distance+xWorld[1]*xWorld[1]);
  double angle3 = atan2(xWorld[1],distance);
  cv::Point2d tc((img2dpoints[2].x+img2dpoints[3].x)/2.0,(img2dpoints[2].y+img2dpoints[3].y)/2.0);
  //std::cout << img2dpoints[2] << std::endl;
  //std::cout << img2dpoints[3] << std::endl;
  position->x=sin(angle2)*distance;
  //position->y=
  position->z=cos(angle2)*distance;
  position->dist=distance;
  position->angle=angle*(180./PI);
  position->angle2=angle2*(180./PI);
  //position->OffSetx=tc.x-center.x;
  //printf("transvec: %.2f, %.2f, %.2f; xWorld: %.2f, %.2f, %.2f\n",transvec[0],transvec[1],transvec[2],xWorld[0],xWorld[1],xWorld[2]);
  //printf("angle3: %.2f, 3DDist: %.2f\n",angle3*(180./PI),dist3D);
  //printf("c0 %.2f, c1 %.2f, c2 %.2f, c3 %.2f",img2dpoints[0],img2dpoints[1],img2dpoints[2],img2dpoints[3]);
  //for(int k=0;k<4;k++)
  //std::cout << img2dpoints[k] << std::endl;
  std::vector<cv::Point3d> axis3D;
  std::vector<cv::Point2d> axis2D;
  axis3D.push_back(cv::Point3d(10.,0,0));
  axis3D.push_back(cv::Point3d(0,10.,0));
  axis3D.push_back(cv::Point3d(0,0,-10.));
  projectPoints(axis3D, rvec, tvec, camera_matrix, dist_coeffs, axis2D);
  if(debugPnP>3)
    std::cout << " axis 2D = " << axis2D << std::endl;
  for(int i=0; i < (int) img2dpoints.size (); i++){
    circle(im, img2dpoints[i], 7,cv::Scalar(0,0,255),-1,8,0);
  }
  cv::line(im, tc, axis2D[0], cv::Scalar(255,0,0),2);//x-blue
  cv::line(im, tc, axis2D[1], cv::Scalar(0,255,0),2);//y-green
  cv::line(im, tc, axis2D[2], cv::Scalar(0,0,255),2);//z-red
  //cv::circle(im,center,4,cv::Scalar(255,255,255),2);
  if(showImg){
    cv::imshow("Output", im);
    cv::waitKey(5000);
  }

  
}

