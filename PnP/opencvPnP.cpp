#include <opencv2/opencv.hpp>


int main(int argc, const char* argv[]){
  cv::Mat im = cv::imread("targetPnP.jpg");
  std::vector<cv::Point2d> img2dpoints;
  std::vector<cv::Point3d> mod3dpoints;
  cv::Point2d center = cv::Point2d(im.cols/2,im.rows/2);
  int dx = 5;
  if(argc>1) dx=atoi(argv[1]);
  int dy = dx/5.;

  img2dpoints.push_back(cv::Point2d(center.x-100+dx,center.y-100-dy));
  img2dpoints.push_back(cv::Point2d(center.x-100+dx,center.y+100+dy));
  img2dpoints.push_back(cv::Point2d(center.x+100-dx,center.y+100-dy));
  img2dpoints.push_back(cv::Point2d(center.x+100-dx,center.y-100+dy));

  mod3dpoints.push_back(cv::Point3d(-1.0,-1.0,0.0));
  mod3dpoints.push_back(cv::Point3d(-1.0, 1.0,0.0));
  mod3dpoints.push_back(cv::Point3d( 1.0, 1.0,0.0));
  mod3dpoints.push_back(cv::Point3d( 1.0,-1.0,0.0));

  double focal_length = im.cols;
  cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type);

  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat rMat;
  cv::solvePnP(mod3dpoints, img2dpoints, camera_matrix, dist_coeffs, rvec, tvec);
  cv::Rodrigues(rvec,rMat);
  printf("WHAT\n");
  std::cout << " focal length = " << focal_length << std::endl;
  std::cout << " tvec = " << tvec << std::endl;
  std::cout << " rvec = " << rvec << std::endl;
  std::cout << " rMat = " << rMat << std::endl;

  cv::Mat rotationVecTest = rMat.t();
  cv::Rodrigues(rMat.t(),rotationVecTest);
  cv::Mat tvecT = -rMat.t()*tvec;

  
  std::cout << rvec << std::endl;
  printf("\n\n");
  std::cout << rMat.t() << std::endl;
  printf("\n\n");
  std::cout << rMat << std::endl;
  printf("\n\n");
  std::cout << rotationVecTest*180./3.151492 << std::endl;
  printf("\n\n");
  std::cout << rotationVecTest << "\n:\n" << tvecT << std::endl;
  printf("\n\n");
  double* vec = tvecT.ptr<double>();
  printf("angle:%f\n",atan2(vec[0],-vec[2])*180./3.151492);
  
  double* tv = tvec.ptr<double>();
  cv::Mat rotT=rMat.t();
  cv::Point3d me(0,0,200.0);
  cv::Mat mme = (cv::Mat_<double>(3,1) << me.x, me.y, me.z);
  cv::Mat rme = rMat*mme;
  double* vme = rme.ptr<double>();
  double alpha = atan2(vme[0],vme[2]);
  cv::Point3d p3me(vme[0],vme[1],vme[2]);
  printf("rme x=%f y=%f z=%f alpha=%f \n",vme[0],vme[1],vme[2],alpha*180./3.14159);


  std::vector<cv::Point3d> axis3D;
  std::vector<cv::Point2d> axis2D;
  axis3D.push_back(cv::Point3d(1.,0,0));
  axis3D.push_back(cv::Point3d(0,1.,0));
  axis3D.push_back(cv::Point3d(0,0,1.));
  projectPoints(axis3D, rvec, tvec, camera_matrix, dist_coeffs, axis2D);
  for(int i=0; i < img2dpoints.size(); i++)
    circle(im, img2dpoints[i], 3, cv::Scalar(0,0,255), 2);
  cv::line(im, center, axis2D[0], cv::Scalar(255,0,0),2);
  cv::line(im, center, axis2D[1], cv::Scalar(0,255,0),2);
  cv::line(im, center, axis2D[2], cv::Scalar(0,0,255),2);
  cv::imshow("Output", im);
  cv::waitKey(0);

  
}
