#include <opencv2/opencv.hpp>



int main(int argc, const char* argv[]){
 
  // Read input image
  cv::Mat im = cv::imread("target.png");


  std::vector<cv::Point2d> image_points;
  image_points.push_back( cv::Point2d(359, 391) ); // image points on the 2d image


  std::vector<cv::Point3d> model_points;
  model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f)); // IRL coordinates 3d of world

  //---------------------------
  cv::solvePnP(markerObjectPoints, imagePoints, [self currentCameraMatrix], _userDefaultsManager.distCoeffs, rvec, tvec);

  tvec.at<double>(0, 0) *= -1; // I don't know why I have to do it, but translation in X axis is inverted

  cv::Mat R;
  cv::Rodrigues(rvec, R); // R is 3x3

  R = R.t();  // rotation of inverse
  tvec = -R * tvec; // translation of inverse

  cv::Mat T(4, 4, R.type()); // T is 4x4
  T(cv::Range(0, 3), cv::Range(0, 3)) = R * 1; // copies R into T
  T(cv::Range(0, 3), cv::Range(3, 4)) = tvec * 1; // copies tvec into T
  double *p = T.ptr<double>(3);
  p[0] = p[1] = p[2] = 0;
  p[3] = 1;
  //---------------------
     
  // Camera internals
  double focal_length = im.cols; // Approximate focal length.
  cv::Point2d center = cv::Point2d(im.cols/2,im.rows/2);
  cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
     
  std::cout << "Camera Matrix " << std::endl << camera_matrix << std::endl ;
  // Output rotation and translation
  cv::Mat rotation_vector; // Rotation in axis-angle form
  cv::Mat translation_vector;
     
  // Solve for pose
  cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
 
     
  // Project a 3D point (0, 0, 1000.0) onto the image plane.
  // We use this to draw a line sticking out of the nose
     
  std::vector<cv::Point3d> nose_end_point3D;
  std::vector<cv::Point2d> nose_end_point2D;
  nose_end_point3D.push_back(cv::Point3d(0,0,1000.0));
     
  projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
     
     
  for(int i=0; i < image_points.size(); i++)
    {
      circle(im, image_points[i], 3, cv::Scalar(0,0,255), -1);
    }
     
  cv::line(im,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);
     
  std::cout << "Rotation Vector " << std::endl << rotation_vector << std::endl;
  std::cout << "Translation Vector" << std::endl << translation_vector << std::endl;
     
  std::cout <<  nose_end_point2D << std::endl;
     
  // Display image.
  cv::imshow("Output", im);
  cv::waitKey(0);


}
