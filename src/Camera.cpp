#include "Camera.h"

Camera::Camera(string _name, cv::Mat _intrinsics, cv::Mat _distortion, cv::Mat _rectification, cv::Mat _projection) {

  this->name = _name;
  cout << "Loading params for this camera: " << this->name << endl;

  this->intrinsics = _intrinsics;
  cout << "Intrinsics:" << endl;
  cout << this->intrinsics << endl;

  this->distortion = _distortion;
  cout << "Distortion:" << endl;
  cout << this->distortion << endl;

  this->rectification = _rectification;
  cout << "Rectification:" << endl;
  cout << this->rectification << endl;

  this->projection = _projection;
  cout << "Projection:" << endl;
  cout << this->projection << endl;
}

Camera::Camera(string _name, const sensor_msgs::CameraInfoConstPtr&  cam_info) {
  this->name = _name;
  cout << "Loading params for this camera: " << this->name << endl;

  this->intrinsics = getCameraMatrixFromCameraInfo(cam_info);
  cout << "Intrinsics:" << endl;
  cout << this->intrinsics << endl;

  this->distortion = getDistortionFromCameraInfo(cam_info);
  cout << "Distortion:" << endl;
  cout << this->distortion << endl;

  this->rectification = getRectificationFromCameraInfo(cam_info);
  cout << "Rectification:" << endl;
  cout << this->rectification << endl;

  this->projection = getProjectionFromCameraInfo(cam_info);
  cout << "Projection:" << endl;
  cout << this->projection << endl;


}

//------------------------------------------------------------------------------
cv::Mat getCameraMatrixFromCameraInfo(const sensor_msgs::CameraInfoConstPtr&  cam_info) {
  // extract all camera calibration variables
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  
  cameraMatrix.at<double>(0, 0) = cam_info->K[0];
  cameraMatrix.at<double>(1, 1) = cam_info->K[4];
  cameraMatrix.at<double>(0, 2) = cam_info->K[2];
  cameraMatrix.at<double>(1, 2) = cam_info->K[5];

  return cameraMatrix;
}



//------
vector<cv::Point2i> Camera::projectPointsToImage(vector<cv::Point3f> points_wld) {
  vector<cv::Point2i> points_im;
  for(int i=0; i<points_wld.size(); i++) {
    cv::Point3f pt3 = points_wld[i];
    cv::Point2i pt;
    pt.x = ( pt3.x / pt3.z) * this->intrinsics.at<double>(0,0) +  this->intrinsics.at<double>(0,2); 
    pt.y = ( pt3.y / pt3.z) * this->intrinsics.at<double>(1,1) +  this->intrinsics.at<double>(1,2);
    points_im.push_back(pt);
  }
  return points_im;
}
    
//------
cv::Mat getRectificationFromCameraInfo(const sensor_msgs::CameraInfoConstPtr&  cam_info) {

  cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);

  R.at<double>(0, 0) = cam_info->R[0]; R.at<double>(0, 1) = cam_info->R[3]; R.at<double>(0, 2) = cam_info->R[6];
  R.at<double>(1, 0) = cam_info->R[1]; R.at<double>(1, 1) = cam_info->R[4]; R.at<double>(1, 2) = cam_info->R[7];
  R.at<double>(2, 0) = cam_info->R[2]; R.at<double>(2, 1) = cam_info->R[5]; R.at<double>(2, 2) = cam_info->R[8];

  return R;
}

//--------
cv::Mat getDistortionFromCameraInfo(const sensor_msgs::CameraInfoConstPtr&  cam_info) {
  cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
  distCoeffs.at<double>(0, 0) = cam_info->D[0];
  distCoeffs.at<double>(0, 1) = cam_info->D[1];
  distCoeffs.at<double>(0, 2) = cam_info->D[2];
  distCoeffs.at<double>(0, 3) = cam_info->D[3];
  distCoeffs.at<double>(0, 4) = cam_info->D[4];
  return distCoeffs;
}

//------
cv::Mat getProjectionFromCameraInfo(const sensor_msgs::CameraInfoConstPtr&  cam_info) {

  cv::Mat P = cv::Mat::zeros(3, 4, CV_64F);

  P.at<double>(0, 0) = cam_info->P[0]; P.at<double>(1, 0) = cam_info->P[4]; P.at<double>(2, 0) = cam_info->P[8];
  P.at<double>(0, 1) = cam_info->P[1]; P.at<double>(1, 1) = cam_info->P[5]; P.at<double>(2, 1) = cam_info->P[9]; 
  P.at<double>(0, 2) = cam_info->P[2]; P.at<double>(1, 2) = cam_info->P[6]; P.at<double>(2, 2) = cam_info->P[10];
  P.at<double>(0, 3) = cam_info->P[3]; P.at<double>(1, 3) = cam_info->P[7]; P.at<double>(2, 3) = cam_info->P[11];
  return P;
}