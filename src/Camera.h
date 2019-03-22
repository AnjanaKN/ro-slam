#ifndef __CAMERA__H_
#define __CAMERA__H_

#include <opencv2/opencv.hpp>
#include "Camera.h"
#include <ostream>
using namespace std;

#include <sensor_msgs/CameraInfo.h>
//------------------------------------------------------------------------------
cv::Mat getCameraMatrixFromCameraInfo(const sensor_msgs::CameraInfoConstPtr&  cam_info);
cv::Mat getRectificationFromCameraInfo(const sensor_msgs::CameraInfoConstPtr&  cam_info);
cv::Mat getDistortionFromCameraInfo(const sensor_msgs::CameraInfoConstPtr&  cam_info);
cv::Mat getProjectionFromCameraInfo(const sensor_msgs::CameraInfoConstPtr&  cam_info);



class Camera {

  public: 
    Camera() {};
    Camera(string _name, cv::Mat _intrinsics, cv::Mat _distortion, cv::Mat _rectification, cv::Mat _projection);
    Camera(string _name, const sensor_msgs::CameraInfoConstPtr&  cam_info);
 
    vector<cv::Point2i> projectPointsToImage(vector<cv::Point3f> points_wld);
    string name;
    cv::Mat intrinsics;
    cv::Mat distortion;
    cv::Mat rectification;
    cv::Mat projection;
};


#endif