#ifndef __STEREOCAMERA__H_
#define __STEREOCAMERA__H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include "Camera.h"
#include "yaml-cpp/yaml.h"

using namespace cv;
using namespace std;
class StereoCamera {

  public: 
    StereoCamera();
    StereoCamera(YAML::Node config);
    void config(YAML::Node config);
    void setCameras(Camera &_cam0, Camera &_cam1);
    int stereoTriangulateKeypoints( 
      vector<Point2i> im0_keypoints, vector<Point2i> &im1_keypoints, 
      Mat &im0_img, Mat &im1_img, 
      vector<Point3f> &points_wld, int num_threads);


    double stereo_minZ;
    double stereo_maxZ;
    double stereo_downsize;
    double stereo_match_thresh;
    int stereo_patch_size;

    Camera cam0;
    Camera cam1;
    double focalX;
    double focalY; 
    double centerX;
    double centerY;
    double baseline;


  private:
     void stereoTriangulateKeypoints_thread( 
  vector<Point2i> im0_keypoints, vector<Point2i> *im1_keypoints, 
  Mat im0_down, Mat im1_down, 
  vector<Point3f> *points_wld);

    

};


#endif