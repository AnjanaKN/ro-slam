
#include "headers.h"

using namespace cv;
using namespace std;

class disparity_class{

public:

    double stereo_maxZ;
    double stereo_minZ;

    double focalX;
    double focalY; 
    
    int stereo_downsize;
    double stereo_match_thresh;
    float centerY;
    float centerX;
    float base_line;
    int stereo_patch_size;

    disparity_class();
     

    pcl::PointCloud<pcl::PointXYZ> get_point_cloud(int minDisp, int maxDisp,int winSize,cv::Mat left,cv::Mat right, float base_line,float centerX, float centerY,float f_norm);

 

    int stereoTriangulateKeypoints(vector<Point2i> im0_keypoints, vector<Point2i> &im1_keypoints, Mat &im0_img, Mat &im1_img, vector<Point3f> &points_wld, int num_threads);

private:
  void stereoTriangulateKeypoints_thread( vector<cv::Point2i> im0_keypoints, vector<cv::Point2i> *im1_keypoints, Mat im0_down, Mat im1_down,vector<Point3f> *points_wld);

};