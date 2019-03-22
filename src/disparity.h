
#include "headers.h"

using namespace cv;
using namespace std;

class disparity_class{


    public:
    pcl::PointCloud<pcl::PointXYZ> get_point_cloud(int minDisp, int maxDisp,int winSize,cv::Mat left,
        cv::Mat right, float base_line,float Cx, float Cy,float f_norm){
        
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cv::Mat sgbm_disp, disp8;
        Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisp,maxDisp,winSize);
        sgbm->setP1(0);
        sgbm->setP2(0);
        sgbm->setMinDisparity(minDisp);
        sgbm->setNumDisparities(maxDisp);
        sgbm->setUniquenessRatio(25);
        sgbm->setPreFilterCap(10);
        sgbm->setSpeckleWindowSize(15);
        sgbm->setSpeckleRange(7);   // lower the better
        sgbm->setDisp12MaxDiff(10);
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
        sgbm->compute(left,right,sgbm_disp);

        //namedWindow("disparity",0); imshow("disparity", sgbm_disp); waitKey(0);

        sgbm_disp.convertTo(disp8, CV_32F, 1.0/16.0);

        pcl::PointXYZ cloud_xyz;   

  

        std::vector<Point3f> coor;


        for(int i=0;i<left.rows;i++){
            for(int j=0;j<left.cols;j++){

                cloud_xyz.z = (float)((-1* f_norm * base_line) / (disp8.at<float>(i,j)));


                if (cloud_xyz.z >= 0.2 && cloud_xyz.z<3.5 ){
                    
                    cloud_xyz.x = (float)(cloud_xyz.z/f_norm) * (j - Cx); // cols
                    cloud_xyz.y = (float)(cloud_xyz.z/f_norm) * (i - Cy); // rows
                    cloud.push_back(cloud_xyz);
                }
         
            }
        }
    return cloud;
    }



};