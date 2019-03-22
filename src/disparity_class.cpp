
#include "headers.h"
#include "disparity_class.h"


disparity_class::disparity_class(){
    stereo_maxZ=672;
    stereo_minZ=192;

     focalX=1;
     focalY=1; 
    
     stereo_downsize=1;
     stereo_match_thresh=0.9;
     centerY=1;
     centerX=1;
     base_line=200.0;
    stereo_patch_size=11;
    }


    pcl::PointCloud<pcl::PointXYZ> disparity_class::get_point_cloud(int minDisp, int maxDisp,int winSize,cv::Mat left,
        cv::Mat right, float base_line,float centerX, float centerY,float f_norm){
        


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
                    
                    cloud_xyz.x = (float)(cloud_xyz.z/f_norm) * (j - centerX); // cols
                    cloud_xyz.y = (float)(cloud_xyz.z/f_norm) * (i - centerY); // rows
                    cloud.push_back(cloud_xyz);
                }
         
            }
        }
    return cloud;
    }




void disparity_class::stereoTriangulateKeypoints_thread( vector<cv::Point2i> im0_keypoints, vector<cv::Point2i> *im1_keypoints, Mat im0_down, Mat im1_down,vector<Point3f> *points_wld)
{

  int minDisparity = (int) (focalX * abs(base_line)) / stereo_maxZ;
  int maxDisparity = (int) (focalX * abs(base_line)) / stereo_minZ;
  minDisparity *= stereo_downsize;
  maxDisparity *= stereo_downsize;
  
  fprintf(stderr,"[INFO] Base_line= %f, MinZ= %f, MaxZ= %f, minDisp= %d, maxDisp= %d\n", base_line, stereo_minZ, stereo_maxZ, minDisparity, maxDisparity);
  if(abs(base_line) < 1e-9) {
    fprintf(stderr,"[ERROR] Base_line is zero (or near zero). Not going to process stereo\n");
    return;
  }

  for(int i=0; i < (int)im0_keypoints.size(); i++) {
    Point2i pt_raw = im0_keypoints.at(i);
    Mat pts_out = Mat(1, 1, CV_64FC2);

    //undistort point
    pts_out.at<cv::Vec2d>(0,0)[0] = pt_raw.x;
    pts_out.at<cv::Vec2d>(0,0)[1] = pt_raw.y;


    //downsize point
    Point2i pt_full_res, pt;
    pt_full_res.x = pts_out.at<cv::Vec2d>(0,0)[0];
    pt_full_res.y = pts_out.at<cv::Vec2d>(0,0)[1];
    pt.x = pt_full_res.x*stereo_downsize;
    pt.y = pt_full_res.y*stereo_downsize;


    //extract image patch around point
    int patch_width = stereo_patch_size;
    if(pt.x < patch_width*2+1 || pt.x > im0_down.cols - patch_width*2-1-1 ||
       pt.y < patch_width*2+1 || pt.y > im0_down.rows - patch_width*2-1-1) {
      points_wld->push_back(Point3f(0,0,0));
      im1_keypoints->push_back(Point2i(-1,-1));
      continue;
    }
    
    Rect patch_rect(pt - Point2i(patch_width,patch_width), pt + Point2i(patch_width,patch_width));
    Mat patch_img = im0_down(patch_rect);

    //set min and max search coords
    int min_search_x;
    int max_search_x;
    if(base_line > 0) {
      min_search_x = pt.x - maxDisparity - patch_img.cols;
      if(min_search_x < 0) {
        min_search_x = 0;
      }
      max_search_x = pt.x - minDisparity + patch_img.cols;
      if(max_search_x < 0) {
        max_search_x = 0;
      }
    } else {
      min_search_x = pt.x + minDisparity - patch_img.cols;
      max_search_x = pt.x + maxDisparity + patch_img.cols;
      if(max_search_x > im1_down.cols - 1) {
        max_search_x = im1_down.cols - 1;
      }
      if(min_search_x > im1_down.cols - 1) {
        min_search_x = im1_down.cols - 1;
      }
    }

    if(min_search_x <= 0 || max_search_x > im1_down.cols-1) {
      points_wld->push_back(Point3f(0,0,0));
      im1_keypoints->push_back(Point2i(-1,-1));
      continue;
    }

    Rect search_rect(Point2i(min_search_x,pt.y-patch_width), Point2i(max_search_x,pt.y+patch_width));
    Mat search_img = im1_down(search_rect);

    if(search_img.cols-patch_img.cols+1 < 1) {
      points_wld->push_back(Point3f(0,0,0));
      im1_keypoints->push_back(Point2i(-1,-1));
      continue;
    }
    //create memory for search result
    Mat search_result(search_img.cols-patch_img.cols+1,1,CV_32F);

    //coduct search
    matchTemplate(search_img, patch_img, search_result,CV_TM_CCORR_NORMED );

    //extract search result
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    minMaxLoc(search_result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    
    //set matching point
    Point2i pt_match = Point2i(min_search_x+maxLoc.x+patch_width, pt.y);

    //get disparity
    float disparity;
    if(base_line > 0) {
      disparity = pt.x - pt_match.x;
    } else {
      disparity = pt_match.x - pt.x;
    }

    //extract 3d coordinate
    float Z=(focalX*abs(base_line))/(disparity/stereo_downsize);
    float X = Z*((pt_full_res.x - centerX)/focalX);
    float Y = Z*((pt_full_res.y - centerY)/focalY);

    //if match not good enough then exit straight away
    if(maxVal < stereo_match_thresh) {
      points_wld->push_back(Point3f(0,0,0));
      im1_keypoints->push_back(Point2i(-1,-1));
      continue;
    }

    //set 3d coordinate
    points_wld->push_back(Point3f(X,Y,Z));
    im1_keypoints->push_back(Point2i(pt_match.x/stereo_downsize, pt_match.y/stereo_downsize));
  }
}

int disparity_class::stereoTriangulateKeypoints( 
  vector<Point2i> im0_keypoints, vector<Point2i> &im1_keypoints, 
  Mat &im0_img, Mat &im1_img, 
  vector<Point3f> &points_wld, int num_threads)
{

  if(im0_keypoints.size() <= 0) { 
    return 0;
  }

  if(im0_keypoints.size() <= num_threads) {
    num_threads = im0_keypoints.size();
  }


  Mat im0_down;
  Mat im1_down;

  //downsize images
  resize(im0_img, im0_down, cv::Size(), stereo_downsize, stereo_downsize);
  resize(im1_img, im1_down, cv::Size(), stereo_downsize, stereo_downsize);

  boost::thread_group threads;
  int chunk_keypoint_cnt = (int) (im0_keypoints.size() / num_threads);

  cout << "Size of keypoint chunks: " << chunk_keypoint_cnt << endl;
  vector<cv::Point2i> chunk_im1_keypoints[num_threads];
  vector<cv::Point3f> chunk_points_wld[num_threads];


  //break keypoints into "num_thread" chunks
  for(int t = 0; t < num_threads; t++) {


     int end_keypoint_idx = (t+1)*chunk_keypoint_cnt;
     if(t == num_threads-1) {
       end_keypoint_idx = im0_keypoints.size();
     }

     int start_keypoint_idx = t*chunk_keypoint_cnt;

     vector<cv::Point2i> chunk_im0_keypoints;

     for(int k = start_keypoint_idx; k < end_keypoint_idx; k++) {
       chunk_im0_keypoints.push_back(im0_keypoints.at(k));
     }

     //send it off to it's thread 
     boost::thread *thread = new boost::thread(boost::bind(&disparity_class::stereoTriangulateKeypoints_thread, this, chunk_im0_keypoints, chunk_im1_keypoints+t, im0_down, im1_down, chunk_points_wld+t));
     threads.add_thread(thread);
  }

  //wait for threads
  threads.join_all();


  for(int t = 0; t < num_threads; t++) {
    for(int k=0; k < chunk_im1_keypoints[t].size(); k++) {
      im1_keypoints.push_back(chunk_im1_keypoints[t].at(k));
    }
    for(int p=0; p < chunk_points_wld[t].size(); p++) {
      points_wld.push_back(chunk_points_wld[t].at(p));
    }
  }

return 1;
}




