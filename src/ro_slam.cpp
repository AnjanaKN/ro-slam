#include "sphere_fitting.h"
#include "svd.h"
#include "slam.h"
#include "disparity_class.h"
#include "visualization.h"

using namespace cv;
//using namespace std;
class Timer
{
public:
    Timer() { clock_gettime(CLOCK_REALTIME, &beg_); }

    double elapsed() {
        clock_gettime(CLOCK_REALTIME, &end_);
        return end_.tv_sec - beg_.tv_sec +
            (end_.tv_nsec - beg_.tv_nsec) / 1000000000.;
    }

    void reset() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
    timespec beg_, end_;
};



int main(){
	disparity_class d;


	vector<cv::Point2i> im0_keypoints;
 	vector<cv::Point2i> im1_keypoints;

	cv::Mat left_image,right_image;
	vector<Point3f> points_wld;
	//int waste=d.stereoTriangulateKeypoints(im0_keypoints,im1_keypoints, left_image, right_image, points_wld, 5);
    Timer tmr;
    custom_svd Svd;
    fit_shape fs;
    Visualisation v;
    //icp_slam slam;
	ifstream inFile;
    inFile.open("/home/anjana/catkin_ws/centers.txt");
	//disparity_class d;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mask_center_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr grape_center_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_center (new pcl::PointCloud<pcl::PointXYZ>);
	float radius;
	pcl::PointXYZ searchPoint;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	pcl::PLYReader reader;
	pcl::PLYWriter w;
	
	string filename="/media/storagedrive/Thesis/Results/March18/filtered_177.ply";
	string filename2="/media/storagedrive/Thesis/Results/March22/filtered_178.ply";
	string center_cloud_name="/media/storagedrive/Thesis/Results/March18/Amask_center_cloud_177.ply";
	string center_cloud_name="/media/storagedrive/Thesis/Results/March22/Amask_center_cloud_178.ply";
	string out_name="/home/anjana/catkin_ws/src/ro_slam/outputs/grape_center_cloud.ply";


	reader.read(filename,*cloud);
	double t = tmr.elapsed();
  	std::cout << "Reading ply: "<<t<<endl;
	
	reader.read(center_cloud_name,*mask_center_cloud);
	tmr.reset();


	
	//cloud=d.get_point_cloud(192,672,11,left_image, right_image,);


//Create kdtree
  	kdtree->setInputCloud(cloud);

	
	//Read roi centers
	for(int i=0;i<429;i++){
		inFile>>searchPoint.x;
		inFile>>searchPoint.y;
		inFile>>searchPoint.z;
		
		roi_cloud=Svd.get_neighbors(0, 400,0.02,kdtree,cloud,searchPoint);

		Svd.compute_cov_matrix(roi_cloud);
		Svd.compute_SVD(S.C);
		

		sphere_center->clear();
		radius=fs.fit_sphere(roi_cloud,sphere_center,0.001,0.02,true);
		
		if (radius<0.03 & radius >0.008)
	 		{
				
				S.write_singular_values();
	 			grape_center_cloud->push_back(sphere_center->points[0]);

	 	    }
	 	
 		
	}

	inFile.close();
   
	std::cout << "Fit spheres: "<<tmr.elapsed()<<endl;
	

//Count total spheres
	cout<<fs.count_fitted_spheres()<<endl;

//Write centers to point cloud
	w.write(out_name,*grape_center_cloud);


//SLAM

	//slam.point_to_point(const pcl::PointCloud<pcl::PointXYZ>::Ptr fixed, const pcl::PointCloud<pcl::PointXYZ>::Ptr moving);
	

//visdualize grapes as spheres
/*pcl::visualization::PCLVisualizer::Ptr viewer = v.simpleVis(grape_center_cloud);
	while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

*/
	return 0;
}





