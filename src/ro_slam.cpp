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


	//vector<cv::Point2i> im0_keypoints;
 	//vector<cv::Point2i> im1_keypoints;

	//cv::Mat left_image,right_image;
	//vector<Point3f> points_wld;
	//int waste=d.stereoTriangulateKeypoints(im0_keypoints,im1_keypoints, left_image, right_image, points_wld, 5);
   // cout<<"initialising1"<<endl;	
    Timer tmr;
    custom_svd Svd;
    fit_shape fs;
    Visualisation v;
	//cout<<"initialising2"<<endl;	
    
	//ifstream inFile;
    //inFile.open("/home/anjana/catkin_ws/centers.txt");
	//disparity_class d;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mask_center_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr grape_center_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_center (new pcl::PointCloud<pcl::PointXYZ>);
	float radius;
	//cout<<"initialising3"<<endl;	
	pcl::PointXYZ searchPoint;
	//cout<<"initialising4"<<endl;	
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	//cout<<"initialising5"<<endl;	
	pcl::PLYReader reader;
	pcl::PLYWriter w;
	
	string filename;
	
	string center_cloud_name;
	string out_name;
	filename="/media/storagedrive/Thesis/Results/March22/Q_"+ patch::to_string(177)+".ply";
	reader.read(filename,*cloud);
	//cout<<"initialising6"<<endl;	
	icp_slam slam(cloud);
	//cout<<"initialising7"<<endl;	
cout<<"Entering slam loop"<<endl;
for (int point_cloud_number=177;point_cloud_number<190;point_cloud_number++)
 {	cout<<"Cloud number: "<<point_cloud_number<<endl;
	filename="/media/storagedrive/Thesis/Results/March22/Q_"+ patch::to_string(point_cloud_number)+".ply";
	center_cloud_name="/media/storagedrive/Thesis/Results/March22/center_cloud_"+patch::to_string(point_cloud_number)+"_new.ply";
	out_name="/home/anjana/catkin_ws/src/ro_slam/outputs/grape_center_cloud"+patch::to_string(point_cloud_number)+".ply";
	reader.read(filename,*cloud);
	double t = tmr.elapsed();
  	//std::cout << "Reading ply: "<<t<<endl;
	
	reader.read(center_cloud_name,*mask_center_cloud);
//	tmr.reset();

	grape_center_cloud->clear();

//Create kdtree
  	kdtree->setInputCloud(cloud);

	
	//Read roi centers
	for(int i=0;i<mask_center_cloud->points.size();i++){
	
		searchPoint=mask_center_cloud->points[i];
		roi_cloud=Svd.get_neighbors(0,100,0.02,kdtree,cloud,searchPoint);
		Svd.compute_cov_matrix(roi_cloud);
		Svd.compute_SVD(Svd.C);

				
		Svd.write_singular_values();
		*grape_center_cloud += *(fs.fit_sphere(roi_cloud,sphere_center,0.001,0.02,true));
 		
	}

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (grape_center_cloud);
  sor.setMeanK (1);
  sor.setStddevMulThresh (1.0);
  sor.filter (*grape_center_cloud);
	//inFile.close();
   	//std::cout << "Fit spheres: "<<tmr.elapsed()<<endl;
	
//Count total spheres
	cout<<"Spheres: "<<fs.count_fitted_spheres()<<endl;
//visualize grapes as spheres
/*pcl::visualization::PCLVisualizer::Ptr viewer = v.simpleVis(grape_center_cloud);
	while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
*/


//Write centers to point cloud
	w.write(out_name,*grape_center_cloud);


/*	if (point_cloud_number==177)
	{   copyPointCloud(*grape_center_cloud, *prev_cloud);
		
		continue;
		
	}
	else
	{
		//slam
		slam.point_to_point(prev_cloud,grape_center_cloud);
		cout<<slam.global_transformation<<endl;

	}
 	
 	copyPointCloud(*grape_center_cloud, *prev_cloud);
 

*/
}
	return 0;
}





