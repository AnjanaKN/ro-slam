#include "sphere_fitting.h"
#include "svd.h"
#include "slam.h"
#include "disparity.h"
#include "visualization.h"


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
	

    Timer tmr;
    custom_svd S;
    fit_shape fs;
    Visualisation v;

	ifstream inFile;
    inFile.open("/home/anjana/catkin_ws/centers.txt");
	
	
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
	string center_cloud_name="/media/storagedrive/Thesis/Results/March18/Amask_center_cloud_177.ply";
	string out_name="/home/anjana/catkin_ws/src/ro_slam/outputs/grape_center_cloud.ply";


	reader.read(filename,*cloud);
	double t = tmr.elapsed();
  	std::cout << "Reading ply: "<<t<<endl;
	
	reader.read(center_cloud_name,*mask_center_cloud);
	tmr.reset();


	
//Create kdtree

  	kdtree->setInputCloud(cloud);

	
	//Read roi centers
	for(int i=0;i<429;i++){
		inFile>>searchPoint.x;
		inFile>>searchPoint.y;
		inFile>>searchPoint.z;
		
		roi_cloud=S.get_neighbors(0, 100,0.02,kdtree,cloud,searchPoint);

		S.compute_cov_matrix(roi_cloud);
		S.compute_SVD(S.C);
		

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





