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
	string filename="/media/storagedrive/Thesis/Results/March18/filtered_177.ply";
	pcl::PLYReader reader;
	reader.read(filename,*cloud);
	double t = tmr.elapsed();
  	std::cout << "Reading ply: "<<t<<endl;
	



	pcl::PointCloud<pcl::PointXYZ>::Ptr mask_center_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr grape_center_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	string center_cloud_name="/media/storagedrive/Thesis/Results/March18/Amask_center_cloud_177.ply";
	reader.read(center_cloud_name,*mask_center_cloud);

   
	
	tmr.reset();
	pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ searchPoint;

	
	
	for(int i=0;i<429;i++){
		inFile>>searchPoint.x;//=center_cloud->points[i].x;
		inFile>>searchPoint.y;
		inFile>>searchPoint.z;

		
		roi_cloud=S.get_neighbors(0, 100,0.02,cloud,searchPoint);

		pcl::PointXYZ sphere_center=fs.fit_sphere(roi_cloud,0.001,0.02,true);
 		cout<<i<<": ";
 		mask_center_cloud->points[i]=sphere_center;
 		
	}
	inFile.close();
   
	std::cout << "Fit spheres: "<<tmr.elapsed()<<endl;
	
	int total_spheres=fs.count_fitted_spheres();
	cout<<total_spheres<<endl;

	pcl::PLYWriter w;

	string out_name="grape_center_cloud2.ply";
	w.write(out_name,*mask_center_cloud);
	

pcl::visualization::PCLVisualizer::Ptr viewer = v.simpleVis(mask_center_cloud);
	while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


	return 0;
}





