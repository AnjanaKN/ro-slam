#include "headers.h"

using namespace std;

class fit_shape{

public:
	fit_shape(){
		fitted_spheres=0;
	}

pcl::PointXYZ fit_sphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float distance_threshold=0.01, float radius_guess=0.1, bool print_flag=false){

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
	vector<int> inliers;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (distance_threshold);
    ransac.computeModel();
    ransac.getInliers(inliers);
  	Eigen::VectorXf modCoeffs;
    if (print_flag){
	    cout<<"Total points: "<<cloud->points.size()<<"  Fitted points: "<<inliers.size()<<endl;
	    ransac.getModelCoefficients(modCoeffs);
	    cout<<"Sphere coefficients(r) "<<modCoeffs[3]<<endl;
	}

	pcl::PointXYZ center;;
	center.x=modCoeffs[0];
	center.y=modCoeffs[1];
    center.z=modCoeffs[2];	
    fitted_spheres++;
	return center;//inliers;
}


/*pcl::PointCloud<pcl::PointXYZ>::Ptr extract_points_from_roi(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, roi){



	return roi_cloud;
}*/



pcl::PointCloud<pcl::PointXYZ>::Ptr delete_fitted_points(vector<int> inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final ( new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr< vector<int> > indicesPtr(new vector<int>(inliers));
	pcl::ExtractIndices<pcl::PointXYZ> extract(true);
	extract.setInputCloud (cloud);
    extract.setIndices (indicesPtr);
    extract.setNegative (true);
    extract.filter (*cloud_final);

	return cloud_final;

}



int count_fitted_spheres(){

	return fitted_spheres;
}


vector<int> select_random_roi(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	vector<int> random_roi_indices;
	return random_roi_indices;
}


//sphere_viewer(){






private: 
	int fitted_spheres;


};