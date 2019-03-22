#include "headers.h"



class custom_svd{


//custom_svd(){}

public:
void compute_cov_matrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){


	Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid (*cloud, xyz_centroid);
    //Eigen::Matrix3f C;
    pcl::computeCovarianceMatrix (*cloud, xyz_centroid, C);
    
	//return C;
}


void compute_SVD( Eigen::Matrix3f covariance_matrix){

	Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
	//vector<double> a;
	a.clear();
	a.push_back(svd.singularValues()[0]);
  	a.push_back(svd.singularValues()[1]);
  	a.push_back(svd.singularValues()[2]);
	//cout<<a<<endl;

	//return a;
}


void write_singular_values(){

	std::ofstream myfile("/home/anjana/catkin_ws/src/ro_slam/outputs/singular_values.csv",std::ofstream::app);
    myfile << a[0]<<","<<a[1]<<','<<a[2]<<endl;
    myfile.close();
   
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_neighbors(int flag, int K,float radius,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ searchPoint){

 	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  	kdtree.setInputCloud (cloud);
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud ( new pcl::PointCloud<pcl::PointXYZ>);
    
    if (flag==0)
   
	  { //k search
	  		std::vector<int> pointIdxNKNSearch(K);
    		std::vector<float> pointNKNSquaredDistance(K);
	  	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	  	{
	  		pcl::copyPointCloud(*cloud,pointIdxNKNSearch,*sub_cloud);

	    /*for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
	      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
	                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
	                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
	                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
	  }
	}


  else//Radius search
  	{	std::vector<int> pointIdxRadiusSearch;
  		std::vector<float> pointRadiusSquaredDistance;
	  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	  { pcl::copyPointCloud(*cloud,pointIdxRadiusSearch,*sub_cloud);
	    /*for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
	      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
	                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
	                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
	                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;*/
	  	
	  }

 	}
 	
	return sub_cloud;
}


Eigen::Matrix3f C;
vector<double> a;
};




