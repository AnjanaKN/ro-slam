#include "headers.h"

using namespace std;

class icp_slam{

public:
//Initializes the global cloud with inital cloud
icp_slam(const pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud){
			grape_count=0;
			pcl::copyPointCloud(*initial_cloud,*global_grape_cloud);
			all_camera_positions[0]=Eigen::Matrix4f::Identity();
	}


void point_to_point(const pcl::PointCloud<pcl::PointXYZ>::Ptr fixed, const pcl::PointCloud<pcl::PointXYZ>::Ptr moving){

	pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(moving);
    icp.setInputTarget(fixed);
    pcl::PointCloud<pcl::PointXYZ> Final;
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (0.05);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (50);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (1);
	// Perform the alignment
	icp.align(*registered_cloud);
	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	local_transformation = icp.getFinalTransformation ();
	global_transformation=local_transformation*global_transformation;
}




void point_to_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr tgt, const pcl::PointCloud<pcl::PointXYZ>::Ptr src){



        pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef pcl::PointNormal PointNormalT;
        typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

      
        // Compute surface normals and curvature
        PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

        pcl::NormalEstimation<PointT, PointNormalT> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        norm_est.setSearchMethod (tree);
        norm_est.setKSearch (30);

        norm_est.setInputCloud (src);
        norm_est.compute (*points_with_normals_src);
        pcl::copyPointCloud (*src, *points_with_normals_src);

        norm_est.setInputCloud (tgt);
        norm_est.compute (*points_with_normals_tgt);
        pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
        cout<<"Normals done"<<endl;
  

        pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
        reg.setTransformationEpsilon (1e-9);
        reg.setMaxCorrespondenceDistance (0.03);
     

        reg.setInputSource (points_with_normals_src);
        reg.setInputTarget (points_with_normals_tgt);
        reg.setMaximumIterations (15);
        PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
        reg.align (*reg_result);
        local_transformation = reg.getFinalTransformation();
        global_transformation=local_transformation*global_transformation;
        


}




int count_grapes(){

	grape_count=global_grape_cloud->points.size();

	return grape_count;
}




float calculate_error(){
 	float error=0.0;
	return error;
}




void output_transformations(){


    cout<<local_transformation;
    cout<<global_transformation;
	
}




pcl::PointCloud<pcl::PointXYZ>::Ptr output_global_grape_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr src){

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*src, *temp, global_transformation);
    *global_grape_cloud += *temp;
    return global_grape_cloud;
}




Eigen::Matrix4f output_camera_positions(){

	Eigen::Matrix4f camera_pos;
	camera_pos=global_transformation;
	all_camera_positions.push_back(camera_pos);

	return camera_pos;
}



//Read pointcloud from .ply file
void read_cloud(std::string filename, pcl::PCLPointCloud2 cloud){

	pcl::PLYReader reader;
	reader.read(filename,cloud);
	if (reader.read (filename, cloud) < 0)
    	cout<<"error reading file"<<endl;
	//print_info ("[done, "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
	//print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

}



//Write pointcloud into a .ply file
/*void write_cloud_to_file(const std::string &filename, pcl::PCLPointCloud2 & write_cloud){

	pcl::PLYWriter writer;
	writer.write(filename,write_cloud);
}*/




pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ( new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::VoxelGrid<pcl::PointXYZ> sor;
	  sor.setInputCloud (cloud);
	  sor.setLeafSize (0.01f, 0.01f, 0.01f);
	  sor.filter (*cloud_filtered);

	 return cloud_filtered;
}



private:
	int grape_count;
	Eigen::Matrix4f local_transformation;
	Eigen::Matrix4f global_transformation;
	vector<Eigen::Matrix4f> all_camera_positions;
	pcl::PointCloud<pcl::PointXYZ>::Ptr global_grape_cloud;// ( new pcl::PointCloud<pcl::PointXYZ>); 
    float error;

};