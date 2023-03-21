#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <algorithm>
#include <random>
#include <chrono>
#include <dirent.h>
#include <ctime>
#include <boost/filesystem.hpp>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;
using namespace pcl;
void GetFileNames(string path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            //filenames.push_back(path + "/" + ptr->d_name);
	    filenames.push_back(ptr->d_name);
    }
    }
    closedir(pDir);
}
int main(int argc, char** argv)
{
	 
    
	// 读取参数
	
	FileStorage f_calib;
        f_calib.open("./calib.yaml", FileStorage::READ);
    	if(!f_calib.isOpened()){
            cout << "Cannot open f_calib" << endl;
            return -1;
    	}
	std::string input_path,output_path;
	double x_max,x_min,y_max,y_min,z_max,z_min,ransac_threshold;
	f_calib["input_path"] >> input_path;
    	f_calib["output_path"] >> output_path;
   	f_calib["x_max"] >> x_max;
    	f_calib["x_min"] >> x_min;
	f_calib["y_max"] >> y_max;
    	f_calib["y_min"] >> y_min;
	f_calib["z_max"] >> z_max;
    	f_calib["z_min"] >> z_min;
	f_calib["ransac_threshold"] >> ransac_threshold;
    	//std::string input_path="/home/ljj/work/lidar_camera_p_plane/pcd";
	//std::string output_path="/home/ljj/work/lidar_camera_p_plane/pcd_filter/";
	//double x_max=0.8;double x_min=-0.8;double y_max=2;double y_min=-2;double z_max=2;double z_min=-2;double ransac_threshold=0.01;
	vector<string> filenames;
	GetFileNames(input_path,filenames);
	for(int i=0;i<filenames.size();i++)
	{
		cout<<filenames[i]<<endl;
		// 创建一个PointCloud对象
    		pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    		// 从文件中读取点云数据
		cout<<input_path+"/"+filenames[i]<<endl;
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path+"/"+filenames[i], *in_cloud) == -1) // 加载失败
		{
			PCL_ERROR("Couldn't read file input_cloud.pcd \n");
			return (-1);
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ >::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ >::Ptr plane_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		
		/// Pass through filters
		pcl::PassThrough<pcl::PointXYZ> pass_x;
		pass_x.setInputCloud(in_cloud);
		pass_x.setFilterFieldName("x");
		pass_x.setFilterLimits(x_min, x_max);
		pass_x.filter(*cloud_filtered_x);

		pcl::PassThrough<pcl::PointXYZ> pass_y;
		pass_y.setInputCloud(cloud_filtered_x);
		pass_y.setFilterFieldName("y");
		pass_y.setFilterLimits(y_min, y_max);
		pass_y.filter(*cloud_filtered_y);

		pcl::PassThrough<pcl::PointXYZ> pass_z;
		pass_z.setInputCloud(cloud_filtered_y);
		pass_z.setFilterFieldName("z");
		pass_z.setFilterLimits(z_min, z_max);
		pass_z.filter(*cloud_filtered_z);
		pcl::io::savePCDFileASCII(output_path+filenames[i], *cloud_filtered_z); // 保存为ASCII格式的PCD文件
		/// Plane Segmentation
		
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
		        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_filtered_z));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(ransac_threshold);
		ransac.computeModel();
		std::vector<int> inliers_indicies;
		ransac.getInliers(inliers_indicies);
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud_filtered_z, inliers_indicies, *plane);

		/// Statistical Outlier Removal
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(plane);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1);
		sor.filter (*plane_filtered);
		pcl::io::savePCDFileASCII(output_path+filenames[i], *plane_filtered); // 保存为ASCII格式的PCD文件
		
		
	}    
	
}
