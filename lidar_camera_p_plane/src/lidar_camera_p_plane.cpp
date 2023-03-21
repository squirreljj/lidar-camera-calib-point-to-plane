#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include  "rotation.h"
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud1;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace std;
using namespace cv;
using namespace pcl;
cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
cv::Mat rvec_l_c = cv::Mat::zeros(3, 1, CV_64F);
cv::Mat tvec_l_c = cv::Mat::zeros(3, 1, CV_64F);
double rx=0;
double ry=0;
double rz=0;
double tx=0;
double ty=0;
double tz=0;
struct CostFunctor {
CostFunctor(double x,double y,double z,double r1,double r2,double r3,double t1,double t2,double t3):_x(x),_y(y),_z(z),_r1(r1),_r2(r2),_r3(r3),_t1(t1),_t2(t2),_t3(t3){}
template<typename T>
bool operator()(const T* const  rx, const T* const ry, const T* const rz, const T* const tx, const T* const ty, const T* const tz, T* residual)  const
{
    T point_r[3]; // 雷达坐标
    point_r[0] = T(_x);
    point_r[1] = T(_y);
    point_r[2] = T(_z);
    T point_c[3]; // 相机坐标系下的坐标
    T r[3];
    r[0] = *rx;
    r[1] = *ry;
    r[2] = *rz;
    AngleAxisRotatePoint(r, point_r, point_c);
    
    // 平移
    point_c[0] += *tx;
    point_c[1] += *ty;
    point_c[2] += *tz;
    T point_b[3];
    
    T r_[3];
    r_[0] = T(_r1);
    r_[1] = T(_r2);
    r_[2] = T(_r3);
    AngleAxisRotatePoint(r_, point_c, point_b);
    // 平移
    point_b[0] += T(_t1);
    point_b[1] += T(_t2);
    point_b[2] += T(_t3);
    
    // 计算点到面点residual
    residual[0] = sqrt(point_b[2]*point_b[2]);
    return true;
  }
  const double _x,_y,_z,_r1,_r2,_r3,_t1,_t2,_t3;
   /* T point_r[3]; // 雷达坐标
    point_r[0] = T(_x);
    point_r[1] = T(_y);
    point_r[2] = T(_z);
    T point_c[3]; // 相机坐标系下的坐标
    T r[3];
    r[0] = *rx;
    r[1] = *ry;
    r[2] = *rz;
    cv::Mat R_1;
    cv::Rodrigues(rvec, R_1);
    
    // 平移
    point_c[0] = point_r[0]*R_1.at<double>(0, 0)+point_r[1]*R_1.at<double>(0, 1)+point_r[2]*R_1.at<double>(0, 2)+*tx;
    point_c[1] = point_r[0]*R_1.at<double>(1, 0)+point_r[1]*R_1.at<double>(1, 1)+point_r[2]*R_1.at<double>(1, 2)+*ty;
    point_c[2] = point_r[0]*R_1.at<double>(2, 0)+point_r[1]*R_1.at<double>(2, 1)+point_r[2]*R_1.at<double>(2, 2)+*tz;
    T point_b[3];
    
    T r_[3];
    r_[0] = T(_r1);
    r_[1] = T(_r2);
    r_[2] = T(_r3);

  

    cv::Mat R_2;
    
    cv::Mat R_inv;
    cv::Rodrigues(rvec, R_2);
    cv::Mat R_3;
    R_3.at<double>(0, 0)=R_2.at<double>(0, 0);  R_3.at<double>(0, 1)=R_2.at<double>(0, 1);  R_3.at<double>(0, 2)=R_2.at<double>(0, 2);  R_3.at<double>(0, 3)=T(_t1);
    R_3.at<double>(1, 0)=R_2.at<double>(1, 0);  R_3.at<double>(1, 1)=R_2.at<double>(1, 1);  R_3.at<double>(1, 2)=R_2.at<double>(1, 2);  R_3.at<double>(1, 3)=T(_t2);
    R_3.at<double>(2, 0)=R_2.at<double>(2, 0);  R_3.at<double>(2, 1)=R_2.at<double>(2, 1);  R_3.at<double>(2, 2)=R_2.at<double>(2, 2);  R_3.at<double>(2, 3)=T(_t3);
    R_3.at<double>(3, 0)=0;                     R_3.at<double>(3, 1)=0;                     R_3.at<double>(3, 2)=0;                     R_3.at<double>(3, 3)=1;
    cv::invert(R_3, R_inv, cv::DECOMP_SVD);
    // 平移
    point_b[0] =point_c[0]*R_inv.at<double>(0, 0) + point_c[1]*R_inv.at<double>(0, 1) + point_c[2]*R_inv.at<double>(0, 2) + R_inv.at<double>(0, 3);
    point_b[1] =point_c[0]*R_inv.at<double>(0, 0) + point_c[1]*R_inv.at<double>(0, 1) + point_c[2]*R_inv.at<double>(0, 2) + R_inv.at<double>(1, 3);
    point_b[2] =point_c[0]*R_inv.at<double>(0, 0) + point_c[1]*R_inv.at<double>(0, 1) + point_c[2]*R_inv.at<double>(0, 2) + R_inv.at<double>(2, 3);
    
    // 计算点到面点residual
    residual[0] = point_b[2];
    return true;
  }
  const double _x,_y,_z,_r1,_r2,_r3,_t1,_t2,_t3;*/
};

int main(int argc, char** argv) {  
    //1.载入内参
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);

    // Define distCoeffs as a 5-element double-precision floating-point vector
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_32FC1);
    
    FileStorage f_calib;
    f_calib.open("/home/ljj/work/lidar_camera_p_plane/config/calib.yaml", FileStorage::READ);
    if(!f_calib.isOpened()){
        cout << "Cannot open f_calib" << endl;
        return -1;
    }
    f_calib["intrinsics"] >> cameraMatrix;
    double nums,banzi_row,banzi_col,squareSize;
    f_calib["nums"] >> nums;
    f_calib["banzi_row"] >> banzi_row;
    f_calib["banzi_col"] >> banzi_col;
    f_calib["squareSize"] >> squareSize;
    f_calib["distortion_coeffs"] >> distCoeffs;
    google::InitGoogleLogging(argv[0]);
    // 构建优化问题
    ceres::Problem problem;
    for(int i=0;i<nums;i++){
	//2.检测marker
	// 准备棋盘格
        cv::Size boardSize(banzi_row,banzi_col); // 棋盘格每行和每列的角点数
        //float squareSize = 0.050f; // 棋盘格方格的物理尺寸（单位：米）

        // 读取图像和角点坐标
        std::vector<cv::Point3f> objectPoints; // 棋盘格的三维坐标
        std::vector<cv::Point2f> imagePoints; // 棋盘格的图像坐标
        cv::Mat image;
	cv::Mat img;
        image=cv::imread("/home/ljj/work/lidar_camera_p_plane/img/"+std::to_string(i+1)+".png");
    	img=cv::imread("/home/ljj/work/lidar_camera_p_plane/img/"+std::to_string(i+1)+".png");
        // 检测角点

        bool found = cv::findChessboardCorners(image, boardSize, imagePoints);

        if (found) {
            // 绘制角点
            cv::drawChessboardCorners(image, boardSize, imagePoints, true);
            // 计算棋盘格的三维坐标
            for (int i = 0; i < boardSize.height; i++) {
                for (int j = 0; j < boardSize.width; j++) {
		    objectPoints.emplace_back(cv::Point3f(j * squareSize, i * squareSize, 0));
                }
            }
        }
        //cout<<found<<endl;
        // 显示图像
        cv::imshow("image", image);
        cv::waitKey(0);
        // 计算相机姿态
	//cv::calibrateCamera(objectPoints, imagePoints, image.size(), cameraMatrix, distCoeffs, rvec, tvec);
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
	
	cv::Mat R;
	cv::Rodrigues(rvec, R);
	cv::Mat R_inv;
	cv::invert(R, R_inv, cv::DECOMP_SVD);
	cv::Mat rotation_vector;
	cv::Mat trans_vector;
	cv::Rodrigues(R_inv,rotation_vector);
	trans_vector=-R_inv*tvec;
	// 将3D点投影到图像上
        /*std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
	cout<<distCoeffs<<endl;
        for (const auto& p : projectedPoints)
        {
            cv::circle(img, p, 3, cv::Scalar(0, 255, 0), cv::FILLED);
        }
        cv::imshow("Projected Points", img);
        cv::waitKey(0);*/
        for(int i=0;i<objectPoints.size();i++){
		double x_b,y_b,z_b;
	    	x_b=objectPoints[i].x*R.at<double>(0, 0)+objectPoints[i].y*R.at<double>(0, 1)+objectPoints[i].z*R.at<double>(0, 2)+tvec.at<double>(0, 0);
		y_b=objectPoints[i].x*R.at<double>(1, 0)+objectPoints[i].y*R.at<double>(1, 1)+objectPoints[i].z*R.at<double>(1, 2)+tvec.at<double>(1, 0);
		z_b=objectPoints[i].x*R.at<double>(2, 0)+objectPoints[i].y*R.at<double>(2, 1)+objectPoints[i].z*R.at<double>(2, 2)+tvec.at<double>(2, 0);
		double u,v;
		
		u=x_b*cameraMatrix.at<double>(0, 0)/z_b+cameraMatrix.at<double>(0, 2);
		v=y_b*cameraMatrix.at<double>(1, 1)/z_b+cameraMatrix.at<double>(1, 2);
		//cout<<cameraMatrix.at<double>(0, 2)<< " "<<cameraMatrix.at<double>(1, 2) <<endl;
		if(u<0||u>=img.rows||v<0||v>=img.cols)
			continue;
		cv::circle(img, cv::Point2f(u, v),3,cv::Scalar(0, 0, 255), 1); 
	
	}
	cv::imshow("img", img);
        cv::waitKey(0);
        //cout<<rvec<<endl;
	//cout<<tvec<<endl;
	//4.读入点云
	PointCloud1::Ptr cloud_c1(new PointCloud1);//保存彩色相机点云
	//add code
	//全局点云PCD路径
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ljj/work/lidar_camera_p_plane/pcd_filter/"+std::to_string(i+1)+".pcd", *cloud_c1) == -1) //* 加载点云数据路径 
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		system("PAUSE");
	}
	    // 设置优化变量的初值
	    // 5.定义雷达与相机之间的旋转向量
	    
	    // 6.添加误差项
	    for (size_t i = 0; i < cloud_c1->points.size(); i++) {
		double x = cloud_c1->points[i].x;
		double y = cloud_c1->points[i].y;
		double z = cloud_c1->points[i].z;
		
		double r1=rotation_vector.at<double>(0, 0);double r2=rotation_vector.at<double>(1, 0);double r3=rotation_vector.at<double>(2, 0);
		double t1=trans_vector.at<double>(0, 0);double t2=trans_vector.at<double>(1, 0);double t3=trans_vector.at<double>(2, 0);
		//cout<<r1<<" "<<r2<<" "<<r3<<" "<<t1<<" "<<t2<<" "<<t3<<" "<<endl;
		ceres::CostFunction* cost_function =new ceres::AutoDiffCostFunction<CostFunctor,1, 1,1,1,1,1,1>(new CostFunctor(x, y, z,r1,r2,r3,t1,t2,t3));
		problem.AddResidualBlock(cost_function, nullptr, &rx, &ry, &rz , &tx , &ty , &tz );
	    }
	    
	    
    }
    // 运行优化器
    // 配置优化器参数
    cout<<"start optimze"<<endl;
    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 打印优化结果
    //std::cout << summary.FullReport() << std::endl;
    std::cout << "rx = " << rx << ", ry = " << ry << ",rz = " << rz << ", tx = " << tx << ",ty = " << ty << ", tz = " << tz << std::endl;
    

    

}

