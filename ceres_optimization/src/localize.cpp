#include "ceres/ceres.h"
#include "glog/logging.h"
#include <iostream>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/registration/icp.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <string.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using namespace std;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//[future work] remove global wariables
Eigen::MatrixXd D_Camera_Proj_fn(3, 4);
Eigen::MatrixXd Cam_Proj(3, 4);

//subscriber class to subscribe TCM(from VINS), depth,scharr image(opencv::mat), point cloud(pcl)

//[future work] add this class inside some function 
class Localize : public ceres::SizedCostFunction<1, 6>
{
public:
    Localize(Eigen::Matrix4f &TCM, PointCloud::Ptr local, Eigen::Vector4f p, Mat &disparity, Mat &scharred)
    {
        TCM_ = TCM;
        p_ = p;
        disparity_ = disparity;
        scharred_ = scharred;
    }
    virtual ~Localize() {}
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {   
        Eigen::Matrix4f Teps_;
        Eigen::MatrixXd depthJac(1, 3);
        Teps_ << 1.0             , -parameters[0][2], parameters[0][1] , parameters[0][3],
                 parameters[0][2], 1.0              , -parameters[0][0], parameters[0][4],
                 -parameters[0][1], parameters[0][0], 1.0              , parameters[0][5],
                 0.0              ,0.0              ,0.0               ,              1.0;

        double sigma = sqrt(pow(scharred_.at<double>((Cam_Proj*(Teps_*(TCM_*p_)))(0),(Cam_Proj*(Teps_*(TCM_*p_)))(1),0),2)+pow(scharred_.at<double>((Cam_Proj*(Teps_*(TCM_*p_)))(0),(Cam_Proj*(Teps_*(TCM_*p_)))(1),1),2));
        residuals[0] = ( ((Teps_*(TCM_*p_))[3]) - disparity_.at<double>((Cam_Proj*(Teps_*(TCM_*p_)))(0),(Cam_Proj*(Teps_*(TCM_*p_)))(1)) ) / sigma;
        if (!jacobians)
			return true;
		double *jacobian = jacobians[0];
		if (!jacobian)
			return true;

        double x = (Teps_*(TCM_*p_))(0,0);
        double y = (Teps_*(TCM_*p_))(0,1);
        double z = (Teps_*(TCM_*p_))(0,2);


        D_Camera_Proj_fn(0,0) /= z;
        D_Camera_Proj_fn(1,1) /= z;
        D_Camera_Proj_fn(0,2) /= (z*z);
        D_Camera_Proj_fn(1,2) /= (z*z);

        Eigen::Matrix4f temp = Eigen::Matrix4f::Zero(); 
        temp(1,2) = -1.0;
        temp(2,1) = 1.0;
        jacobian[0] = (temp*(TCM_*p_))[3] - ((depthJac)*(D_Camera_Proj_fn)*(temp*(TCM_*p_)))[0];

        temp = Eigen::Matrix4f::Zero();
        temp(0,2) = 1.0;
        temp(2,0) = 1.0;
        jacobian[1] = (temp*(TCM_*p_))[3] - ((depthJac)*(D_Camera_Proj_fn)*(temp*(TCM_*p_)))[0];

        temp = Eigen::Matrix4f::Zero();
        temp(0,1) = -1.0;
        temp(1,0) = 1.0;
		jacobian[2] = (temp*(TCM_*p_))[3] - ((depthJac)*(D_Camera_Proj_fn)*(temp*(TCM_*p_)))[0];

        temp = Eigen::Matrix4f::Zero();
        temp(0,3) = 1.0;
		jacobian[3] = (temp*(TCM_*p_))[3] - ((depthJac)*(D_Camera_Proj_fn)*(temp*(TCM_*p_)))[0];
        
        temp = Eigen::Matrix4f::Zero();
        temp(1,3) = 1.0;
		jacobian[4] = (temp*(TCM_*p_))[3] - ((depthJac)*(D_Camera_Proj_fn)*(temp*(TCM_*p_)))[0];

        temp = Eigen::Matrix4f::Zero();
        temp(2,3) = 1.0;
		jacobian[5] = (temp*(TCM_*p_))[3] - ((depthJac)*(D_Camera_Proj_fn)*(temp*(TCM_*p_)))[0];

		return true;
    }

private:
    Eigen::Matrix4f TCM_;
    Eigen::Vector4f p_;
    Mat disparity_;
    Mat scharred_;
};

//[future work]add functions for subscribing dpeth image and vins output
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "localize_node");
    ros::NodeHandle nh_localize_node;
    double fx, fy, s, cx, cy;
    bool param_success = true;
    param_success &= nh_localize_node.getParam("/fx", fx);
    param_success &= nh_localize_node.getParam("/fy", fy);
    param_success &= nh_localize_node.getParam("/s",s);
    param_success &= nh_localize_node.getParam("/cx",cx);
    param_success &= nh_localize_node.getParam("/cy",cy);
    if (param_success)
    {
        std::cout << "Parameters Loaded Successfully";
    }
    D_Camera_Proj_fn << fx, 0.0, -fx, 0.0,
                        0.0, fy, -fy, 0.0,
                        0.0, 0.0, 0.0, 1.0;
    Cam_Proj << fx, s, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0;
    return 0;
}