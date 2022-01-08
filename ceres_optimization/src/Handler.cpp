#include <ceres_optimization.hpp>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using namespace std;
using namespace cv;
#define RADIUS 20.0f

Handler::Handler(ros::NodeHandle &nodeHandle) : nh(nodeHandle), global_octree(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(128.0f))
{
    pub = nh.advertise<geometry_msgs::TransformStamped>("stereo_output", 10);
    global_octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(128.0f);
    merged = PointCloud::Ptr(new PointCloud);
    D_Camera_Proj_fn = Eigen::MatrixXf(3, 4);
    Cam_Proj = Eigen::MatrixXf(3, 4);
    if (!readParams())
    {
        ROS_INFO("Parameters were not read.");
        ros::requestShutdown();
    }
    else
    {
        ROS_INFO("Parameters readed .");
    }
}

Handler::~Handler()
{
}

bool Handler::readParams()
{
    string key;
    string tempKey;

    if (!ros::param::search("depth_topic_name", key))
        return false;
    ros::param::get(key, Handler::depth_topic);
    if (!ros::param::search("scharr_topic_name", key))
        return false;
    ros::param::get(key, Handler::scharr_topic);
    if (!ros::param::search("vins_topic_name", key))
        return false;
    ros::param::get(key, Handler::vins_topic);
    if (!ros::param::search("pcd_path", key))
        return false;
    ros::param::get(key, Handler::pcd_path);

    if (!ros::param::search("fx", key))
        return false;
    ros::param::get(key, tempKey);
    Handler::fx = atof(tempKey.c_str());

    if (!ros::param::search("fy", key))
        return false;
    ros::param::get(key, tempKey);
    Handler::fy = atof(tempKey.c_str());

    if (!ros::param::search("cx", key))
        return false;
    ros::param::get(key, tempKey);
    Handler::cx = atof(tempKey.c_str());

    if (!ros::param::search("cy", key))
        return false;
    ros::param::get(key, tempKey);
    Handler::cy = atof(tempKey.c_str());

    // if (!ros::param::search("s", key))
    //     return false;
    // ros::param::get(key, Handler::s);

    D_Camera_Proj_fn << fx, 0.0, -fx, 0.0,
        0.0, fy, -fy, 0.0,
        0.0, 0.0, 0.0, 1.0;
    Cam_Proj << fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(Handler::pcd_path, *(Handler::merged)) == -1) // load the file
    {
        ROS_INFO("Couldn't read file\n");
        return false;
    }

    Handler::global_octree.setInputCloud(Handler::merged);
    Handler::global_octree.addPointsFromInputCloud();
    return true;
}

void Handler::run()
{
    cout << depth_topic << endl;
    cout << vins_topic << endl;

    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 10);
    // message_filters::Subscriber<sensor_msgs::Image> scharr_sub(nh, scharr_topic, 10);

    message_filters::Subscriber<nav_msgs::Odometry> vins_sub(nh, vins_topic, 10);

    // Ref: https://stackoverflow.com/questions/62267850/ros-c-approximate-time-synchronizer-callback-not-working-in-node-member-class
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vins_sub, depth_sub);

    ROS_INFO("Subscribed topics now going into callback.\n");
    sync.registerCallback(boost::bind(&Handler::optimize, this, _1, _2));
    ros::spin();
}

void Handler::optimize(const nav_msgs::Odometry::ConstPtr &tf_msg,
                       const sensor_msgs::Image::ConstPtr &depth_img)
{
    ROS_INFO("Call back function is being called.\n");
    // qt = tf_msg->transform.rotation;
    // t = tf_msg->transform.translation;

    google::InitGoogleLogging("file_name");
    ceres::Problem problem;
    double params[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::Vector4f t, p;
    p(3) = 1;
    t(0) = tf_msg->pose.pose.position.x;
    t(1) = tf_msg->pose.pose.position.y;
    t(2) = tf_msg->pose.pose.position.z;
    t(3) = 1;

    Eigen::Quaternionf q;
    q.x() = tf_msg->pose.pose.orientation.x;
    q.y() = tf_msg->pose.pose.orientation.y;
    q.z() = tf_msg->pose.pose.orientation.z;
    q.w() = tf_msg->pose.pose.orientation.w;

    Eigen::Matrix3f R = q.normalized().toRotationMatrix();
    Eigen::Matrix4f _TCM = Eigen::Matrix4f::Zero();
    _TCM.block<3, 3>(0, 0) = R;
    _TCM.block<4, 1>(3, 0) = t; // Review

    cv_bridge::CvImagePtr cv_ptr1, cv_ptr2;
    cv_ptr1 = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::BGR8);
    // cv_ptr2 = cv_bridge::toCvCopy(scharr_img, sensor_msgs::image_encodings::BGR8);
    Scharr(cv_ptr1->image, cv_ptr2->image, -1, 1, 1);

    pcl::PointXYZ center(t(0), t(1), t(2));
    float radius = RADIUS;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    ROS_INFO("Imaged read and now going into ceres solver.\n");
    if (global_octree.radiusSearch(center, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            p(0) = (*merged)[pointIdxRadiusSearch[i]].x;
            p(1) = (*merged)[pointIdxRadiusSearch[i]].y;
            p(2) = (*merged)[pointIdxRadiusSearch[i]].z;
            ROS_INFO("%ld points added to ceres solver\n", i);
            if ((Cam_Proj * (_TCM * p))(0) >= 0 && (Cam_Proj * (_TCM * p))(1) >= 0 && (Cam_Proj * (_TCM * p))(0) <= cv_ptr1->image.rows && (Cam_Proj * (_TCM * p))(1) <= cv_ptr1->image.cols)
            {
                ceres::CostFunction *cost_func = new Localize(_TCM, p, cv_ptr1->image, cv_ptr2->image, D_Camera_Proj_fn, Cam_Proj);
                problem.AddResidualBlock(
                    cost_func,
                    nullptr,
                    params);
            }
        }
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ROS_INFO("Data loaded to the solver and solving is started\n");
        ceres::Solve(options, &problem, &summary);
        ROS_INFO("Solved gg.\n");
    }
    Eigen::Matrix4f teps = Eigen::Matrix4f::Identity();
    teps(0, 1) = -params[2];
    teps(1, 0) = params[2];
    teps(0, 2) = params[1];
    teps(2, 0) = -params[1];
    teps(1, 2) = -params[0];
    teps(2, 1) = params[0];
    teps(0, 3) = params[3];
    teps(1, 3) = params[4];
    teps(2, 3) = params[5];

    _TCM = teps * _TCM;
    cout << _TCM << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ceres_optimiser");
    ros::NodeHandle nh_ceres;

    Handler a(nh_ceres);
    a.run();

    return 0;
}