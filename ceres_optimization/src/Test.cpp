#include "test.hpp"

namespace stereo_cam
{

void return_diff(const nav_msgs::Odometry::ConstPtr &a, const nav_msgs::Odometry::ConstPtr &b, Eigen::Vector3f &t, Eigen::Vector4f &o)
{
    t(0) = a->pose.pose.position.x - b->pose.pose.position.x;
    t(1) = a->pose.pose.position.y - b->pose.pose.position.y;
    t(2) = a->pose.pose.position.z - b->pose.pose.position.z;

    o(0) = a->pose.pose.orientation.x - b->pose.pose.orientation.x;
    o(1) = a->pose.pose.orientation.y - b->pose.pose.orientation.y;
    o(2) = a->pose.pose.orientation.z - b->pose.pose.orientation.z;
    o(3) = a->pose.pose.orientation.w - b->pose.pose.orientation.w;
}

ceres_test::ceres_test(ros::NodeHandle &nodeHandle) :nh(nodeHandle)
{
}

ceres_test::~ceres_test()
{
}

bool ceres_test::readParams()
{
    std::string key;
    std::string tempKey;

    if (!ros::param::search("output_topic_name", key))
        return false;
    ros::param::get(key, ceres_test::after_topic);

    if (!ros::param::search("vins_topic", key))
        return false;
    ros::param::get(key, ceres_test::before_topic);

    if (!ros::param::search("ground_truth_topic", key))
        return false;
    ros::param::get(key, ceres_test::ground_topic);

    return true;
}

void ceres_test::run()
{
    message_filters::Subscriber<nav_msgs::Odometry> before_sub(nh, before_topic, 10);
    message_filters::Subscriber<nav_msgs::Odometry> after_sub(nh, after_topic, 10);
    message_filters::Subscriber<nav_msgs::Odometry> ground_sub(nh, ground_topic, 30);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ground_sub, before_sub, after_sub);

    sync.registerCallback(boost::bind(&ceres_test::sync_subs, this, _1, _2, _3));
    ros::spin();
}

void ceres_test::sync_subs(const nav_msgs::Odometry::ConstPtr &ground, const nav_msgs::Odometry::ConstPtr &before, const nav_msgs::Odometry::ConstPtr &after)
{
    Eigen::Vector3f pose_error;
    Eigen::Vector4f oren_error;
    return_diff(ground, before, pose_error, oren_error);
    ROS_INFO_STREAM("Error before stereo: \nPose Error:\n"<< pose_error.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]")) << "\n\nOrientation Error:\n" << oren_error.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]")));
    return_diff(ground, before, pose_error, oren_error);
    ROS_INFO_STREAM("-----------------------------------------------\n");
    ROS_INFO_STREAM("Error after stereo: \nPose Error:\n"<< pose_error.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]")) << "\n\nOrientation Error:\n" << oren_error.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]")));
    ROS_INFO_STREAM("===============================================\n\n");
}
} // namespace stereo_cam

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ceres_test");
    ros::NodeHandle nh_test;

    stereo_cam::ceres_test A(nh_test);
    A.run();
    return 0;
}