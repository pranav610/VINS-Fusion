#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <string.h>

namespace stereo_cam
{

class ceres_test
{
private:
    ros::NodeHandle nh;
    std::string after_topic, before_topic, ground_topic; 
public:
    ceres_test(ros::NodeHandle &nodeHandle);
    virtual ~ceres_test();
    bool readParams();
    void run();
    void sync_subs(const nav_msgs::Odometry::ConstPtr &ground, const nav_msgs::Odometry::ConstPtr &before, const nav_msgs::Odometry::ConstPtr &after);
};

} // namespace stereo_cam