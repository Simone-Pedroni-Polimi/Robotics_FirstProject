#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>

std::string param;
std_msgs::String local_msg1;
std_msgs::String local_msg2;
std::string local_input_odom;
std::string local_root_frame;
std::string local_child_frame;

class odom_to_tf{
public:
    odom_to_tf(ros::NodeHandle n){
        local_msg1.data = local_input_odom;
        sub = n.subscribe(local_msg1.data, 1000, &odom_to_tf::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        transform.setRotation(q);

        local_msg1.data = local_root_frame;
        local_msg2.data = local_child_frame;
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), local_msg1.data, local_msg2.data));
    }

private:
    tf::TransformBroadcaster br;
    ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle n;
    ros::NodeHandle nh_private("~");

    std::string input_odom = ros::this_node::getName() + "/input_odom";
    std::string root_frame = ros::this_node::getName() + "/root_frame";
    std::string child_frame = ros::this_node::getName() + "/child_frame";

    n.getParam(input_odom, local_input_odom);
    n.getParam(root_frame, local_root_frame);
    n.getParam(child_frame, local_child_frame);

    odom_to_tf my_odom_to_tf(n);
    ros::spin();
    return 0;
}