#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>

std::string param;
std_msgs::String local_msg;
std::string local_name_from_global;

class prova{
public:
  	prova(ros::NodeHandle n){
    local_msg.data = local_name_from_global;;
    //if(local_msg.data == "/odom")
  	    sub = n.subscribe(local_msg.data, 1000, &prova::callback, this);
        ROS_INFO("I heard: [%s]\n", local_msg.data.c_str());
}



void callback(const nav_msgs::Odometry::ConstPtr& msg){
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
        tf::Quaternion q;
        q.setRPY(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 0);
        transform.setRotation(q);
        if(local_msg.data == "/odom")
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wheel_odom"));
        else br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "gps_odom"));
    }

private:
  tf::TransformBroadcaster br;
  ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "prova");
    ros::NodeHandle n;
    ros::NodeHandle nh_private("~");
	std::string param_name = ros::this_node::getName() + "/input_odom"; 
	n.getParam(param_name, local_name_from_global);
    //nh_private.getParam("input_odom", param);
    prova my_prova(n);
    ros::spin();
    return 0;
}