#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf/transform_broadcaster.h>

std::string param;
tf::TransformBroadcaster br;
    std_msgs::String local_msg;
    tf::Transform transform;

    
/*
class odom_to_tf{

public:

    odom_to_tf(){
        n.getParam("/input_odom", param);
        sub = n.subscribe(param->data, 1000, &odom_to_tf::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
        tf::Quaternion q;
        q.setRPY(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 0);
        transform.setRotation(q);
        if(param.data.c_str() == "/odom")
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wheel_odom"));
        else br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "gps_odom"));
    }

private:
    ros::NodeHandle n;
    tf::TransformBroadcaster br;
    ros::Subscriber sub;
};

*/


void callback(const nav_msgs::Odometry::ConstPtr& msg){
        /*transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
        tf::Quaternion q;
        q.setRPY(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 0);
        transform.setRotation(q);
        if(local_msg.data == "/odom")
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wheel_odom"));
        else br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "gps_odom"));
    */}


int main(int argc, char **argv){
    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle n;
    ros::Subscriber sub;
    //odom_to_tf my_odom_to_tf;

    n.getParam("/input_odom", param);
    local_msg.data = param;
    sub = n.subscribe(local_msg.data, 1000, callback);


    ros::spin();

    return 0;
}