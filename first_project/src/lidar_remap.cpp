#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/odomConfig.h>
#include "sensor_msgs/PointCloud2.h"

bool odomFrame = true;
ros::Publisher odom_pub;

void callback(first_project::odomConfig &config) {
  odomFrame = config.odom_param;
  ROS_INFO("Reconfigure Request: %s", 
            odomFrame?"True":"False");
}


void subCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sensor_msgs::PointCloud2 lidar_msg;
    lidar_msg.header = msg->header;
    if(odomFrame)
        lidar_msg.header.frame_id = "wheel_odom";
    else lidar_msg.header.frame_id = "gps_odom";

    /*lidar_msg.height = msg->height;
    lidar_msg.width = msg->width;
    lidar_msg.fields = msg->fields;
    lidar_msg.is_bigendian = msg->is_bigendian;
    lidar_msg.point_step = msg->point_step;
    lidar_msg.row_step = msg->row_step;
    lidar_msg.data = msg->data;
    lidar_msg.is_dense = msg->is_dense;*/

    odom_pub.publish(lidar_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_remap");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1000, subCallback);
  odom_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1);

  dynamic_reconfigure::Server<first_project::odomConfig> server;
  dynamic_reconfigure::Server<first_project::odomConfig>::CallbackType f;

  f = boost::bind(&callback, _1);
  server.setCallback(f);

  //ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}