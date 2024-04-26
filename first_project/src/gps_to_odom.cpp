#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <cmath>

const double A = 6378137;
const double B = 6356752;
const double e = 1 - pow(B, 2)/pow(A, 2);

double lat_par;
double lon_par;

double lat_rel;
double lon_rel;
double alt_rel;

double x_old = 0;
double y_old = 0;
double z_old = 0;

ros::Publisher odom_pub;

void pubCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    double lat = msg->latitude*M_PI/180;
    double lon = msg->longitude*M_PI/180;
    double alt = msg->altitude*M_PI/180;

    double N = A/sqrt(1 - e*pow((sin(lat)), 2));
    double N_rel = A/sqrt(1 - e*pow((sin(lat_rel)), 2));

    double X = (N + alt)*cos(lat)*cos(lon);
    double Y = (N + alt)*cos(lat)*sin(lon);
    double Z = (N*(1-e) + alt)*sin(lat);

    double X_rel = (N_rel + alt_rel)*cos(lat_rel)*cos(lon_rel);
    double Y_rel = (N_rel + alt_rel)*cos(lat_rel)*sin(lon_rel);
    double Z_rel = (N_rel*(1-e) + alt_rel)*sin(lat_rel);

    double X_matrix = X - X_rel;
    double Y_matrix = Y - Y_rel;
    double Z_matrix = Z - Z_rel;

    double x = -sin(lon_rel)*X_matrix + cos(lon_rel)*Y_matrix;
    double y = -sin(lat_rel)*cos(lon_rel)*X_matrix - sin(lat_rel)*sin(lon_rel)*Y_matrix + cos(lat_rel)*Z_matrix;
    double z = cos(lon_rel)*cos(lat_rel)*X_matrix + cos(lat_rel)*sin(lon_rel)*Y_matrix + sin(lat_rel)*Z_matrix; 

    double dist_xy = sqrt(pow(x - x_old, 2) + pow(y - y_old, 2));
    double dist_xz = sqrt(pow(x - x_old, 2) + pow(z - z_old, 2));
    double dist_yz = sqrt(pow(y - y_old, 2) + pow(z - z_old, 2));

    // Angle between x and y wrt x axis
    double angle_xy = asin((y - y_old)/dist_xy);
    // Angle between x and z wrt x axis
    double angle_xz = asin((z - z_old)/dist_xz);
    // Angle between y and z wrt x axis
    //double angle_yz = arcsin((x - x_old)/dist_yz);

    //ROS_INFO("I heard angle_xy: [%lf]\n", angle_xy);
    //ROS_INFO("I heard angle_xz: [%lf]\n\n", angle_xz);

    x_old = x;
    y_old = y;
    z_old = z;

    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;
    odom_msg.pose.pose.orientation.x = angle_xy;
    odom_msg.pose.pose.orientation.y = angle_xz;

    ROS_INFO("I heard x: [%lf]\n", x);
    ROS_INFO("I heard odom x: [%lf]\n", odom_msg.pose.pose.position.x);
    ROS_INFO("I heard angle_xy: [%lf]\n", angle_xy);
    ROS_INFO("I heard odom angle_xy: [%lf]\n\n", odom_msg.pose.pose.orientation.x);

    odom_pub.publish(odom_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle n;

    n.getParam("/lat_r", lat_par);
    n.getParam("/lon_r", lon_par);
    n.getParam("/alt_r", alt_rel);
    lat_rel = lat_par*M_PI/180;
    lon_rel = lon_par*M_PI/180;
    ros::Subscriber sub = n.subscribe("/fix", 1000, pubCallback);

    odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1);

    ros::spin();

    return 0;
}