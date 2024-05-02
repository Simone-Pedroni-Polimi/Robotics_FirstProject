#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
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
    //covert in radiant
    double lat = msg->latitude*M_PI/180;
    double lon = msg->longitude*M_PI/180;
    double alt = msg->altitude*M_PI/180;

    //convert (latitude-longitude-altitude) to Cartesian ECEF
    double N = A/sqrt(1 - e*pow((sin(lat)), 2));
    double N_rel = A/sqrt(1 - e*pow((sin(lat_rel)), 2));

    double X = (N + alt)*cos(lat)*cos(lon);
    double Y = (N + alt)*cos(lat)*sin(lon);
    double Z = (N*(1-e) + alt)*sin(lat);

    double X_rel = (N_rel + alt_rel)*cos(lat_rel)*cos(lon_rel);
    double Y_rel = (N_rel + alt_rel)*cos(lat_rel)*sin(lon_rel);
    double Z_rel = (N_rel*(1-e) + alt_rel)*sin(lat_rel);

    //convert Cartesian ECEF to ENU
    double X_matrix = X - X_rel;
    double Y_matrix = Y - Y_rel;
    double Z_matrix = Z - Z_rel;

    double x = -sin(lon_rel)*X_matrix + cos(lon_rel)*Y_matrix;
    double y = -sin(lat_rel)*cos(lon_rel)*X_matrix - sin(lat_rel)*sin(lon_rel)*Y_matrix + cos(lat_rel)*Z_matrix;
    double z = cos(lon_rel)*cos(lat_rel)*X_matrix + cos(lat_rel)*sin(lon_rel)*Y_matrix + sin(lat_rel)*Z_matrix;

    //rotation of x e y of 135 degrees
    double alfa= atan2(y,x);
    double dist= sqrt(pow(x,2)+pow(y,2));
    x=dist*sin(alfa-135*M_PI/180);
    y=dist*cos(alfa-135*M_PI/180);

    double orientation = atan2((y-y_old),(x-x_old));

    //set old value
    x_old = x;
    y_old = y;
    z_old = z;

    //creation of odom message
    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;

    tf::Quaternion q;
    q.setRPY(0, 0, orientation);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

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