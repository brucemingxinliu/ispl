#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define PI 3.14159265

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::NodeHandle * nh_ptr;
pcl::PointCloud<pcl::PointXYZ> cloud;
ros::Publisher * pubCloud_ptr;

void cloudCB(const PointCloud::ConstPtr& cloud_holder)
{

}

int learn_intrinsic_parameters()
{



	return 0;
}


 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"learn_intrinsic_parameters");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    ros::Subscriber point_cloud_sub = nh.subscribe("/ispl/point_cloud", 1, cloudCB);

    learn_intrinsic_parameters();

    // Create sensor params, but should come from the algorithm 
    float x_dist = 1.55;
    float y_dist = 5.62;
    float z_dist = -0.66;
    float roll = 0.05;
    float pitch = 1.15;
    float yaw = -0.1511;

    // Publish found parameters
    nh_ptr->setParam("/ispl/x_dist", x_dist);
    nh_ptr->setParam("/ispl/y_dist", y_dist);
    nh_ptr->setParam("/ispl/z_dist", z_dist);
    nh_ptr->setParam("/ispl/roll", roll);
    nh_ptr->setParam("/ispl/pitch", pitch);
    nh_ptr->setParam("/ispl/yaw", yaw);

    nh_ptr->setParam("/calibration_done", true);
    
    ros::spin();
    return 0;
}