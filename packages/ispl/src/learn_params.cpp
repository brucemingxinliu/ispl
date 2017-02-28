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

void cloudCallback(const PointCloud::ConstPtr& cloud_holder)
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

    /*if(!nh_ptr->getParam("/motor_wobble/min_ang", min_ang))
    {
        ROS_WARN("FAILED TO GET WOBBLER MIN ANG");
    }
    */
    ros::Subscriber sub = nh.subscribe("scan_cloud", 1, cloudCallback);

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud", 1);
    pubCloud_ptr = &pubCloud;

    ros::spin();
    return 0;
}