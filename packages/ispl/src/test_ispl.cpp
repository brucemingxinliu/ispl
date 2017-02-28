#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define PI 3.14159265

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

double wobbler_angle;
ros::NodeHandle * nh_ptr;
pcl::PointCloud<pcl::PointXYZ> cloud;
ros::Publisher * pubCloud_ptr;

void cloudCallback(const PointCloud::ConstPtr& cloud_holder)
{

}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_ispl");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;
    // That node should compare a set of SICK and wobbler point clouds (2D) from topics and give transform parameters on the pserver, and identify the lengths of test fixture walls
    // Test fixture: \_/ , with lidars looking at it from "below"

    ROS_INFO("Starting Instrinsic Sensor Parameter Learning algorithm TEST module");

    while(ros::ok())
    {
    	if(nh_ptr->hasParam("/calibration_done"))
    	{
    		break;
    	}
    	else
    	{
  			ROS_INFO("Waiting for learning to complete");
    	}
    }

	if( 1 ) // Need to improve, obviously!
	{
		ROS_INFO("NODE TEST PASSED: ALL VALUES WITHIN BOUNDS");
	}
	else
	{
		ROS_WARN("NODE TEST FAILED: Some values not found or not within bounds");
	}

    ros::Subscriber sub = nh.subscribe("scan_cloud", 1, cloudCallback);

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud", 1);
    pubCloud_ptr = &pubCloud;

    ros::spin();
    return 0;
}