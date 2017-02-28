#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

ros::NodeHandle * nh_ptr;

bool is_reasonable_param(float sensor_param_val)
{
    if((sensor_param_val < 100000) && (sensor_param_val > 0))
    {
        ROS_INFO("%f sounds good", sensor_param_val);
        return true;  
    }
    else
    {
        ROS_INFO("%f is not a reasonable param value.", sensor_param_val);
        return true;  
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_ispl");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    bool node_level_tests_passed = false;

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

    float z_hit;;
    float z_short;
    float z_max;
    float z_rand;
    float sig_hit;
    float lam_short;

    if (nh_ptr->getParam("/ispl/z_hit", z_hit) &&
        nh_ptr->getParam("/ispl/z_short", z_short) &&
        nh_ptr->getParam("/ispl/z_max", z_max) &&
        nh_ptr->getParam("/ispl/z_rand", z_rand) &&
        nh_ptr->getParam("/ispl/sig_hit", sig_hit) &&
        nh_ptr->getParam("/ispl/lam_short", lam_short))
    {
        if (is_reasonable_param(z_hit) &&
            is_reasonable_param(z_short) &&
            is_reasonable_param(z_max) &&
            is_reasonable_param(z_rand) &&
            is_reasonable_param(sig_hit) &&
            is_reasonable_param(lam_short))
        {
            node_level_tests_passed = true;
        }
    }

	if(node_level_tests_passed == true)
	{
		ROS_INFO("NODE TEST PASSED: ALL PARAMTERS FOUND");
	}
	else
	{
		ROS_WARN("NODE TEST FAILED: Something didn't work out right");
	}

    ros::spin();
    return 0;
}