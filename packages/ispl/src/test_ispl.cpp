
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

ros::NodeHandle * nh_ptr;

// This is a test function for an ISPl paramter value to check that it is a reasonable value
bool is_reasonable_param(float sensor_param_val)
{
    // Liable to change
    if((sensor_param_val < 100000) && (sensor_param_val > 0))
    {
        return true;  
    }
    else
    {
        ROS_INFO("%f is not a reasonable param value.", sensor_param_val);
        return false;  
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_ispl");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    ROS_INFO("Starting Instrinsic Sensor Parameter Learning algorithm test module");

    // Wait for the learn_params node to finish
    while(ros::ok())
    {
    	if(nh_ptr->hasParam("/learning_done"))
    	{
    		break;
    	}
    	else
    	{
  			//ROS_INFO("Waiting for learning to complete");
    	}
    }

    // These represent the calculated output parameter values by the node
    float z_hit;;
    float z_short;
    float z_max;
    float z_rand;
    float sig_hit;
    float lam_short;

    bool node_level_tests_passed = false;

    // Retrieve these output parameter values from the param server, and check validity
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
            if( nh_ptr->hasParam("/ispl/success"))
            {
                node_level_tests_passed = true;
            }
        }
    }

    // Final test line report
	if(node_level_tests_passed == true)
	{
		ROS_INFO("NODE TEST PASSED: All tests complete");
	}
	else
	{
		ROS_WARN("NODE TEST FAILED: Something didn't work out right");
	}

    ros::spin();
    return 0;
}