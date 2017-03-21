
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::NodeHandle * nh_ptr;

bool g_received_measurements;

// This is a test function for an ISPl paramter value to check that it is a reasonable value
// This should be updated with new constraints whenever is appropriate
bool is_reasonable_param(float sensor_param_val)
{
    // Liable to change
    if((sensor_param_val < 100000) && (sensor_param_val >= 0))
    {
        return true;  
    }
    else
    {
        ROS_INFO("%f is not a reasonable param value.", sensor_param_val);
        return false;  
    }
}

void cloudCB(const PointCloud::ConstPtr& point_cloud)
{
    g_received_measurements = true;
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_ispl");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    g_received_measurements = false;

    ros::Subscriber meas_sub = nh.subscribe("/ispl/meas_pc", 1, cloudCB);

    ROS_INFO("Starting ISPL algorithm test node");

    // Wait for the learn_params node to finish, then continue with checking for params
    while(ros::ok())
    {
    	if(nh_ptr->hasParam("/learning_done"))
    	{
            break;
    	}
        ros::spinOnce();
    }

    // These represent the calculated output parameter values from the node
    float z_hit;;
    float z_short;
    float z_max;
    float z_rand;
    float sig_hit;
    float lam_short;

    int node_level_tests_failed = 0;

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
			ROS_INFO("Found parameters: \n z_hit = %f \n z_short = %f \n z_max = %f \n z_rand = %f \n | sig_hit = %f \n | lam_short = %f", z_hit, z_short, z_max, z_rand, sig_hit, lam_short);
        	// This param value is main test result from module
            if(nh_ptr->hasParam("/ispl/success"))
            {
            	ROS_INFO("...and these values may be trusted.");
            }
            else
            {
            	ROS_INFO("...but the values are not trustworthy!");
                node_level_tests_failed++;
            }
        }
        else
        {
            node_level_tests_failed++;
        }
    }
    else
    {
        node_level_tests_failed++;
    }

    if (g_received_measurements == false)
    {
        node_level_tests_failed++;
        ROS_WARN("Never received published point cloud of measurements!");
    }

    // Final test line report
	if(node_level_tests_failed == 0)
	{
		ROS_INFO("NODE TEST PASSED: All tests complete");
	}
	else
	{
		ROS_WARN("NODE TESTS FAILED = %d", node_level_tests_failed);
	}

    return 0;
}