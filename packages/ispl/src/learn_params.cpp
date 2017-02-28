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

bool test_p_hit()
{

}

float p_hit()
{

}



int learn_intrinsic_parameters()
{
	bool converged = false;

	// Loop following until convergence criteria is met
	do
	{


	}
	while(converged == false);

	return 0;
}

void get_cloud_slice(const PointCloud::ConstPtr& point_cloud)
{
	int cloud_size = point_cloud->points.size();

	float min_z_plane = 0.2;
	float max_z_plane = 0.3;
	
	PointCloud level_cloud;
	for(int i = 0; i < cloud_size; i++)
	{
		if ((point_cloud->points[i].z < max_z_plane) && (point_cloud->points[i].z > min_z_plane))
		{
			level_cloud.push_back(point_cloud->points[i]);
			ROS_INFO("HEY");
		}

	}

}

void cloudCB(const PointCloud::ConstPtr& cloud_holder)
{
	get_cloud_slice(cloud_holder);
}

void mapCB(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"learn_intrinsic_parameters");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    ros::Subscriber point_cloud_sub = nh.subscribe("/ispl/point_cloud", 1, cloudCB);
    ros::Subscriber map_sub = nh.subscribe("/ispl/map", 1, mapCB);

    //learn_intrinsic_parameters();

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