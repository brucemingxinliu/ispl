#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::NodeHandle * nh_ptr;

PointCloud g_point_cloud_data;

ros::Publisher * pc_pub_ptr;

bool test_p_hit()
{
}

float p_hit()
{
}

void sort_cloud_slice(const PointCloud::ConstPtr& point_cloud)
{
	int cloud_size = point_cloud->points.size();

	float min_z_plane = 0.05;
	float max_z_plane = 0.15;
	
	for(int i = 0; i < cloud_size; i++)
	{
		if ((point_cloud->points[i].z < max_z_plane) && (point_cloud->points[i].z > min_z_plane))
		{
			g_point_cloud_data.push_back(point_cloud->points[i]);
		}
	}
}

bool g_cloud_received = false;
bool g_scan_received = false;

void cloudCB(const PointCloud::ConstPtr& cloud_holder)
{
	g_cloud_received = true;
	sort_cloud_slice(cloud_holder);
}

void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	g_scan_received = true;
}

bool waitForSubs()
{
	int count = 0;
	int time_to_wait = 5;
	ros::Rate count_rate(1);

	while(count < time_to_wait)
	{
		if((g_cloud_received == true) && (g_scan_received == true))
		{
			return true;
		}
		ros::spinOnce();
		count_rate.sleep();
		count++;
	}
	return false;
}

class SensorModel
{
public:
	bool createModel(PointCloud*);

	bool setLearningData();
	bool setLocation();
	bool setMap();
	bool learnParameters(PointCloud*);
private:

};

bool SensorModel::createModel(PointCloud * point_cloud)
{
	int cloud_size = point_cloud->points.size();

	ROS_INFO("Modeling sensor based on %d points", cloud_size);
	for(int i = 0; i < cloud_size; i++)
	{
		//ROS_INFO("Point: x=%f, y=%f, z=%f.", point_cloud->points[i].x,point_cloud->points[i].y, point_cloud->points[i].z);
	}

    point_cloud->header.frame_id = "map";
    pc_pub_ptr->publish(*point_cloud);

	setLearningData();

	setLocation();

	setMap();

	return learnParameters(point_cloud);
}

bool SensorModel::setLearningData()
{
	return true;
}

bool SensorModel::setLocation()
{
	return true;
}

bool SensorModel::setMap()
{
	return true;
}

bool SensorModel::learnParameters(PointCloud * Z)
{
	bool converged = false;
	int i = 0;
	int max_i = 50;

	// Loop following until convergence criteria is met
	do
	{

		i++;
	}
	while((converged == false) && (i <= max_i));

	if (i != max_i)
	{
		return true;
	}
	else
	{
		return false;	
	}
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"learn_intrinsic_parameters");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    bool test_active = true;
    bool test_passed = true;

    g_scan_received = false;
    g_cloud_received = false;

    ros::Subscriber map_sub = nh.subscribe("/ispl/scan_map", 1, scanCB);
    ros::Subscriber point_cloud_sub = nh.subscribe("/ispl/point_cloud", 1, cloudCB);

 	ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ispl/meas_pc", 1);
    pc_pub_ptr = &pc_pub;

    if(test_active == true)
    {
    	if(waitForSubs() == false)
    	{
    		ROS_WARN("Didn't receive any publications!");
    		test_passed = false;
    	}

    	// Instantiate a sensor model
    	SensorModel ourSensor;

    	if(ourSensor.createModel(&g_point_cloud_data) == false)
    	{
    		ROS_WARN("Failed to model sensor!");
    		test_passed = false;    	
    	}
    }

    if(test_passed == false)
    {
    	ROS_WARN("FUNCTION TEST FAILED: Something is borked");
    }
    else
    {
    	ROS_INFO("FUNCTION TEST PASSED");
    	nh_ptr->setParam("/ispl/success", true);
    }

    // Create sensor params, but should come from the algorithm 
    float z_hit = 1.55;
    float z_short = 5.62;
    float z_max = 0.66;
    float z_rand = 0.05;
    float sig_hit = 1.15;
    float lam_short = 0.1511;

    // Publish found parameters
    nh_ptr->setParam("/ispl/z_hit", z_hit);
    nh_ptr->setParam("/ispl/z_short", z_short);
    nh_ptr->setParam("/ispl/z_max", z_max);
    nh_ptr->setParam("/ispl/z_rand", z_rand);
    nh_ptr->setParam("/ispl/sig_hit", sig_hit);
    nh_ptr->setParam("/ispl/lam_short", lam_short);

    // Tell test node that we are done
    nh_ptr->setParam("/learning_done", true);
   
    return 0;
}