// Internal Sensor Parameter Learning node
// Created Feb 23 2017 by Trent Ziemer
// Last updated (NEEDS UPDATE) by Trent Ziemer

// Includes map fixture and sensor model classes
#include <ispl/models.h>

// File IO for C++
#include <iostream>
#include <fstream>

// Define shorthand names for PCL points and clouds
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Point cloud of sensor measurements to be used to make our sensor model of parameters
PointCloud g_point_cloud_data;

// Variables that help the main program wait until ROS topics for input data are received
bool g_cloud_received;
bool g_scan_received;

/*
Function: sort_cloud_slice()
Input: A rostopic Point Cloud pointer, specifically from a rostopic Point cloud callback function
Output: Stores all points from input cloud into a global point cloud for points between two z values.
Notes: WARNING: This function is hacky.
*/
void sort_cloud_slice(const PointCloud::ConstPtr& point_cloud)
{
	int cloud_size = point_cloud->points.size();

	float min_z_plane = -0.5;
	float max_z_plane = 0.1;
	
	for(int i = 0; i < cloud_size; i++)
	{
		if ((point_cloud->points[i].z < max_z_plane) && (point_cloud->points[i].z > min_z_plane))
		{
			g_point_cloud_data.push_back(point_cloud->points[i]);
		}
	}
}

/*
*/
void cloudCB(const PointCloud::ConstPtr& cloud_holder)
{
	g_cloud_received = true;
	sort_cloud_slice(cloud_holder);
}

/*
*/
void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	g_scan_received = true;
}

// Waits for GLOBALLY DEFINED BOOLEANS to become true for a set number of seconds that are triggered by specific ros topics
// Quick and dirty ROS-based initialization
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

//////////////////////////     FUNCTIONAL TESTING    /////////////////
bool runIntersectionTesting(SensorModel * ourSensor, MapFixture * ourMap, Point sensorOrigin)
{
	std::vector<Point> test_points;
	test_points.push_back(Point(0, 1, 1.5));
	test_points.push_back(Point(0.2, 0.1, 1));
	test_points.push_back(Point(0.2, 5, 1));
	test_points.push_back(Point(-2, 5, 3));
	test_points.push_back(Point(-1, 6, 0));
	test_points.push_back(Point(0.5, 0.02, 0.5));

	// Iterate through all test cases added above
	for(int i = 0; i < test_points.size(); i++)
	{
		Point intersection_point = ourMap->rayTrace(sensorOrigin, test_points[i]);
		
		ROS_INFO("The intersection of the line from (%f, %f, %f) to (%f, %f, %f),", 
			sensorOrigin.x, sensorOrigin.y, sensorOrigin.z, test_points[i].x, test_points[i].y, test_points[i].z);
		ROS_INFO("is located at the point (%f, %f, %f).", intersection_point.x, intersection_point.y,intersection_point.z);
	}
    return true;
}

// This function can be used to display distances between various points and the map plane
// NOTE: currently, it doesnt actually validate the points given automatically (the return from that function is not stored)
bool runPlaneValidation(MapFixture * ourMap)
{
	// Set the size and resolution of our validation space
	float left_point = -5;
	float right_point = 5;
	float delta_point = 0.5;
	// Iterate through whole space to check if each point lies on the plane
	for(float i = left_point; i < right_point; i += delta_point)
	{
		for(float j = left_point; j < right_point; j += delta_point)
		{
			for(float k = left_point; k < right_point; k += delta_point)
			{
				Point planePoint = ourMap->getPlaneNormal();
				ROS_INFO("Test Point (%f, %f, %f)", i, j, k);
				ROS_INFO("Plane Eqn (%f, %f, %f | %f)", planePoint.x, planePoint.y, planePoint.z, ourMap->getPlaneParameter());
				ourMap->validateCorner(Point(i, j, k));
			}
		}
	}

	return false;
}
///////////////////////////        MAIN       ////////////////////////////
int main(int argc, char **argv)
{

	// THINGS TO DO:
	// Output data as histogram for manual analysis?
	// Take in /home/mordoc/ispl_data as a histogram and process and then run alg on it
	// Check if need to filter (my rosbag) incoming data
	// Iterate across various z-heights (at top) and maybe do some analysis?
    ros::init(argc,argv,"learn_intrinsic_parameters");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    bool test_active = true;
    bool test_active2 = false;
    bool test_passed = true;

    g_scan_received = false;
    g_cloud_received = false;

    ros::Subscriber map_sub = nh.subscribe("/ispl/scan_map", 1, scanCB);
    ros::Subscriber pc_sub = nh.subscribe("/ispl/point_cloud", 1, cloudCB);
    
    // Publish point cloud for reference by other nodes, not currently really useful
	ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ispl/meas_pc", 1);
    pc_pub_ptr = &pc_pub;

    // Instantiate a sensor model
    SensorModel ourSensor;

	// Instantiate our map model
	MapFixture ourMap;

    // Instantiate the sensor location as being at the origin of our 'universe', this is a basic assumption of our map model
	Point sensorOrigin(0,0,0);

    if(test_active == true)
    {
    	if(waitForSubs() == false)
    	{
    		ROS_WARN("Didn't receive any publications!");
    		test_passed = false;
    	}

    	// Set the dimensions (corner points) of the map fixture that the LIDAR will get data for
    	// These are current assumptions that can/should/will/may change
    	if(!ourMap.setCorners(Point(-2,1,1), Point(2,1,1), Point(-2,1,-1), Point(2,1,-1)))
    	{
    		ROS_WARN("Failed to set corners on map fixture!");
    	}

    	// Set the "seed" values of the 6 internal sensor parameters. Doesn't matter much what they are?
    	if(!ourSensor.setInitialParams(0.5, 0.1, 0.2, 0.2, 0.8, 0.2))
    	{
    		ROS_WARN("Failed to set initial model parameter values!");
    		test_passed = false;
    	}

    	if(test_active2 == true)
    	{
	    	// Run a test suite of various points that reports intersection locations
	    	if(!runIntersectionTesting(&ourSensor, &ourMap, sensorOrigin))
	    	{
	    		ROS_WARN("Failed to find number intersections in map/sensor testing!");
				test_passed = false;
	    	}	

	    	if(!runPlaneValidation(&ourMap))
	    	{	    		
	    		ROS_WARN("Failed to validate validateCorner function!");
				test_passed = false;
	    	}
    	}

    	// Create a model of the sensor based on a set of PCL data and a defined map
    	if(!ourSensor.createModel(&g_point_cloud_data, &ourMap, &sensorOrigin))
    	{
    		ROS_WARN("Failed to model sensor!");
    		test_passed = false;    	
    	}
    }

    // Check this-node functional testing and report to user
    if(test_passed == false)
    {
    	ROS_WARN("FUNCTIONAL TESTING FAILED: Something is borked");
    }
    else
    {
    	ROS_INFO("LEARN PARAMS FUNCTION TESTING PASSED");
    	nh_ptr->setParam("/ispl/success", true);
    }

    // Publish found parameters contained within sensor model
    nh_ptr->setParam("/ispl/z_hit", ourSensor.z_hit);
    nh_ptr->setParam("/ispl/z_short", ourSensor.z_short);
    nh_ptr->setParam("/ispl/z_max", ourSensor.z_max);
    nh_ptr->setParam("/ispl/z_rand", ourSensor.z_rand);
    nh_ptr->setParam("/ispl/sig_hit", ourSensor.sig_hit);
    nh_ptr->setParam("/ispl/lam_short", ourSensor.lam_short);

    // Tell test node that we are done, so it can start doing things
    nh_ptr->setParam("/learning_done", true);
   
    return 0;
}