// Internal Sensor Parameter Learning node
//created by Trent Ziemer
//updated by Mingxin Liu

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

/*
Function: sort_cloud_slice()
Input: A rostopic Point Cloud pointer, specifically from a rostopic Point cloud callback function
Output: Stores all points from input cloud into a global point cloud for points between two z values.
Notes: WARNING: This function is hacky.
*/
void cloudCB(const PointCloud::ConstPtr& point_cloud)
{
	int cloud_size = point_cloud->points.size();

	//////////    DIFFERENT POSSIBLE FILTERS FOR VARIOUS DATA SETS. NOTE: THIS IS A HACKY WAY
	/* FOR BAG 1 
	float min_x = 0.69;
	float max_x = 0.74;
	float min_y = -0.134;
	float max_y = 0.47;
	float min_z = 0.46;
	float max_z = 1.122;
	*/

	/* FOR BAG 1 with B Coords */
	float min_x = -0.4;
	float max_x = 0.4;
	float min_y = -0.1;
	float max_y = 0.1;
	float min_z = -0.05;
	float max_z = 0.48; 
	
    /* FOR BAG 2
    float min_x = 0.564;
    float max_x = 0.74;
    float min_y = -0.134;
    float max_y = 0.47;
    float min_z = 0.46;
    float max_z = 1.122;
	*/  

	/* FOR BAG 3 OR 4 
	float min_x = 0.55;
	float max_x = 0.63;
	float min_y = -0.11;
	float max_y = 0.5;
	float min_z = 0.48;
	float max_z = 1.122;
	*/

	/* FOR BAG 5 with B Coords 
	float min_x = -27.8/100;
	float max_x = 27.8/100;
	float min_y = -18.07/100;
	float max_y = 18.07/100;
	float min_z = -0.01;
	float max_z = 44/100; */ 

	/* FOR BAG 6 with B Coords 
	float min_x = -0.0799;
	float max_x = 0.0799;
	float min_y = -0.321;
	float max_y = 0.321;
	float min_z = -0.01;
	float max_z = 0.44;
*/

   	int filtering_constant = 1;
	int counter = 0;

	for(int i = 0; i < cloud_size; i++)
	{
		if ((point_cloud->points[i].z < max_z) && (point_cloud->points[i].z > min_z))
		{
			if ((point_cloud->points[i].y < max_y) && (point_cloud->points[i].y > min_y))
			{
				if ((point_cloud->points[i].x < max_x) && (point_cloud->points[i].x > min_x))
				{
					if(counter % filtering_constant == 0)
					{
						g_point_cloud_data.push_back(point_cloud->points[i]);		 // NOTE: CHANGE THIS; IT BREAK THE FILE READ IF WE DO THIS WHILE TRYING TO FILE READ AND THERE IS A POINT CLOUD GETTING PUBLISHED TOO
					}
					counter++;
				}
			}
		}
	}
	g_cloud_received = true;
}

// Waits for GLOBALLY DEFINED BOOLEANS to become true for a set number of seconds that are triggered by specific ros topics
// Quick and dirty ROS-based initialization
bool waitForSubs()
{
	int count = 0;
	int time_to_wait = 12000; // ms
	ros::Rate count_rate(1000); // ms

	while(count < time_to_wait)
	{
		if((g_cloud_received == true))
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
// NOTE: currently, it doesnt actually validate the points given automatically
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
				if(!ourMap->validateCorner(Point(i, j, k)))
				{
					return false;
				}
			}
		}
	}

	return true;
}

// Mega-hacky function
bool getDataFromFile(std::string filename)
{
    std::ifstream input_data;
    input_data.open(filename.c_str());
    if(!input_data.is_open())
    {
        ROS_WARN("%s IS NOT OPEN", filename.c_str());
    }

    float value;
    float value2;
    int j = 1;
    if(0) // THIS IS A HACK DONT EVEN BOTHER TRYING TO COMPREHEND
    {
    	while(input_data >> value)
    	{
    		value2 =  1.732051*j/(88*sqrt(3));
    		for(int i = 0; i < value; i++)
    		{
    			g_point_cloud_data.push_back(Point(value2, value2, value2));
    		}
    		// ROS_INFO("Added %f of %f", value, value2*sqrt(3));
    		j++;
    	}
    }
    else // This is real...a real state machine
    {
    bool getting_x = true;
    bool getting_y = false;
    bool getting_z = false;

    float x;
    float y;
    while(input_data >> value)
    {
        if(getting_x)
        {
        	x = value;
        	getting_x = false;
        	getting_y = true;
        }
        else if(getting_y)
        {
        	y = value;
        	getting_y = false;
        	getting_z = true;
        }
        else if(getting_z)
        {
        	//ROS_INFO("Adding Point(%f, %f, %f) to global cloud", x, y, value);
			g_point_cloud_data.push_back(Point(x, y, value));
			getting_z = false;
        	getting_x = true;
        }
    }
    }
    if(g_point_cloud_data.points.size() > 0)
    {
    	return true;
    }
    else
    {
    	return false;
    }
}

///////////////////////////        MAIN       ////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc,argv,"learn_intrinsic_parameters");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    bool test_active = true;
    bool test_active2 = false;
    bool test_passed = true;

    g_cloud_received = false;

    ros::Subscriber pc_sub = nh.subscribe("/ispl/point_cloud", 1, cloudCB);
    
    // Publish point cloud for reference by other nodes, not currently really useful
	ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ispl/meas_pc", 1);
	ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ispl/map_pc", 1);
    pc_pub_ptr = &pc_pub;

    // Instantiate a sensor model
    SensorModel ourSensor;

	// Instantiate our map model
	MapFixture ourMap;
   
    std::string data_source;
    std::string filename;

	Point sensorOrigin;

	// Set the dimensions (corner points) of the map fixture that the LIDAR will get data for

	//////////    THESE ARE VARIOUS "MAP" PLANES (TEST FIXTURES) THAT WE CAN USE. NOTE: THIS IS MEGA HACKY
	// WARNING: Need to set the variable names in the setCorners method call below
	// 1
	Point map1_1 = Point(-0.33, 0, 0);       
	Point map1_2 = Point(-0.33, 0, 0.43);
	Point map1_3 = Point( 0.33, 0, 0.43);
	Point map1_4 = Point( 0.33, 0, 0);

	/* FROM MY ESTIMATIONS
	Point map1_1 = Point(0.6935, 0.47187, 1.1);
	Point map1_2 = Point(0.724,-0.11783,1.1219);
	Point map1_3 = Point(0.70244,0.4681,0.48421);
	Point map1_4 = Point(0.73256,-0.13364,0.4661); */
	
	// 2 FROM MY ESTIMATIONS
	Point map2_1 = Point(0.564, 0.47979, 0.49127);
	Point map2_2 = Point(0.623,0.5034,1.1075);
	Point map2_3 = Point(0.6586,-0.1022,1.129);
	Point map2_4 = Point(0.6045,-0.1152,0.4672);

	// 3 == 4 FROM MY ESTIMATIONS
	Point map3_1 = Point(0.6026, -0.1106, 0.49);
	Point map3_2 = Point(0.624,-0.11783,1.12);
	Point map3_3 = Point(0.582,0.4927,1.0735);
	Point map3_4 = Point(0.56, 0.49 ,0.485);

	/* // 5
	Point map5_1 = Point(-0.2769, -0.18051, 0);
	Point map5_2 = Point(-0.2769, -0.18051, 0.43);
	Point map5_3 = Point(0.2769, 0.18051, 0.43);
	Point map5_4 = Point(0.2769, 0.18051, 0); */

	// 5 - MY ESTIMATIONS
	Point map5_1 = Point(0.62667, 0.2762, 0.52265);
	Point map5_2 = Point(0.61922, 0.17958, 1.1208);
	Point map5_3 = Point(0.46219, 0.-0.02067, 1.1212);
	Point map5_4 = Point(0.38731, -0.018492, 0.481);

	/* // 6
	Point map6_1 = Point(-0.07986, -0.3201, 0);
	Point map6_2 = Point(-0.07986, -0.3201, 0.43);
	Point map6_3 = Point(0.07986,  0.3201, 0.43);
	Point map6_4 = Point(0.07986,  0.3201, 0); */

	// 6
	Point map6_1 = Point(0.3116, 0.10595, 0.48461);
	Point map6_2 = Point(0.43174, 0.10445, 1.1206);
	Point map6_3 = Point(0.60237, 0.17013, 1.0839);
	Point map6_4 = Point(0.61587, 0.19083, 0.54741);

	// freq_data2 
	Point mapf_1 = Point(1.049, 0.97369, 1.0071);
	Point mapf_2 = Point(1.0471, 0.98999, 0.98648);
	Point mapf_3 = Point(0.99777, 1.0457, 0.98328);
	Point mapf_4 = Point(0.99012, 1.0189, 1.0196);

    if(test_active == true)
    {
		if(!nh_ptr->getParam("data_source", data_source))
		{
			ROS_WARN("Failed to get data source type (file or topic)!");
		}
		if(!nh_ptr->getParam("filename", filename))
		{
			ROS_WARN("Failed to get file name for file data source!");
		}
		if(   !nh_ptr->getParam("sensor_origin_x", sensorOrigin.x) 
		   || !nh_ptr->getParam("sensor_origin_y", sensorOrigin.y)
		   || !nh_ptr->getParam("sensor_origin_z", sensorOrigin.z))
		{
			ROS_WARN("Failed to get sensor origin parameters!");
		}

    	if(data_source == "file")
    	{
    		if(!getDataFromFile(filename.c_str()))
    		{
    			ROS_WARN("Couldn't get data from file!");
				test_passed = false;
    		}
    	}
    	else if (data_source == "topic")
    	{
    		if(!waitForSubs())
			{
				ROS_WARN("Didn't receive any publications!");
				test_passed = false;
			}
    	}
    	else
    	{
    		ROS_WARN("Failed to read data_source type!");
    		test_passed = false;
    	}

    	if(!ourMap.setCorners(map1_1, map1_2, map1_3, map1_4))
    	{
    		ROS_WARN("Failed to set corners on map fixture!");
    	}

    	// Set the "seed" values of the 6 internal sensor parameters. Doesn't matter much what they are, except for converge time.
    	if(!ourSensor.setInitialParams(0.4, 0.3, 0.2, 0.1, 0.5, 1.1))
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

    	if(g_point_cloud_data.size() > 0)
    	{
	    	// Create a model of the sensor based on a set of PCL data and a defined map
	    	if(!ourSensor.createModel(&g_point_cloud_data, &ourMap, &sensorOrigin))
	    	{
	    		ROS_WARN("Failed to model sensor!");
	    		test_passed = false;    	
	    	}
    	}
    	else
    	{
    		ROS_WARN("Failed to try to create model: point cloud is empty!");
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
   
   	int time_to_wait = 1000; // ms
	ros::Rate count_rate(10); // ms
	int count = 0;

	// Create point cloud of the "map" that we used
	PointCloud map_pc;

   	while(ros::ok() && (count < time_to_wait))
   	{
		// Send an output cloud of what we measured, for other nodes to see
		g_point_cloud_data.header.frame_id = "lidar_link"; // Or lidar link?
		pc_pub_ptr->publish(g_point_cloud_data);

		map_pc.push_back(map3_1);
		map_pc.push_back(map3_2);
		map_pc.push_back(map3_3);
		map_pc.push_back(map3_4);
		map_pc.push_back(sensorOrigin);
		
		map_pc.header.frame_id = "lidar_link";
		map_pub.publish(map_pc);
		
		ros::spinOnce();
		count_rate.sleep();
		count++;
	}

    return 0;
}
