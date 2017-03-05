#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>

// Define shorthand names for PCL points and clouds
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Point cloud of sensor measurements to make model off of
PointCloud g_point_cloud_data;

ros::NodeHandle * nh_ptr;

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

class MapFixture
{
public:
	bool setCorners(Point, Point, Point, Point);

	Point rayTrace(Point, Point);

private:

	// Point from which to conventionally measure things from wrt the plane
	Point origin_corner;
	// Other two defining points in the plane
	Point second_corner;
	Point third_corner;
	// Technically superfluous, but can use for numerical self-validation 
	Point validation_corner;

	// This point object represents the vector that is normal to the map fixture's plane
	Point plane_normal;
	// Together with the normal, this plane_parameter fully defines the equation of a 3D plane
	float plane_parameter;

	// Calcultes the unit-length (normalized) cross product between two points, presumably in the plane but not necessarily
	Point unitCrossProduct(Point, Point);
};

/*
Function: MapFixture::setCorners

Input: Four co-planar points that define the "screen" of the test fixture
Output: Sets various mathematical parameters related to the map fixture setup, returns 'true' if successful

Notes: The four input points should be very approximately co-planar or validation will fail and all bets are off
		Also, because I am an engineer and not a mathematician, I highly blur the line between what 
		a "point" is versus a "vector". Deal with it.
*/

bool MapFixture::setCorners(Point corner1, 
						Point corner2, 
						Point corner3, 
						Point corner4)
{
	// Just go ahead and store these points because they define the fixture
	// This poinnt is the "origin frame" of the plane
	MapFixture::origin_corner = corner1;
	// These next two points are needed to define a single plane
	MapFixture::second_corner = corner2;
	MapFixture::third_corner = corner3;
	// Can be used to check our work and check that the user knows what "co-planar" means
	MapFixture::validation_corner = corner4;

	//ROS_INFO("TEST: point(%f,%f,%f) x point(%f,%f,%f)...", second_corner.x,second_corner.y,second_corner.z,origin_corner.x,origin_corner.y,origin_corner.z);

	// Define a vector from origin on plane to a point on the plane
	Point first_vector(second_corner.x - origin_corner.x, 
						second_corner.y - origin_corner.y,
						second_corner.z - origin_corner.z);

	//ROS_INFO("TEST2: point(%f,%f,%f) x point(%f,%f,%f)...", third_corner.x,third_corner.y,third_corner.z,origin_corner.x,origin_corner.y,origin_corner.z);

	// Define another vector from origin on plane to another point on plane
	Point second_vector(third_corner.x - origin_corner.x, 
						third_corner.y - origin_corner.y,
						third_corner.z - origin_corner.z);

	// Calculate the (normalized) cross product of the two vectors, which will give us
	//   three of the parameters of the plane
	plane_normal = unitCrossProduct(first_vector, second_vector);

	// The fourth parameter of the plane is a function of other already-found parameters
	//   Think of it like an 'offset' value that is dependent on the rest of the plane parameters
	plane_parameter = -(plane_normal.x*origin_corner.x + plane_normal.y*origin_corner.y + plane_normal.z*origin_corner.z);
	//ROS_INFO("PLANE EQUATION IS: %f, %f, %f, %f", plane_normal.x, plane_normal.y, plane_normal.z, plane_parameter);
	return true;
}

Point MapFixture::rayTrace(Point origin, Point rayPoint)
{
	float x1 = origin.x;
	float y1 = origin.y;
	float z1 = origin.z;
	//ROS_INFO("Origin %f %f %f", x1, y1, z1);

	float a = rayPoint.x;
	float b = rayPoint.y;
	float c = rayPoint.z;
	//ROS_INFO("RayPt %f %f %f", a, b, c);

	float A = plane_normal.x;
	float B = plane_normal.y;
	float C = plane_normal.z;
	float D = plane_parameter;

	//ROS_INFO("2PLANE EQUATION IS: %f, %f, %f, %f", A, B, C, D);
	// Equation credit (and reference for those mathematically curious) goes to:
	//    http://www.ambrsoft.com/TrigoCalc/Plan3D/PlaneLineIntersection_.htm

	// This is the main calculation term, which is common to each of the three dimensions intersection points
	float common_term = (A*x1 + B*y1 + C*z1 + D)/(A*a + B*b + C*c); 
	// Calculate the intersection point 
	float intersection_x = x1 - (a*common_term);
	float intersection_y = y1 - (b*common_term);
	float intersection_z = z1 - (c*common_term);
	//ROS_INFO("Common term: %f, Intersection pts: %f, %f, %f", common_term, intersection_x, intersection_y, intersection_z);

	// Assemble three floats into a single point object
	Point intersection_point(intersection_x, intersection_y, intersection_z);
	if(!std::isfinite(intersection_x) || !std::isfinite(intersection_y) || !std::isfinite(intersection_z))
	{
		ROS_WARN("INVALID INTERSECTION POINT: One of the point coordinates was inf or nan!");
	}
	return intersection_point;
}

Point MapFixture::unitCrossProduct(Point u, Point v)
{
	//ROS_INFO("TEST: vector(%f,%f,%f) x vector(%f,%f,%f)...", u.x,u.y,u.z,v.x,v.y,v.z);
	float a = u.y*v.z - u.z*v.y;
	float b = u.z*v.x - u.x*v.z;
	float c = u.x*v.y - u.y*v.x;
	float length = a*a + b*b + c*c;
	a = a/length;
	b = b/length;
	c = c/length;
	//ROS_INFO("...is (%f,%f,%f)", a, b, c);

	return Point(a,b,c);
}

class SensorModel
{
public:
	bool createModel(PointCloud*, MapFixture *, Point *);

	bool learnParameters(PointCloud*);
private:
};

bool SensorModel::createModel(PointCloud * point_cloud, 
								MapFixture * map_plane,
								Point * origin)
{
	int cloud_size = point_cloud->points.size();

	ROS_INFO("Modeling sensor based on %d points", cloud_size);
	for(int i = 0; i < cloud_size; i++)
	{
		//ROS_INFO("Point: x=%f, y=%f, z=%f.", point_cloud->points[i].x,point_cloud->points[i].y, point_cloud->points[i].z);
	}

	// Publish point cloud for reference by other nodes
    point_cloud->header.frame_id = "map";
    pc_pub_ptr->publish(*point_cloud);

	return learnParameters(point_cloud);
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
    ros::Subscriber pc_sub = nh.subscribe("/ispl/point_cloud", 1, cloudCB);

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

    	MapFixture ourMap;

    	// Define the sensor as being at the origin of our 'universe'
    	Point sensor_origin(0,0,0);

    	// Set the dimensions (corner points) of the map fixture that the LIDAR will get data for
    	if(!ourMap.setCorners(Point(-2,1,1), Point(2,1,1), Point(-2,1,-1), Point(2,1,-1)))
    	{
    		ROS_WARN("FAILED TO SET CORNERS ON MAP FIXTURE");
    	}

    	std::vector<Point> test_points;
    	test_points.push_back(Point(0, 1, 1.5));
    	test_points.push_back(Point(0.2, 0.1, 1));
    	test_points.push_back(Point(0.2, 0, 1));
    	test_points.push_back(Point(-2, 5, 3));
    	test_points.push_back(Point(-1, 6, 0));
    	test_points.push_back(Point(0.5, 0, 0.5));

    	// Iterate through all test cases
    	for(int i = 0; i < test_points.size(); i++)
    	{
    		Point intersection_point = ourMap.rayTrace(sensor_origin, test_points[i]);
    		
    		ROS_INFO("The intersection of the line from (%f, %f, %f) to (%f, %f, %f),", 
    			sensor_origin.x, sensor_origin.y, sensor_origin.z, test_points[i].x, test_points[i].y, test_points[i].z);
    		ROS_INFO("is located at the point (%f, %f, %f).", intersection_point.x, intersection_point.y,intersection_point.z);
    	}

    	if(ourSensor.createModel(&g_point_cloud_data, &ourMap, &sensor_origin) == false)
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