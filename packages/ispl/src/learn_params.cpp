// Sensor Parameter learning node
// Created Feb 23 2017 by Trent Ziemer
// Last updated (NEEDS UPDATE) by Trent Ziemer
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>

#define PI 3.159265
// Used for numerical integration of Gaussian functions
#define INTEGRAL_STEPS 1000
// Define shorthand names for PCL points and clouds
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Point cloud of sensor measurements to make model off of
PointCloud g_point_cloud_data;

// ROS global pointers 
ros::NodeHandle * nh_ptr;
ros::Publisher * pc_pub_ptr;

// Variables that help the main program wait until ros topics are received
bool g_cloud_received = false;
bool g_scan_received = false;

/*

*/
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

/*

*/
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

/////////////////////           MATHEMATICS FUNCTIONS           ////////////////
// Calculates the y value of the normal distribution at some point x given some mean and variance
float normalDistribution(float x, float mean, float variance)
{
	float exp_term = exp(-(x - mean)*(x - mean)/(2*variance));

	return exp_term/(sqrt(2*PI*variance));
}

// Courtesy helloacm.com, slightly modified to integrate a gaussian function with a mean and variance
float integral(float(*f)(float x1, float x2, float x3), float a, float b, int n, float mean, float variance) {
    float step = (b - a) / n;  // width of each small rectangle
    float area = 0.0;  // signed area
    for (int i = 0; i < n; i ++) {
        area += f(a + (i + 0.5) * step, mean, variance) * step; // sum up each small rectangle
    }
    return area;
}

/*
Function: vectorLength()
Input: Two PCL Point objects anywhere in space
Output: Returns the float value of the magnitude length from one point to the other point 
Notes: Put garbage in (inf, nan), get garbage out (inf, nan)
*/
float vectorLength(Point a, Point b)
{
	Point vector(a.x - b.x, a.y - b.y, a.z - b.z);
	return sqrt((vector.x)*(vector.x) + (vector.y)*(vector.y) + (vector.z)*(vector.z));
}

/////////////////////////////          MAP FIXTURE       /////////////////////
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
	float a = u.y*v.z - u.z*v.y;
	float b = u.z*v.x - u.x*v.z;
	float c = u.x*v.y - u.y*v.x;
	float length = a*a + b*b + c*c;
	a = a/length;
	b = b/length;
	c = c/length;

	return Point(a,b,c);
}

/////////////////////////////      SENSOR MODEL       /////////////////////////
class SensorModel
{
public:
	bool createModel(PointCloud*, MapFixture *, Point *);

	float z_hit;
	float z_short;
	float z_max;
	float z_rand;
	float sig_hit;
	float lam_short;
private:
	bool learnParameters(PointCloud*, Point *, MapFixture *);
	float p_hit(Point, Point, MapFixture *);
	float p_short(Point, Point, MapFixture *);
	float p_max(Point, Point, MapFixture *);
	float p_rand(Point, Point, MapFixture *);

	float p_hit_offset(Point, Point, MapFixture *);

	// A static parameter of any single point cloud, it's the distance to the farthest point
	float furthest_z;

	// A computational-only parameter of the model. Represents max %difference from z_max that the measurement can be
	//   Occurs due to the fact that computers cannot process infinitesimal width distributions
	float z_max_tol;
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

	// Publish point cloud for reference by other nodes, not currently really useful
    point_cloud->header.frame_id = "map";
    pc_pub_ptr->publish(*point_cloud);

    // Determine which point is the farthest away from the sensor, call this the longest range, which is thus furthest_z
    float longest_range = -1; 
    float range;
	for(int i = 0; i < cloud_size; i++)
	{
		Point meas_point = point_cloud->points[i];
		range = sqrt((meas_point.x)*(meas_point.x) + (meas_point.y)*(meas_point.y) 
						+ (meas_point.z)*(meas_point.z));
		if(range > longest_range)
		{
			longest_range = range;
		}
	}

	// Check that we actually found a furthest_z here, and store it in the object
	if (longest_range > 0)
	{
		furthest_z = longest_range;
	}
	else
	{
		ROS_WARN("COULD NOT FIND furthest_z/longest_range, furthest_z is uninitialized!");
	}

	// This seems reasonable to me
	z_max_tol = 0.01;

	// Do magic!
	return learnParameters(point_cloud, origin, map_plane);
}

bool SensorModel::learnParameters(PointCloud * Z, Point * X, MapFixture * m)
{
	PointCloud data_cloud = *Z;
	Point sensor_origin = *X;
	// Convergence learning parameters
	bool converged = false;
	int i = 0;
	// Make maybe 50 later?
	int max_i = 50;

	// ASSUME for now. This is the intrinsic noise parameter of the meas model
	sig_hit = 0.1;
	lam_short = 0.2;

	float p_hit_val;
	float p_short_val;
	float p_max_val;
	float p_rand_val;

	float p_hit_offset_val;
	
	// Overall normalization value
	float eta;

	std::vector<float> e_hit;
	std::vector<float> e_short;
	std::vector<float> e_max;
	std::vector<float> e_rand;
	std::vector<float> e_hit_offset;

	float mag_Z = 0;

	float e_hit_sum = 0;
	float e_short_sum = 0;
	float e_max_sum = 0;
	float e_rand_sum = 0;

	float e_hit_offset_sum = 0;
	float e_short_ext_sum = 0;

	// Loop following until convergence criteria is met or we try to many times?
	do
	{
		for (int k = 0; k < Z->size(); k++)
		{
			p_hit_val = p_hit(data_cloud[k], sensor_origin, m);
			p_short_val = p_short(data_cloud[k], sensor_origin, m);
			p_max_val = p_max(data_cloud[k], sensor_origin, m);
			p_rand_val = p_rand(data_cloud[k], sensor_origin, m);

			// This includes "calculate z_i_star":
			p_hit_offset_val = p_hit_offset(data_cloud[k], sensor_origin, m);

			/*
			ROS_INFO("p_hit_val = %f", p_hit_val);
			ROS_INFO("p_short_val = %f", p_short_val);
			ROS_INFO("p_max_val = %f", p_max_val);
			ROS_INFO("p_rand_val = %f", p_rand_val);
			*/

			eta = 1/(p_hit_val + p_short_val + p_max_val + p_rand_val);

			// NOTE: As long as this loops index stays in 0 to size(z) order, then index of data_cloud and these four vectors will match up
			e_hit.push_back(eta * p_hit_val);
			e_short.push_back(eta * p_short_val);
			e_max.push_back(eta * p_max_val);
			e_rand.push_back(eta * p_rand_val);

			// For sig_hit re-calculation later
			e_hit_offset.push_back(eta * p_hit_offset_val);

			// Add on this point cloud magnitude to the sum of all magnitudes in the point cloud for later use
			mag_Z += vectorLength(sensor_origin, data_cloud[k]);
		}

		// Compute sums...
		for(int j = 0; j < e_hit.size(); j++)
		{
			e_hit_sum += e_hit[j];
		}

		for(int j = 0; j < e_short.size(); j++)
		{
			e_short_sum += e_short[j];
		}

		for(int j = 0; j < e_short.size(); j++)
		{
			e_short_ext_sum += e_short[j]*vectorLength(sensor_origin, data_cloud[j]);
		}

		for(int j = 0; j < e_max.size(); j++)
		{
			e_max_sum += e_max[j];
		}

		for(int j = 0; j < e_rand.size(); j++)
		{
			e_rand_sum += e_rand[j];
		}

		for(int j = 0; j < e_hit_offset.size(); j++)
		{
			e_hit_offset_sum += pow(e_hit_offset[j], 2);
		}

		// Compute each four of these parameters (at least for this iteration)
		z_hit = e_hit_sum/mag_Z;
		z_short = e_short_sum/mag_Z;
		z_max = e_max_sum/mag_Z;
		z_rand = e_rand_sum/mag_Z;

		sig_hit = sqrt((e_hit_offset_sum)/(e_hit_sum));

		lam_short = e_short_sum/e_short_ext_sum;

		ROS_INFO("z_hit: %f", z_hit);
		ROS_INFO("z_short: %f", z_short);
		ROS_INFO("z_max: %f", z_max);
		ROS_INFO("z_rand: %f", z_rand);
		ROS_INFO("sig_hit: %f", sig_hit);
		ROS_INFO("lam_short: %f", lam_short);

		i++;
	}
	while((converged == false) && (i < max_i));

	if (i != max_i)
	{
		return true;
	}
	else
	{
		return false;	
	}
}


/*
Function: SensorModel::p_hit_offset
Input: A single measurement (point), as well as the location of the sensor and the map of the test fixture
Output: Returns the probability that the measurement specified actually hit the location specified by the sensor/map geometry, given a normal noise distribution
Notes: Companion function to "p_hit". Calls for sig_hit and furthest_z, within the SensorModel instantiation object
*/
float SensorModel::p_hit_offset(Point meas_point, Point sensor_origin, MapFixture * m)
{
	// Ray trace from the origin to the measurement point to find where it intersects the map
	// This point is the location of where the meas_point 'should' have been
	Point intersection_point = m->rayTrace(sensor_origin, meas_point);

	// Compute the magnitude distance from the sensor to the measurment point
	float z_k = vectorLength(sensor_origin, meas_point);

	// Compute the magnitude distance from the sensor to the intersection point
	float z_k_star = vectorLength(sensor_origin, intersection_point);

	//float temp = z_k;
	z_k = z_k - z_k_star;
	//ROS_INFO("Reassigning z_k from %f to %f for p hit offset thing.", temp, z_k);

	if(!std::isfinite(z_k_star))
	{
		ROS_WARN("P_HIT_OFFSET reports that the measurement point doesn't intersect the map plane, returning 0!");
		return 0;
	}
	else
	{
		// Compute the total area under the normal distribution curve
		float in2 = normalDistribution(z_k_star, z_k_star, sig_hit*sig_hit);
		float eta2 = 1/in2;
		float nd = normalDistribution(z_k, z_k_star, sig_hit*sig_hit);
		float p_hit_offset;
		if(!std::isfinite(eta2))
		{
			p_hit_offset = 0; // Otherwise p_hit_offset ends up being not a number; don't worry, all this means is that p_hit_offset is nothing
		}
		else
		{
			p_hit_offset = eta2*nd;
		}

		if(p_hit_offset < 0 || p_hit_offset > 1) //!std::isfinite(p_hit_offset))
		{
			ROS_WARN("WARNING: Potential improper p_hit_offset value!");
			ROS_INFO("Measured Pt.: (%f, %f, %f)", meas_point.x, meas_point.y, meas_point.z);
			ROS_INFO("Intersection Pt.: (%f, %f, %f)", intersection_point.x, intersection_point.y, intersection_point.z);
			ROS_INFO("Distance to measured point z_k = %f", z_k);
			ROS_INFO("Distance to intersection point z_k_star = %f", z_k_star);
			ROS_INFO("Sig_hit = %f  furthest_z = %f", sig_hit, furthest_z);
			ROS_INFO("Integrated normalizer = %f", in2);
			ROS_INFO("eta2 is %f", eta2);
			ROS_INFO("nd is %f", nd);
			ROS_INFO("p_hit_offset is %f \n", p_hit_offset);
		}
		return p_hit_offset;
	}
}

/*
Function: SensorModel::p_hit
Input: A single measurement (point), as well as the location of the sensor and the map of the test fixture
Output: Returns the probability that the measurement specified actually hit the location specified by the sensor/map geometry, given a normal noise distribution
Notes: Calls for sig_hit and furthest_z, within the SensorModel instantiation object
*/
float SensorModel::p_hit(Point meas_point, Point sensor_origin, MapFixture * m)
{
	// Ray trace from the origin to the measurement point to find where it intersects the map
	// This point is the location of where the meas_point 'should' have been
	Point intersection_point = m->rayTrace(sensor_origin, meas_point);

	// Compute the magnitude distance from the sensor to the measurment point
	float z_k = vectorLength(sensor_origin, meas_point);

	// Compute the magnitude distance from the sensor to the intersection point
	float z_k_star = vectorLength(sensor_origin, intersection_point);

	if(!std::isfinite(z_k_star))
	{
		ROS_WARN("P_HIT reports that the measurement point doesn't intersect the map plane, returning 0!");
		return 0;
	}
	else
	{
		// Compute the total area under the normal distribution curve
		//float integrated_normalizer = integral(normalDistribution, 0, furthest_z, INTEGRAL_STEPS, z_k_star, sig_hit*sig_hit);
		float in2 = normalDistribution(z_k_star, z_k_star, sig_hit*sig_hit);
		//float eta = 1/integrated_normalizer;
		float eta2 = 1/in2;
		float nd = normalDistribution(z_k, z_k_star, sig_hit*sig_hit);
		// Alternative to above: float nd2 = integral(normalDistribution, 0, z_k, INTEGRAL_STEPS, z_k_star, sig_hit*sig_hit);
		float p_hit;
		if(!std::isfinite(eta2))
		{
			p_hit = 0; // Otherwise p_hit ends up being not a number; don't worry, all this means is that p_hit is nothing
		}
		else
		{
			p_hit = eta2*nd;
		}

		if(p_hit < 0 || p_hit > 1) //!std::isfinite(p_hit))
		{
			ROS_WARN("WARNING: Potential improper p_hit value!");
			ROS_INFO("Measured Pt.: (%f, %f, %f)", meas_point.x, meas_point.y, meas_point.z);
			ROS_INFO("Intersection Pt.: (%f, %f, %f)", intersection_point.x, intersection_point.y, intersection_point.z);
			ROS_INFO("Distance to measured point z_k = %f", z_k);
			ROS_INFO("Distance to intersection point z_k_star = %f", z_k_star);
			ROS_INFO("Sig_hit = %f  furthest_z = %f", sig_hit, furthest_z);
			ROS_INFO("Integrated normalizer = %f", in2);
			ROS_INFO("eta2 is %f", eta2);
			ROS_INFO("nd is %f", nd);
			ROS_INFO("P_hit is %f \n", p_hit);
		}
		return p_hit;
	}
}

/*
Function: SensorModel::p_short
Input: A single measurement (point), as well as the location of the sensor and the map of the test fixture
Output: Returns the probability that the measurement specified was actually short of the actual object
Notes: Calls for lam_short, within the SensorModel instantiation object
*/
float SensorModel::p_short(Point meas_point, Point sensor_origin, MapFixture * m)
{
	// Ray trace from the origin to the measurement point to find where it intersects the map
	// This point is the location of where the meas_point 'should' have been
	Point intersection_point = m->rayTrace(sensor_origin, meas_point);

	// Compute the magnitude distance from the sensor to the measurment point
	float z_k = vectorLength(sensor_origin, meas_point);

	// Compute the magnitude distance from the sensor to the intersection point
	float z_k_star = vectorLength(sensor_origin, intersection_point);

	if(!std::isfinite(z_k_star))
	{
		ROS_WARN("P_SHORT reports that the measurement point doesn't intersect the map plane, returning 0!");
		return 0;
	}
	else
	{
		float eta = 1/(1 - exp(-lam_short*z_k_star));

		float p_short;
		if(!std::isfinite(eta))
		{
			p_short = 0; // Otherwise p_short ends up being not a number; don't worry, all this means is that p_short is nothing
		}
		else
		{
			p_short = eta * lam_short * exp(-lam_short*z_k);
		}

		if(p_short < 0 || p_short > 1)
		{
			ROS_WARN("WARNING: Potential improper p_short value!");
			ROS_INFO("Measured Pt.: (%f, %f, %f)", meas_point.x, meas_point.y, meas_point.z);
			ROS_INFO("Intersection Pt.: (%f, %f, %f)", intersection_point.x, intersection_point.y, intersection_point.z);
			ROS_INFO("Distance to measured point z_k = %f", z_k);
			ROS_INFO("Distance to intersection point z_k_star = %f", z_k_star);
			ROS_INFO("Lam_short = %f", lam_short);
			ROS_INFO("eta is %f", eta);
			ROS_INFO("P_short is %f \n", p_short);
		}
		return p_short;
	}
}

/*
Function: SensorModel::p_max
Input: A single measurement (point), as well as the location of the sensor and the map of the test fixture
Output: Returns the probability that the measurement specified was actually a maxed-out range
Notes: Calls for furthest_z, within the SensorModel instantiation object
*/
float SensorModel::p_max(Point meas_point, Point sensor_origin, MapFixture * m)
{
	// Ray trace from the origin to the measurement point to find where it intersects the map
	// This point is the location of where the meas_point 'should' have been
	Point intersection_point = m->rayTrace(sensor_origin, meas_point);

	// Compute the magnitude distance from the sensor to the measurment point
	float z_k = vectorLength(sensor_origin, meas_point);

	float p_max;
	if(z_k > furthest_z)
	{
		ROS_WARN("P MAX Found a measurement larger than the largest measurement furthest_z, that's pretty messed up.");
	}
	else
	{
		if(z_k > (1 - z_max_tol)*furthest_z)
		{
			p_max = 1;
		}
		else
		{
			p_max = 0;
		}
	}
	return p_max;
}

/*
Function: SensorModel::p_rand
Input: A single measurement (point), as well as the location of the sensor and the map of the test fixture
Output: Returns the probability that the measurement specified was actually just some random value in the range from 0 -> furthest_z
Notes: Calls for furthest_z within the SensorModel instantiation object
*/
float SensorModel::p_rand(Point meas_point, Point sensor_origin, MapFixture * m)
{
	// Ray trace from the origin to the measurement point to find where it intersects the map
	// This point is the location of where the meas_point 'should' have been
	Point intersection_point = m->rayTrace(sensor_origin, meas_point);

	// Compute the magnitude distance from the sensor to the measurment point
	float z_k = vectorLength(sensor_origin, meas_point);

	float p_rand;
	if(z_k > furthest_z)
	{
		ROS_WARN("P RAND Found a measurement larger than the largest measurement, that's pretty messed up.");
	}
	else
	{
		p_rand = 1/furthest_z;
	}
	return p_rand;
}


///////////////////////////        MAIN       ////////////////////////////
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

    // Start non ros stuff
    // Instantiate a sensor model
    SensorModel ourSensor;

	// Instantiate our map model
	MapFixture ourMap;

    if(test_active == true)
    {
    	if(waitForSubs() == false)
    	{
    		ROS_WARN("Didn't receive any publications!");
    		test_passed = false;
    	}

    	// Define the sensor as being at the origin of our 'universe', this is a basic assumption of our map model
    	Point sensor_origin(0,0,0);

    	// Set the dimensions (corner points) of the map fixture that the LIDAR will get data for
    	if(!ourMap.setCorners(Point(-2,1,1), Point(2,1,1), Point(-2,1,-1), Point(2,1,-1)))
    	{
    		ROS_WARN("FAILED TO SET CORNERS ON MAP FIXTURE");
    	}

    	std::vector<Point> test_points;
    	/*
    	test_points.push_back(Point(0, 1, 1.5));
    	test_points.push_back(Point(0.2, 0.1, 1));
    	test_points.push_back(Point(0.2, 0, 1));
    	test_points.push_back(Point(-2, 5, 3));
    	test_points.push_back(Point(-1, 6, 0));
    	test_points.push_back(Point(0.5, 0, 0.5));
		*/

    	// Iterate through all test cases
    	for(int i = 0; i < test_points.size(); i++)
    	{
    		Point intersection_point = ourMap.rayTrace(sensor_origin, test_points[i]);
    		
    		ROS_INFO("The intersection of the line from (%f, %f, %f) to (%f, %f, %f),", 
    			sensor_origin.x, sensor_origin.y, sensor_origin.z, test_points[i].x, test_points[i].y, test_points[i].z);
    		ROS_INFO("is located at the point (%f, %f, %f).", intersection_point.x, intersection_point.y,intersection_point.z);
    	}

    	// Create a model of the sensor (high-level command)
    	if(ourSensor.createModel(&g_point_cloud_data, &ourMap, &sensor_origin) == false)
    	{
    		ROS_WARN("Failed to model sensor!");
    		test_passed = false;    	
    	}

    }

    // Check this-node functional testing and report to user
    if(test_passed == false)
    {
    	ROS_WARN("FUNCTION TEST FAILED: Something is borked");
    }
    else
    {
    	ROS_INFO("FUNCTION TEST PASSED");
    	nh_ptr->setParam("/ispl/success", true);
    }

    // Publish found parameters
    nh_ptr->setParam("/ispl/z_hit", ourSensor.z_hit);
    nh_ptr->setParam("/ispl/z_short", ourSensor.z_short);
    nh_ptr->setParam("/ispl/z_max", ourSensor.z_max);
    nh_ptr->setParam("/ispl/z_rand", ourSensor.z_rand);
    nh_ptr->setParam("/ispl/sig_hit", ourSensor.sig_hit);
    nh_ptr->setParam("/ispl/lam_short", ourSensor.lam_short);

    // Tell test node that we are done
    nh_ptr->setParam("/learning_done", true);
   
    return 0;
}