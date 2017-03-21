// ISPL Sensor and Map model classes library header
// Created Mar 21 2017 by Trent Ziemer
// Last updated (NEEDS UPDATE) by Trent Ziemer

// Various ROS headers
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


/////////////////////////////          MAP FIXTURE       /////////////////////
class MapFixture
{
public:
	MapFixture();
	bool setCorners(Point, Point, Point, Point);

	Point rayTrace(Point, Point);
private:
	// Point from which to conventionally measure things from wrt the plane
	Point origin_corner;
	// Other two defining points in the plane. Three points define a plane.
	Point second_corner;
	Point third_corner;
	// Technically superfluous, but can use for numerical self-validation 
	Point validation_corner;

	// This point object represents the vector that is normal to the map fixture's plane
	Point plane_normal;
	// Together with the normal, this plane_parameter fully defines the equation of a 3D plane
	float plane_parameter;

	// Max distance that a point can be from the plane and still be considered "on" the plane. Used for validation of points.
	float plane_thickness_tol;

	// Calculates the unit-length (normalized) cross product between two points, presumably in the plane but not necessarily
	Point unitCrossProduct(Point, Point);

	// Checks that the given point is close enough (by private tolerance variable) to being on the plane (should already be defined)
	bool validateCorner(Point);
};

/////////////////////////////      SENSOR MODEL       /////////////////////////
class SensorModel
{
public:
	SensorModel();
	bool createModel(PointCloud*, MapFixture *, Point *);
	bool setInitialParams(float, float, float, float, float, float);
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

	// By percentage, how converged the parameter values have to be to be considered 'close enough'
	float z_hit_conv_perc;
	float z_short_conv_perc;
	float z_max_conv_perc;
	float z_rand_conv_perc;
	float sig_hit_conv_perc;
	float lam_short_conv_perc;

	// for debug?
	std::ofstream param_file;
};