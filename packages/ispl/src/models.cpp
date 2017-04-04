// ISPL Sensor and Map model classes library
// Created Mar 21 2017 by Trent Ziemer
// Last updated (NEEDS UPDATE) by Trent Ziemer

#include <ispl/models.h>
#include <ispl/math_functions.h>

///////////         MAP FIXTURE CLASS MEMBER FUNCTIONS       //////////
// Map constructor. Sets the "maps" thickness to a chosen value
MapFixture::MapFixture()
{
	plane_thickness_tol = 0.01;
}

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
	origin_corner = corner1;
	// These next two points are needed to define a single plane
	second_corner = corner2;
	third_corner = corner3;
	// Can be used to check our work and check that the user knows what "co-planar" means
	validation_corner = corner4;

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
	
	// If the fourth (validation corner) is found to be near enough to the plane, then setting the corners was successful
	if(validateCorner(validation_corner))
	{
		return true;
	}
	else
	{
		return false;
	}
}

// This functions check if the specified point is "on" the map (ie is near the plane). 
// Note: It's prudent to specify your plane as four points, and use the fourth point/corner as validation here before acutally using it as a map
bool MapFixture::validateCorner(Point testPoint)
{
	// The following equation for the distance from the testPoint to the MapFixture plane is a well known 3D-geometry formula
	float numerator = plane_normal.x * testPoint.x + plane_normal.y * testPoint.y + plane_normal.z * testPoint.z + plane_parameter;
	float denominator = sqrt(pow(plane_normal.x, 2) + pow(plane_normal.y, 2) + pow(plane_normal.z, 2));
	float point_to_plane_distance = numerator/denominator;

	if(point_to_plane_distance <= plane_thickness_tol)
	{
		return true;
	}
	else
	{
		return false;
		//ROS_INFO("Point not on plane found! Distance p 2 p = %f \n", point_to_plane_distance);
	}
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

	if(!std::isfinite(intersection_x) || !std::isfinite(intersection_y) || !std::isfinite(intersection_z))
	{
		ROS_WARN("INVALID INTERSECTION POINT: One of the point coordinates was inf or nan!");
	}
	return Point (intersection_x, intersection_y, intersection_z);
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

Point MapFixture::getPlaneNormal()
{
	return plane_normal;
}

float MapFixture::getPlaneParameter()
{
	return plane_parameter;
}

///////////     SENSOR MODEL CLASS MEMBER FUNCTIONS     /////////////////
SensorModel::SensorModel()
{
}

// Set the starting values of the intrinsic noise parameters that we want to have converge after applying our algorithm
bool SensorModel::setInitialParams(float z_hit_init, float z_short_init, float z_max_init, float z_rand_init, float sig_hit_init, float lam_short_init)
{
	if (z_hit_init > 0)
	{
		z_hit = z_hit_init;
	}

	if (z_short_init > 0)
	{
		z_short = z_short_init;
	}

	if (z_max_init > 0)
	{
		z_max = z_max_init;
	}

	if (z_rand_init > 0)
	{
		z_rand = z_rand_init;
	}

	if (sig_hit_init > 0)
	{
		sig_hit = sig_hit_init;
	}

	if (lam_short_init > 0)
	{
		lam_short = lam_short_init;
	}
	ROS_INFO("Setting initial model parameters = {%f, %f, %f, %f | %f, %f}", z_hit, z_short, z_max, z_rand, sig_hit, lam_short);
	return true;
}

bool SensorModel::createModel(PointCloud * point_cloud, 
								MapFixture * map_plane,
								Point * origin)
{
	// for debug?
	param_file.open("/home/mordoc/ispl_model_file.txt");
	//param_file << "z_hit" << " " << "z_short" << " " << "z_max" << " " << "z_rand" << " " << "sig_hit" << " " << "lam_short" << std::endl;
	
	int cloud_size = point_cloud->points.size();

	ROS_INFO("Modeling sensor based on %d points", cloud_size);
	for(int i = 0; i < cloud_size; i++)
	{
		//ROS_INFO("Point: x=%f, y=%f, z=%f.", point_cloud->points[i].x,point_cloud->points[i].y, point_cloud->points[i].z);
	}

    // Determine which point is the farthest away from the sensor, call this the longest range, which is thus furthest_z
    float longest_range = -1; 
    float range;
	for(int i = 0; i < cloud_size; i++)
	{
		Point meas_point = point_cloud->points[i];
		range = sqrt( (meas_point.x - origin->x)*(meas_point.x - origin->x) 
					+ (meas_point.y - origin->y)*(meas_point.y - origin->y) 
					+ (meas_point.z - origin->z)*(meas_point.z - origin->z));
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

	/*if(furthest_z < 3)
	{
		furthest_z = 3;
	}*/

	// Governs the "width" of the z_max bin (numerical impracticality-driven)
	// This seems reasonable to me. Current testing usually shows that it's identically 0, so this may be over precautious
	z_max_tol = 0.01;

	// Minimum percent that the specific parameter needs to be before the algorithm will stop trying to be more accurate
	// This represents, for example if = 0.05, a less 5% change between iterations before the learning algorithm will stop itself,
	//     or the difference between iterations is less than that absolute value (this dual meaning of the tolerance is purely for convenience)
	z_hit_conv_perc = 0.001;
	z_short_conv_perc = 0.001;
	z_max_conv_perc = 0.001;
	z_rand_conv_perc = 0.001;
	sig_hit_conv_perc = 0.001;
	lam_short_conv_perc = 0.001;

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
	int max_i = 100;

	// Holds the probabilities of each event happening as returned from p_xxx type function
	float p_hit_val;
	float p_short_val;
	float p_max_val;
	float p_rand_val;
	
	// Overall normalization value
	float eta;

	bool first_time = true;

	// Loop following until convergence criteria is met or we reach the maximum number of tries
	do
	{
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
		float e_short_ext_sum = 0;
		float altered_e_hit_sum = 0;

		for (int k = 0; k < Z->size(); k++)
		{
			p_hit_val = p_hit(data_cloud[k], sensor_origin, m);
			p_short_val = p_short(data_cloud[k], sensor_origin, m);
			p_max_val = p_max(data_cloud[k], sensor_origin, m);
			p_rand_val = p_rand(data_cloud[k], sensor_origin, m);

			Point intersection_point = m->rayTrace(sensor_origin, data_cloud[k]);
			float relative_distance = computeDistance(sensor_origin, data_cloud[k], intersection_point);
			if(!std::isfinite(relative_distance))
			{
				ROS_WARN("Record relative distance failed: non-finite!");
			}
			if(first_time == true)
			{
				param_file << relative_distance << std::endl;
			}

			eta = 1/(p_hit_val + p_short_val + p_max_val + p_rand_val);

			float e_hit_val = eta * p_hit_val;
			float e_short_val = eta * p_short_val;
			float e_max_val = eta * p_max_val;
			float e_rand_val = eta * p_rand_val;

			//ROS_INFO("e hit = %f, e short = %f, e max = %f, e rand = %f", e_hit_val, e_short_val, e_max_val, e_rand_val);

			// Compute the magnitude distance from the sensor to the measurment point
			float z_k = vectorLength(sensor_origin, data_cloud[k]);

			// Compute the magnitude distance from the sensor to the intersection point
			float z_k_star = vectorLength(sensor_origin, intersection_point);

			altered_e_hit_sum += e_hit_val*pow(z_k - z_k_star, 2); 
			//ROS_INFO("aehs = %f, ehv = %f, altdist = %f, altdist^2 = %f", altered_e_hit_sum, e_hit_val, z_k - z_k_star, pow(z_k - z_k_star, 2));
			// NOTE: As long as this loops index stays in 0 to size(z) order, then index of data_cloud and these four vectors will match up
			e_hit.push_back(e_hit_val);
			e_short.push_back(e_short_val);
			e_max.push_back(e_max_val);
			e_rand.push_back(e_rand_val);
		}
		first_time = false;
	
		// Compute sums...
		for(int j = 0; j < e_hit.size(); j++)
		{
			e_hit_sum += e_hit[j];
		}

		for(int j = 0; j < e_short.size(); j++)
		{
			e_short_sum += e_short[j];
		}

		for(int j = 0; j < e_max.size(); j++)
		{
			e_max_sum += e_max[j];
		}

		for(int j = 0; j < e_rand.size(); j++)
		{
			e_rand_sum += e_rand[j];
		}

		for(int j = 0; j < e_short.size(); j++)
		{
			e_short_ext_sum += e_short[j]*vectorLength(sensor_origin, data_cloud[j]);
		}

		float z_hit_old = z_hit;
		float z_short_old = z_short;
		float z_max_old = z_max;
		float z_rand_old = z_rand;
		float sig_hit_old = sig_hit;
		float lam_short_old = lam_short;
		mag_Z = e_hit_sum + e_short_sum + e_max_sum + e_rand_sum;

		// Compute each four of these parameters (at least for this iteration)
		z_hit = e_hit_sum/mag_Z;
		z_short = e_short_sum/mag_Z;
		z_max = e_max_sum/mag_Z;
		z_rand = e_rand_sum/mag_Z;

		//ROS_INFO("ehs=%f ess=%f ems=%f ers=%f sig_hit=%f lam_short=%f", e_hit_sum, e_short_sum, e_max_sum, e_rand_sum, sig_hit, lam_short);

		if(!normalized(z_hit, z_short, z_max, z_rand))
		{
			ROS_WARN("PROBLEM: z_xxx param values are not normalized together!");
		}

		// This little check avoids NaN's for sig_hit, if dividing 0 by 0
		if(altered_e_hit_sum == 0)
		{
			sig_hit = 0;
		}
		else
		{
			sig_hit = sqrt(altered_e_hit_sum/e_hit_sum);
		}
		
		// This little check avoids NaN's for lam_short, if dividing 0 by 0. Same as above, but I dunno if this one is needed
		if(e_short_sum == 0)
		{
			lam_short = 0;
		}
		else
		{
			lam_short = e_short_sum/e_short_ext_sum;
		}

		/*
		ROS_INFO("AFTER:");
		ROS_INFO("z_hit: %f", z_hit);
		ROS_INFO("z_short: %f", z_short);
		ROS_INFO("z_max: %f", z_max);
		ROS_INFO("z_rand: %f", z_rand);
		ROS_INFO("sig_hit: %f", sig_hit);
		ROS_INFO("lam_short: %f \n \n", lam_short);
		*/
		int num_conv_param = 0;
		// There's a much better way to diagnose issues here and reduce computation time, 
		//     but until then we simply check if each value has converged and if so, raise that flag to leave the do...while loop.

		if(checkConvergence(z_hit, z_hit_old, z_hit_conv_perc))
		{
			num_conv_param++;
		}

		if(checkConvergence(z_short, z_short_old, z_short_conv_perc))
		{
			num_conv_param++;
		}

		if(checkConvergence(z_max, z_max_old, z_max_conv_perc))
		{
			num_conv_param++;
		}

		if(checkConvergence(z_rand, z_rand_old, z_rand_conv_perc))
		{
			num_conv_param++;
		}

		if(checkConvergence(sig_hit, sig_hit_old, sig_hit_conv_perc))
		{
			num_conv_param++;
		}

		if(checkConvergence(lam_short, lam_short_old, lam_short_conv_perc))
		{
			num_conv_param++;
		}

		if(num_conv_param == 6)
		{
			converged = true;
		}

		//param_file << z_hit << " " << z_short << " " << z_max << " " << z_rand << " " << sig_hit << " " << lam_short << " " << std::endl;
		
		i++;
	}
	while((converged == false) && (i < max_i));

	if (i < max_i)
	{
		ROS_INFO("Converged to solution after %d iterations.", i);
		return true;
	}
	else
	{
		ROS_INFO("Failed to converge after running %d iterations.", i);
		return false;	
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
		float integrated_normalizer = integral(normalDistribution, 0, furthest_z, INTEGRAL_STEPS, z_k_star, sig_hit*sig_hit);
		//float in2 = normalDistribution(z_k_star, z_k_star, sig_hit*sig_hit);
		float eta = 1/integrated_normalizer;
		//float eta2 = 1/in2;
		float nd = normalDistribution(z_k, z_k_star, sig_hit*sig_hit);
		//ROS_INFO("Normal dist at %f with mean %f and var %f is %f", z_k, z_k_star, sig_hit*sig_hit, nd);
		// Alternative to above: 
		//float nd2 = integral(normalDistribution, 0, z_k, INTEGRAL_STEPS, z_k_star, sig_hit*sig_hit);
		float p_hit;
		if(!std::isfinite(eta))
		{
			p_hit = 0; // Otherwise p_hit ends up being not a number; don't worry, all this means is that p_hit is nothing
		}
		else
		{
			p_hit = eta*nd;
		}

		if(0)
		{
			ROS_WARN("WARNING: Potential improper p_hit value!");
			ROS_INFO("Measured Pt.: (%f, %f, %f)", meas_point.x, meas_point.y, meas_point.z);
			ROS_INFO("Intersection Pt.: (%f, %f, %f)", intersection_point.x, intersection_point.y, intersection_point.z);
			ROS_INFO("Distance to measured point z_k = %f", z_k);
			ROS_INFO("Distance to intersection point z_k_star = %f", z_k_star);
			ROS_INFO("Sig_hit = %f  furthest_z = %f", sig_hit, furthest_z);
			ROS_INFO("Integrated normalizer = %f", integrated_normalizer);
			ROS_INFO("eta is %f", eta);
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
		if (z_k > z_k_star)
		{
			return 0;
		}
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

		if(0)
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
	// Compute the magnitude distance from the sensor to the measurment point
	float z_k = vectorLength(sensor_origin, meas_point);

	float p_max;
	if(z_k > furthest_z)
	{
		ROS_WARN("P MAX Found a measurement larger than the largest measurement furthest_z, that's pretty messed up.");
		return 0;
	}
	else
	{
		if(z_k > (1 - z_max_tol)*furthest_z)
		{
			//ROS_INFO("z_k = %f, z_max_tol = %f, furthest_z = %f, so z_max = 1", z_k, z_max_tol, furthest_z);
			p_max = 1;
		}
		else
		{
			p_max = 0;
		}
	}
	//ROS_INFO("z_k = %f, z_max_tol = %f, furthest_z = %f", z_k, z_max_tol, furthest_z);
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
		return 0;
	}
	else
	{
		p_rand = 1/furthest_z;
	}
	return p_rand;
}