// Internal Sensor Parameter Learning node
// Created Mar 9 2017 by Trent Ziemer
// Last updated Mar 21 2017 by Trent Ziemer

// Used for numerical integration of Gaussian functions
#define INTEGRAL_STEPS 1000
#define PI 3.159265

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

// Checks if two values are within a specified relative tolerance
// This is used in the main algorithm to check if the new value has changed from the old value. 
// If not, then the variable has converged. Also, this deals with some 0/NaN values.
bool checkConvergence(float new_val, float old_val, float tolerance)
{
	float difference = new_val - old_val;
	// If we know that the change in the two values is identically 0, we can just return that it converged, because if we did the calculation it would be NaN.
	if (difference == 0)
	{
		return true;
	}
	else if (!std::isfinite(difference))
	{
		ROS_WARN("Checked convergence on a non-finite value (old or new)! Failed!");
		return false;
	}
	return (fabs(difference/new_val) < tolerance);
}

// TO DO: ADD COMMENT HERE
bool normalized(float a, float b, float c, float d)
{
	float tol = 0.001;
	float sum = a + b + c + d;
	if(sum > (1 + tol) || sum < (1 - tol))
	{
		return false;
	}
	else
	{
		return true;
	}
}

// TO DO: ADD COMMENT HERE
float computeDistance(Point origin, Point measured, Point intersection)
{
	float measured_distance = vectorLength(origin, measured);

	float expected_distance = vectorLength(origin, intersection);

	float relative_distance = measured_distance/expected_distance;

	return relative_distance;
}
