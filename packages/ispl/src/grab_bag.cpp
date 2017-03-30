// Records rosbags of point clouds to file for ISPL processing
// Created Mar 30 2017 by Trent Ziemer
// Last updated Mar 30 by Trent Ziemer

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Define shorthand names for PCL points and clouds
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::NodeHandle * nh_ptr;
bool g_cloud_received;
PointCloud g_point_cloud_data;
// Waits for GLOBALLY DEFINED BOOLEAN to become true, triggered by specific ros topics
bool waitForSubs()
{
    int count = 0;
    int time_to_wait = 5;
    ros::Rate count_rate(1);

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

void cloudCB(const PointCloud::ConstPtr& point_cloud)
{
    int cloud_size = point_cloud->points.size();

    float min_x = 0.69;
    float max_x = 0.74;
    float min_y = -0.134;
    float max_y = 0.47;
    float min_z_plane = 0.46;
    float max_z_plane = 1.122;
    int filtering_constant = 1;
    int counter = 0;
    for(int i = 0; i < cloud_size; i++)
    {
        if ((point_cloud->points[i].z < max_z_plane) && (point_cloud->points[i].z > min_z_plane))
        {
            if ((point_cloud->points[i].y < max_y) && (point_cloud->points[i].y > min_y))
            {
                if ((point_cloud->points[i].x < max_x) && (point_cloud->points[i].x > min_x))
                {
                    if(counter % filtering_constant == 0)
                    {
                        g_point_cloud_data.push_back(point_cloud->points[i]);       
                    }
                    counter++;
                }
            }
        }
    }
    g_cloud_received = true;
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"grab_bag");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    g_cloud_received = false;

    ros::Subscriber meas_sub = nh.subscribe("/front_wobbler/point_cloud", 1, cloudCB);

    ROS_INFO("Starting BAG GRABBER");
    waitForSubs();

    while(ros::ok())
    {

    }

    return 0;
}