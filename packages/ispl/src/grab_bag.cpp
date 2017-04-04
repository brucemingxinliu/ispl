// Records rosbags of point clouds to file for ISPL processing
// Created Mar 30 2017 by Trent Ziemer
// Last updated Apr 1 by Trent Ziemer

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// File IO for C++
#include <iostream>
#include <fstream>

// Define shorthand names for PCL points and clouds
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::NodeHandle * nh_ptr;
bool g_cloud_received;
PointCloud g_point_cloud_data;
int cloud_number;

// Waits for GLOBALLY DEFINED BOOLEAN to become true, triggered by specific ros topics
bool waitForSubs()
{
    int count = 0;
    int time_to_wait = 9;
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

int g_cloud_to_get;
int g_cloud_counter;

void cloudCB(const PointCloud::ConstPtr& point_cloud)
{
    g_cloud_counter++;
    if(g_cloud_counter == g_cloud_to_get)
    {
        ROS_INFO("CLOUD CB CALLED");
        int cloud_size = point_cloud->points.size();
        if(cloud_size == 0)
        {
            ROS_WARN("CLOUD CB found a cloud of size 0!");
            return;
        }
        else
        {
            ROS_INFO("CLOUD CB found a cloud of size %d!", cloud_size);
        }
        /* FOR BAG 1
        float min_x = 0.69;
        float max_x = 0.74;
        float min_y = -0.134;
        float max_y = 0.47;
        float min_z = 0.46;
        float max_z = 1.122;
        */
        /* FOR BAG 2
        float min_x = 0.564;
        float max_x = 0.74;
        float min_y = -0.134;
        float max_y = 0.47;
        float min_z = 0.46;
        float max_z = 1.122;
*/  
        /* FOR BAG 3 OR 4 */
        float min_x = 0.2; // CHANGE TO 0.55
        float max_x = 0.63;
        float min_y = -0.11;
        float max_y = 0.5;
        float min_z = 0.48;
        float max_z = 1.122;
        
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
                            g_point_cloud_data.push_back(point_cloud->points[i]);
                        }
                        counter++;
                    }
                }
            }
        }

        if(g_point_cloud_data.size() > 0)
        {
            std::ofstream output_data;
            std::string name = "/home/mordoc/point_cloud.txt";
            output_data.open(name.c_str());
            ROS_INFO("OUTPUT DATA...%s", name.c_str());
            if(output_data.is_open())
            {
                ROS_INFO("IS OPEN: %lu", g_point_cloud_data.points.size());
            }
            else
            {
                ROS_INFO("IS NOT OPEN");
            }
            for(int i = 0; i < g_point_cloud_data.points.size(); i++)
            {
                output_data << g_point_cloud_data.points[i].x << " " << g_point_cloud_data.points[i].y << " " << g_point_cloud_data.points[i].z << std::endl;
            }
            cloud_number++;     
        }
        else
        {
            ROS_WARN("PROCESSED A CLOUD DOWN TO 0 PTS");
        }

        g_cloud_received = true;
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"grab_bag");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;
    cloud_number = 0;
    g_cloud_to_get = 13;
    g_cloud_counter = 0;
    g_cloud_received = false;

    ros::Subscriber meas_sub = nh.subscribe("/pc", 1, cloudCB);

    ROS_INFO("Starting BAG GRABBER");
    if(waitForSubs())
    {
        ROS_INFO("GOT A CALLBACK SUCCESS");
    }
    else
    {
        ROS_WARN("FAILURE TO GET CALLBACK SUCCESS");
    }

    while(ros::ok())
    {

    }

    return 0;
}