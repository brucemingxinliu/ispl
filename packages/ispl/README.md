# ispl

ISPL: Internal Sensor Parameter Learning
Algorithm implementation for Algorithmic Robotics class taught by Dr. "Cenk" Cavusoglu @ Case Western Reserve University Spring 2017

This ROS package will analyze LIDAR, SONAR, or other range-finding sensors for scan data and attempt to learn the internal model parameters, according to the algorithms demonstrated in Probabilistic Robotics by Thrun, Burgard, and Fox (pg 131, sec 6.3.2).

Created and maintained by Trent Ziemer and Mingxin Liu

## Example usage
roslaunch ispl ispl.launch

## Running tests/demos
Can grab point cloud using rosrun ispl grab_bag /front_wobbler/point_cloud:=/pc
Could possible run learn_params without testing (or output) using rosrun ispl learn_params
