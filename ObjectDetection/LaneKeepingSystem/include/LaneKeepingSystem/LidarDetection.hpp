#ifndef LIDAR_DETECTING_SYSTEM_HPP_
#define LIDAR_DETECTING_SYSTEM_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <xycar_msgs/xycar_motor.h>
#include <yaml-cpp/yaml.h>

namespace Xycar {
class LidarDetectingSystem
{
public:
    void ScanCallback(sont sensor_msgs::LaserScan::ConstPtr& scan);
    std::pair<float, float> GetMinValue(std::vector<float> ranges)
}
}