#include <algorithm>

#include "LaneKeepingSystem/LidarDetecion.hpp"

std::vector<float> distance;

namespace Xycar {
template <typename PREC>
void LidarDetectingSystem<PREC>::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(i = 0; i < scan->ranges.size(); i++){
        distance[i] = scan->ranges[i];
    }
    // std::cout<<distance<<std::endl;
    // std::cout<<distance.size()<<std::endl;
}

template <typename PREC>
std::pair<float, float> LidarDetectingSystem<PREC>::GetMinValue(std::vector<float> ranges)
{
    std::vector<float> leftRange = distance[:(distance.size() * 1) / 6];
    std::vector<float> rightRange = distance[(distnace.size() * 5) / 6:];

    leftRange.remove(leftRange.begin(), leftRange.end(), 0.0);
    rightRange.remove(rightRange.begin(), rightRange.end(), 0.0);

    float left_min = * __std_min_element(leftRange.begin(), leftRange.end());
    float right_min = * __std_min_element(rightRange.begin(), rightRange.end());

    return {left_min, right_min};
}
}