// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.hpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class header file
 * @version 1.1
 * @date 2023-05-02
 */
#ifndef LANE_KEEPING_SYSTEM_HPP_
#define LANE_KEEPING_SYSTEM_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <xycar_msgs/xycar_motor.h>
#include <yolov3_trt_ros/ObjectID.h>
#include <yaml-cpp/yaml.h>
#include <string>

#include <sensor_msgs/LaserScan.h>

#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"
#include "LaneKeepingSystem/MovingAverageFilter.hpp"
#include "LaneKeepingSystem/PIDController.hpp"
// // stanley
// #include "LaneKeepingSystem/StanleyController.hpp"

namespace Xycar {
/**
 * @brief Lane Keeping System for searching and keeping Hough lines using Hough, Moving average and PID control
 *
 * @tparam Precision of data
 */
template <typename PREC>
class LaneKeepingSystem
{
public:
    using Ptr = LaneKeepingSystem*;                                     ///< Pointer type of this class
    using ControllerPtr = typename PIDController<PREC>::Ptr;            ///< Pointer type of PIDController
    using FilterPtr = typename MovingAverageFilter<PREC>::Ptr;          ///< Pointer type of MovingAverageFilter
    using DetectorPtr = typename HoughTransformLaneDetector<PREC>::Ptr; ///< Pointer type of LaneDetector
    // using StlcontrolPtr = typename StanleyController<PREC>::Ptr;
    static constexpr int32_t kXycarSteeringAangleLimit = 50; ///< Xycar Steering Angle Limit
    static constexpr double kFrameRate = 33.0;               ///< Frame rate
    int l_chk;
    int r_chk;
    /**
     * @brief Construct a new Lane Keeping System object
     */
    LaneKeepingSystem();

    /**
     * @brief Destroy the Lane Keeping System object
     */
    virtual ~LaneKeepingSystem();

    /**
     * @brief Run Lane Keeping System
     */
    void run();

    cv::Mat calibrateImage(const cv::Mat& frame, const cv::Mat& mtx, const cv::Mat& dist, const cv::Mat& calMtx, const cv::Rect& calRoi);
    // cv::Mat imageProcessing(const cv::Mat& image, int lowThresholdValue);
    // void detectStopline(const cv::Mat& calImage, int lowThresholdValue);

    /**
     * @brief Send signal to houghtransformLaneDetctor
    */
    void sender(bool leftDetector, bool rightDetector, DetectorPtr ptr);

private:
    /**
     * @brief Set the parameters from config file
     *
     * @param[in] config Configuration for searching and keeping Hough lines using Hough, Moving average and PID control
     */
    void setParams(const YAML::Node& config);

    /**drive
     * @brief Control the speed of xycar
     *
     * @param[in] steeringAngle Angle to steer xycar. If over max angle, deaccelerate, otherwise accelerate
     */
    void speedControl(PREC steeringAngle);

    /**
     * @brief publish the motor topic message
     *
     * @param[in] steeringAngle Angle to steer xycar actually
     */
    // void drive(PREC steeringAngle);
    // void drive(PREC steeringAngle, int32_t countNonZero, PREC objectAngle=0, PREC objectSpeed=0);
    void drive(PREC steeringAngle, int32_t countNonZero, int l_chk, int r_chk);

    /**
     * @brief Callback function for image topic
     *
     * @param[in] message Image topic message
     */
    void imageCallback(const sensor_msgs::Image& message);

    //라이다
    /**
     * @brief Callback function for image topic
     *
     * @param[in] scan_in lidar topic message
     */

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

    std::pair<float, float> lidarDetector();

    void objectCallback(const yolov3_trt_ros::ObjectID::ConstPtr& object_in);


private:
    ControllerPtr mPID;                      ///< PID Class for Control
    ControllerPtr curvePID;
    FilterPtr mMovingAverage;                ///< Moving Average Filter Class for Noise filtering
    DetectorPtr mHoughTransformLaneDetector; ///< Hough Transform Lane Detector Class for Lane Detection
    // StlcontrolPtr mStanleyController;

    // ROS Variables
    ros::NodeHandle mNodeHandler;          ///< Node Hanlder for ROS. In this case Detector and Controler
    ros::Publisher mPublisher;             ///< Publisher to send message about
    ros::Subscriber mSubscriber;           ///< Subscriber to receive image
    std::string mPublishingTopicName;      ///< Topic name to publish
    std::string mSubscribedTopicName;      ///< Topic name to subscribe
    uint32_t mQueueSize;                   ///< Max queue size for message
    xycar_msgs::xycar_motor mMotorMessage; ///< Message for the motor of xycar

    ros::Subscriber mSubscriber_lidar;     //라이다
    ros::Subscriber mSubscriber_Object;
    ros::Publisher mPublisher_lidar;       //라이다
    std::string mSubscribedTopicName_lidar;//라이다
    std::string mPublishingTopicName_lidar;//라이다
    std::string mSubscribedTopicName_Object;
    uint32_t mQueueSize_lidar;             //라이다
    uint32_t mQueueSize_Object;
    int mObjectID;

    // OpenCV Image processing Variables
    cv::Mat mFrame; ///< Image from camera. The raw image is converted into cv::Mat

    // Xycar Device variables
    PREC mXycarSpeed;                 ///< Current speed of xycar
    PREC mXycarMaxSpeed;              ///< Max speed of xycar
    PREC mXycarMinSpeed;              ///< Min speed of xycar
    PREC mXycarSpeedControlThreshold; ///< Threshold of angular of xycar
    PREC mAccelerationStep;           ///< How much would accelrate xycar depending on threshold
    PREC mDecelerationStep;           ///< How much would deaccelrate xycar depending on threshold
    PREC mCteParams;
    // Debug Flag
    bool mDebugging; ///< Debugging or not
    bool mStopFlag;
    // double stanleyAngle;

    PREC obstacle_threshold;
    PREC static_threshold;

    bool obs_flag;
};
} // namespace Xycar

#endif // LANE_KEEPING_SYSTEM_HPP_
