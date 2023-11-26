// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class source file
 * @version 1.1
 * @date 2023-05-02
 */


#include "LaneKeepingSystem/LaneKeepingSystem.hpp"


namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    curvePID = new PIDController<PREC>(config["PID"]["CURVE_P_GAIN"].as<PREC>(), config["PID"]["CURVE_I_GAIN"].as<PREC>(), config["PID"]["CURVE_D_GAIN"].as<PREC>());
    mPID = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = new MovingAverageFilter<PREC>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mHoughTransformLaneDetector = new HoughTransformLaneDetector<PREC>(config);
    // // stanley
    // mStanleyController = new StanleyController<PREC>(config);
    setParams(config);

    mObjectID = -1;
    mStopFlag = false;

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
    mSubscriber_lidar = mNodeHandler.subscribe(mSubscribedTopicName_lidar, mQueueSize_lidar, &LaneKeepingSystem::lidarCallback, this);
    mSubscriber_Object = mNodeHandler.subscribe(mSubscribedTopicName_Object, mQueueSize, &LaneKeepingSystem::objectCallback, this);
}


template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();

    mSubscribedTopicName_lidar= config["TOPIC"]["SUB_LIDAR"].as<std::string>();
    mQueueSize_lidar = config["TOPIC"]["QUEUE_SIZE_LIDAR"].as<uint32_t>();

    mSubscribedTopicName_Object = "/object_detect";
    // mSubscribedTopicName_Object = config["TOPIC"]["SUB_OBJECT"].as<std::string>();
    mQueueSize_Object = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    mCteParams = config["CTE"]["CTE_ERROR"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();

    obstacle_threshold = config["LIDAR"]["OBS_THRESHOLD"].as<PREC>();
    static_threshold = config["LIDAR"]["STATIC_THRESHOLD"].as<PREC>();
    //obs_flag = false;
}

template <typename PREC>
LaneKeepingSystem<PREC>::~LaneKeepingSystem()
{
    delete mPID;
    delete curvePID;
    delete mMovingAverage;
    delete mHoughTransformLaneDetector;
    // // stanley
    // delete mStanleyController;
}

template <typename PREC>
cv::Mat LaneKeepingSystem<PREC>::calibrateImage(const cv::Mat& frame, const cv::Mat& mtx, const cv::Mat& dist, const cv::Mat& calMtx, const cv::Rect& calRoi) {
    cv::Mat tfImage;
    cv::undistort(frame, tfImage, mtx, dist, calMtx);
    cv::Mat roi = tfImage(calRoi);
    cv::resize(roi, roi, frame.size());
    return roi;
}

// template <typename PREC>
// void LaneKeepingSystem<PREC>::detectStopline(const cv::Mat& calImage, int lowThresholdValue) {
//     cv::Rect roiRect(calImage.cols / 2 - 125, 350, 250, 10);
//     cv::Mat stoplineRoi = calImage(roiRect);
//     cv::Mat image = imageProcessing(stoplineRoi, lowThresholdValue);
//     int countNonZero = cv::countNonZero(image);
//     if (countNonZero > 2000) {
//         std::cout << countNonZero << std::endl;
//         std::cout << "stopline" << std::endl;
//     }
// }

// template <typename PREC>
// void LaneKeepingSystem<PREC>::sender(bool leftDetector, bool rightDetector, DetectorPtr ptr)
// {
//     ptr->oneLaneByLeft = leftDetector;
//     ptr->oneLaneByRight = rightDetector;
// }


bool static_flag = false;
template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
   
    ros::Rate rate(kFrameRate);
    int i = 0;
    int idx = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        if (mFrame.empty())
            continue;

          
        // cv::imshow("origin", mFrame);

        // //이미지 저장
        // std::cout<<"capture"<<std::endl;
        // cv::imwrite(std::to_string(i)+".jpg",mFrame);
        // if (i % 30 == 0){
        //     cv::imwrite("/home/nvidia/xycar_ws/src/LaneKeepingSystem/src/LaneKeepingSystem/output/" + std::to_string(idx)+ ".jpg" ,mFrame);
        //     idx++;
        //     std::cout << "imwrite" <<std::endl;
        // }
        // i++;

        int32_t leftPositionX, rightPositionX, countNonZero;
        
    //     bool obs_flag = false;
         const auto [left_min, right_min] = lidarDetector();
    //     // const auto [leftPositionX, rightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame);

        std::tie(leftPositionX, rightPositionX, countNonZero, l_chk, r_chk) = mHoughTransformLaneDetector->getLanePosition(mFrame);
        std::cout << "l_chk: " << l_chk << "r_chk: " << r_chk << std::endl;

         if (left_min < obstacle_threshold && right_min < obstacle_threshold){
            std::cout<<"dynamic"<<std::endl;
            ros::Duration(3).sleep();
        }
        else if(left_min < static_threshold){
            std::cout<<"left_obstacle"<<std::endl;

            mMovingAverage->addSample(static_cast<int32_t>(rightPositionX+50));
            int32_t estimatedPositionX = static_cast<int32_t>(mMovingAverage->getResult());
            int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);

        }
        else if(right_min < static_threshold){
            std::cout<<"right_obstacle"<<std::endl;

            mMovingAverage->addSample(static_cast<int32_t>(leftPositionX)-50);
            int32_t estimatedPositionX = static_cast<int32_t>(mMovingAverage->getResult())+10;
            int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);

        }
        else{
            mMovingAverage->addSample(static_cast<int32_t>((leftPositionX + rightPositionX) / 2) + 5); 
            int32_t estimatedPositionX = static_cast<int32_t>(mMovingAverage->getResult());
            // std::cout << "estimagedPositionX: " << estimatedPositionX << std::endl;
                int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
        }

    // std::cout << countNonZero << std::endl;
        
    // mfiltering 반환: (leftPositionX + rightPositionX) / 2 == newSample임
        mMovingAverage->addSample(static_cast<int32_t>((leftPositionX + rightPositionX) / 2) + 5); 
        
        int32_t estimatedPositionX = static_cast<int32_t>(mMovingAverage->getResult());
        // std::cout << "estimagedPositionX: " << estimatedPositionX << std::endl;
        
        int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
        
        
        PREC steeringAngle = std::max(static_cast<PREC> (-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mPID->getControlOutput(errorFromMid)), static_cast<PREC> (kXycarSteeringAangleLimit)));
        // 기존 코드
        speedControl(steeringAngle);
        drive(steeringAngle, countNonZero, l_chk, r_chk);
        //sender(leftDetector, rightDetector, mHoughTransformLaneDetector);

        // speedControl(errorFromMid);
        // // 추가
        // double stanleyAngle = mStanleyController->StanleyControl(leftPositionX, rightPositionX, errorFromMid, mXycarSpeed);
        // drive(stanleyAngle, countNonZero);
        // sender(leftDetector, rightDetector, mHoughTransformLaneDetector);


        if (mDebugging)
        {
            // std::cout << "lpos: " << leftPositionX << ", rpos: " << rightPositionX << ", mpos: " << estimatedPositionX << std::endl;
            mHoughTransformLaneDetector->drawRectangles(leftPositionX, rightPositionX, estimatedPositionX);
            // cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());
            // cv::imshow("roi", mHoughTransformLaneDetector->getDebugROI());
            cv::waitKey(1);
        }
        // rate.sleep();
    }

}



template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);

    cv::Mat mtx = (cv::Mat_<double>(3, 3) << 366.668911, 0.000000, 332.515277,
                                             0.000000, 366.249932, 211.313860,
                                             0.000000, 0.000000, 1.000000);
    cv::Mat dist = (cv::Mat_<double>(1, 5) << -0.348425, 0.111577, -0.007641, 0.002064, 0.000000);

    cv::Mat cal_mtx;
    cv::Rect cal_roi;
    cv::Size size = mFrame.size();
    cv::getOptimalNewCameraMatrix(mtx, dist,  size, 1, size, &cal_roi);
    // cal_mtx = cv::getOptimalNewCameraMatrix(mtx, dist,  size, 1, size, &cal_roi);
    // cv::Mat map1, map2;
    // cv::initUndistortRectifyMap(mtx, dist, cv::Mat(), mtx, size, cv::CV_8UC3, map1, map2);
    // cv::Mat calImage;
    // cv::remap(mFrame, calImage, map1, map2, cv::INTER_LINEAR);
    cv::Mat calImage = calibrateImage(mFrame, mtx, dist, cal_mtx, cal_roi);
    int alpha = 1;
    calImage = (calImage * 2) - (40 * alpha);
    mFrame = calImage;

    // cv::imshow("cali", mFrame);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    float yaw = std::abs(steeringAngle);
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        //std::cout << "DecelerationsStep : " << mDecelerationStep << '\n';
        mXycarSpeed -= (mDecelerationStep) * ((exp(yaw/ 20 - 0.5)) - pow(1.3, yaw / 20 - 0.5));;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);

        return;
    }
    //std::cout << "AccelerationStep : " << mAccelerationStep << '\n';
    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}


std::vector<float> right_ranges;
std::vector<float> left_ranges;
template <typename PREC>
void LaneKeepingSystem<PREC>::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    // float min_x = -270;  // x 축 최소 값
    // float max_x = -3;   // x 축 최대 값
    // float min_y = -130;  // y 축 최소 값
    // float max_y = 130;   // y 축 최대 값

    float min_x = -45;  // x 축 최소 값
    float max_x = -3;   // x 축 최대 값
    float min_y = -40;  // y 축 최소 값
    float max_y = 40;   // y 축 최대 값

    std::vector<float> ranges = scan_in->ranges;
    std::vector<float> tmp_right_ranges;
    std::vector<float> tmp_left_ranges;

    for (int i = 0; i < ranges.size(); ++i)
    {
        float angle = scan_in->angle_min + i * scan_in->angle_increment;
        float x = ranges[i] * cos(angle) *100;  // 레이저 데이터의 x 좌표 계산
        float y = ranges[i] * sin(angle) *100;  // 레이저 데이터의 y 좌표 계산

        
        
        if (x >= min_x && x <= max_x && y >= min_y && y < 0)
        {
            // std::cout<<"left"<<std::endl;
            tmp_left_ranges.push_back(ranges[i]);
            
        }
        else if (x >= min_x && x <= max_x && y >= 0 && y <= max_y)
        {

            // std::cout<<"right"<<std::endl;
            tmp_right_ranges.push_back(ranges[i]);

        }

    }

    right_ranges = tmp_right_ranges;
    left_ranges = tmp_left_ranges;


    
}

template <typename PREC>
std::pair<float, float> LaneKeepingSystem<PREC>::lidarDetector()
{
    
    float left_min = 1000.0;
    float right_min = 1000.0;

    if(left_ranges.size()>0){
        remove(left_ranges.begin(), left_ranges.end(),0.0); 
        left_min = (* std::min_element(left_ranges.begin(),left_ranges.end()))*100;
    }

    if(right_ranges.size()>0)
    {
        remove(right_ranges.begin(), right_ranges.end(),0.0);
        right_min = (* std::min_element(right_ranges.begin(),right_ranges.end()))*100;
    }

    std::cout<<"left_min: "<<left_min<<" right_min: "<<right_min<<std::endl;

    return {left_min, right_min};
}



template <typename PREC>
void LaneKeepingSystem<PREC>::objectCallback(const yolov3_trt_ros::ObjectID::ConstPtr& object_in) {
    mObjectID = object_in->object_id;
}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle, int32_t countNonZero, int l_chk, int r_chk)
{
    std::cout << "drive function: "<<l_chk <<r_chk<< std::endl;
    // objectID defaul value is -1 when object not founded
    // 0 crosswalk
    // 2 left
    // 3 right
    // 4 stop
    // 5 car
    // 6 traffic light green
    // 7 traffic light red
    // 8 traffic light yellow

    // mObjectID == LABLES[]
    ros::Time begin = ros::Time::now();

    if (mObjectID == 0 || mObjectID == 4 || mObjectID == 7 || mObjectID == 8)
    {
        mStopFlag = true;
    }

    xycar_msgs::xycar_motor motorMessage;
    if (countNonZero >= 900) 
    {
        if (!mStopFlag)
        {
            motorMessage.angle = std::round(steeringAngle);
            motorMessage.speed = std::round(mXycarSpeed);
            mPublisher.publish(motorMessage);

            return;
        }
        if (mObjectID == 0 || mObjectID == 4) {
            std::cout << "cross walk or stop" << std::endl;
            motorMessage.angle = std::round(steeringAngle);
            motorMessage.speed = 0.0;
            mPublisher.publish(motorMessage);
            ros::Duration(5).sleep();
            mStopFlag = false;
        }
        else if (mObjectID == 7 || mObjectID == 8) {
            std::cout << "red of yellow" << std::endl;
            motorMessage.angle = std::round(steeringAngle);
            motorMessage.speed = 0.0;
            mPublisher.publish(motorMessage);
            ros::Duration(1).sleep();
            mStopFlag = false;
        }

        else {
            motorMessage.angle = std::round(steeringAngle);
            motorMessage.speed = std::round(mXycarSpeed);
            mPublisher.publish(motorMessage);
        }
    }
    else if(mObjectID == 2)
    {
        std::cout << "turn left" << std::endl;

        if(r_chk == -1){
            motorMessage.angle = std::round(-100);
            motorMessage.speed = std::round(mXycarSpeed);
            mPublisher.publish(motorMessage);
            
            while(true){
                ros::Duration end = ros::Time::now() - begin;
                mPublisher.publish(motorMessage);
                if (end.toSec() >= 2) break;
            }
        }
    }
    else if(mObjectID == 3)
    {
        std::cout << "turn right" << std::endl;

        if(l_chk == -1){
            motorMessage.angle = std::round(100);
            motorMessage.speed = std::round(mXycarSpeed);
        
            while(true){
                ros::Duration end = ros::Time::now() - begin;
                mPublisher.publish(motorMessage);
                if (end.toSec() >= 2) break;
            }
        }
    }
    else if(mObjectID == 5)
    {
        std::cout << "car" << std::endl;
        motorMessage.angle = std::round(steeringAngle);
        motorMessage.speed = std::round(mXycarSpeed);
        mPublisher.publish(motorMessage);
    }
    else if(mObjectID == 6)
    {
        std::cout << "green" << std::endl;
        motorMessage.angle = std::round(steeringAngle);
        motorMessage.speed = std::round(mXycarSpeed);
        mPublisher.publish(motorMessage);
    }
    else
    {
        motorMessage.angle = std::round(steeringAngle);
        motorMessage.speed = std::round(mXycarSpeed);
        mPublisher.publish(motorMessage);
    }
    mObjectID = -1;
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
}// namespace Xycar
