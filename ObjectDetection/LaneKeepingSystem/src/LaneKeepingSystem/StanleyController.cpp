// Copyright (C) 2023 Grepp CO.
// All rights reserved.

#include "LaneKeepingSystem/StanleyController.hpp"

namespace Xycar {

template <typename PREC>
void StanleyController<PREC>::setConfiguration(const YAML::Node& config)
{
    mGain = config["STANLEY"]["STANLEY_GAIN"].as<int32_t>();
    mLookAheadDistance = config["STANLEY"]["LOOK_AHEAD"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
}

// 차량의 현재 위치와 목표 지점을 기반으로 궤적 오차(cross-track error)를 계산하는 함수
template <typename PREC>
PREC StanleyController<PREC>::calculateCrossTrackError(const Point2D& currentPos, const Point2D& targetPos)
{
    // 궤적 오차는 현재 위치와 목표 지점 간의 거리로 정의될 수 있습니다.
    // Euclidean 거리를 사용
    PREC dx = targetPos.x - currentPos.x;
    PREC dy = targetPos.y - currentPos.y;
    return std::sqrt(dx * dx + dy * dy);
}

// 차량의 현재 헤딩과 목표 헤딩을 기반으로 헤딩 오차(heading error)를 계산하는 함수
template <typename PREC>
PREC StanleyController<PREC>::calculateHeadingError(PREC currentHeading, PREC targetHeading)
{
    // 헤딩 오차는 현재 헤딩과 목표 헤딩의 차이로 정의될 수 있습니다.
    // 두 헤딩 값의 차이
    return targetHeading - currentHeading;
}

template <typename PREC>
PREC StanleyController<PREC>::normalizeAngle(PREC angle)
{
    // 주어진 각도를 [-π, π] 범위 내로 정규화합니다.
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

template <typename PREC>
void StanleyController<PREC>::calculateSteeringAngle(PREC crossTrackError, PREC headingError, PREC velocity)
{
    // Calculate the cross-track error (cte) compensation
    PREC alpha = std::atan2(-mGain * crossTrackError, velocity);

    // Calculate the desired heading angle
    PREC desiredHeading = this->normalizeAngle(headingError) + alpha;

    // Calculate the steering angle using the desired heading and look-ahead distance
    PREC steeringAngle = std::atan2(2 * mLookAheadDistance * std::sin(desiredHeading), velocity);

    this->mResult = steeringAngle;
}

template <typename PREC>
PREC StanleyController<PREC>::StanleyControl(int32_t leftPositionX, int32_t rightPositionX, PREC steeringAngle, PREC mXycarSpeed)
{
    // 현재 위치와 목표 지점을 설정
    Point2D currentPos(leftPositionX, mROIHeight/2);
    Point2D targetPos(rightPositionX, mROIHeight/2);

    // 현재 헤딩과 목표 헤딩을 설정
    PREC currentHeading = 0;
    PREC targetHeading = steeringAngle;

    // crossTrackError와 headingError 계산
    PREC crossTrackError = calculateCrossTrackError(currentPos, targetPos);
    PREC headingError = calculateHeadingError(currentHeading, targetHeading);

    // Stanley 컨트롤러 객체 생성
    StanleyController<PREC> stanleyController;

    // 조향 각도 계산
    PREC velocity = mXycarSpeed; // 차량 속도 설정
    stanleyController.calculateSteeringAngle(crossTrackError, headingError, velocity);

    // 계산된 조향 각도 출력
    steeringAngle = stanleyController.getResult

    return steeringAngle;
}

template class StanleyController<float>;
template class StanleyController<double>;
} // namespace Xycar