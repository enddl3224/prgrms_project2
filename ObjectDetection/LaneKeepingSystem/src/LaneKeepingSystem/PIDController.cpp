// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file PIDController.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief PID Controller Class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include <cmath>
#include <algorithm>
#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(int32_t errorFromMid)
{
    PREC castError = static_cast<PREC>(errorFromMid);
    
    float straight_mPropertionalGain;

    mDifferentialGainError = castError - mProportionalGainError;
    mProportionalGainError = castError;
    mIntegralGainError += castError;
    // 직선일경우
    if (std::abs(castError) < 66)  //회전반경
    {   
        straight_mPropertionalGain = mProportionalGain / 2;
        //return straight_mPropertionalGain * mProportionalGainError + mDifferentialGain * mDifferentialGainError;
        return straight_mPropertionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError;
    }
    mIntegralGainError = std::max(mIntegralGainError,static_cast<PREC>(500.0f));
    return mProportionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError + mDifferentialGain * mDifferentialGainError;
}

template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar