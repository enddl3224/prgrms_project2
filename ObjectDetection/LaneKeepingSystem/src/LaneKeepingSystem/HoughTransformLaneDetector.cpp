// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file HoughTransformLaneDetector.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief hough transform lane detector class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include <numeric>

#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"

namespace Xycar {

template <typename PREC>
void HoughTransformLaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();
    mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();
    mHoughLineSlopeRange = config["HOUGH"]["ABS_SLOPE_RANGE"].as<PREC>();
    mHoughThreshold = config["HOUGH"]["THRESHOLD"].as<int32_t>();
    mHoughMinLineLength = config["HOUGH"]["MIN_LINE_LENGTH"].as<int32_t>();
    mHoughMaxLineGap = config["HOUGH"]["MAX_LINE_GAP"].as<int32_t>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
std::pair<PREC, PREC> HoughTransformLaneDetector<PREC>::getLineParameters(const Lines& lines, const Indices& lineIndices) {

    uint32_t numLines = static_cast<uint32_t>(lineIndices.size());
    if (numLines == 0)
        return { 0.0f, 0.0f };

    int32_t xSum = 0;
    int32_t ySum = 0;
    PREC mSum = 0.0f;
    for (const auto lineIndex : lineIndices)
    {
        int32_t x1 = lines[lineIndex][HoughIndex::x1];
        int32_t y1 = lines[lineIndex][HoughIndex::y1];
        int32_t x2 = lines[lineIndex][HoughIndex::x2];
        int32_t y2 = lines[lineIndex][HoughIndex::y2];
        xSum += x1 + x2;
        ySum += y1 + y2;
        mSum += static_cast<PREC>((y2 - y1)) / (x2 - x1);
    }

    PREC xAverage = static_cast<PREC>(xSum) / (numLines * 2);
    PREC yAverage = static_cast<PREC>(ySum) / (numLines * 2);
    PREC m = mSum / numLines;
    PREC b = yAverage - m * xAverage;

    return { m, b };
}

template <typename PREC>
int32_t HoughTransformLaneDetector<PREC>::getLinePositionX(const Lines& lines, const Indices& lineIndices, Direction direction)
{
    const auto [m, b] = getLineParameters(lines, lineIndices);
    
    // 차선을 잃었을 경우
    if (std::abs(m) <= std::numeric_limits<PREC>::epsilon() && std::abs(b) <= std::numeric_limits<PREC>::epsilon())
    {
        if (direction == Direction::LEFT) {
            l_chk = -1;
            return 0.0f;
        }
        else if (direction == Direction::RIGHT) {
            r_chk = -1;
            return static_cast<PREC>(mImageWidth);
        }
    }

    PREC y = static_cast<PREC>(mROIHeight) * 0.5f;
    return std::round((y - b) / m);
}

template <typename PREC>
std::pair<Indices, Indices> HoughTransformLaneDetector<PREC>::divideLines(const cv::Mat& image, const Lines& lines)
{
    Indices leftLineIndices;
    Indices rightLineIndices;
    uint32_t linesSize = static_cast<uint32_t>(lines.size());
    leftLineIndices.reserve(linesSize);
    rightLineIndices.reserve(linesSize);
    PREC slope = 0.0f;
    PREC leftLineSumX = 0.0f;
    PREC rightLineSumX = 0.0f;

    for (uint32_t i = 0; i < linesSize; ++i)
    {
        const auto& line = lines[i];

        int32_t x1 = line[HoughIndex::x1];
        int32_t y1 = line[HoughIndex::y1];
        int32_t x2 = line[HoughIndex::x2];
        int32_t y2 = line[HoughIndex::y2];

        pt1 = cv::Point2i(x1, y1 + mROIStartHeight);
        pt2 = cv::Point2i(x2, y2 + mROIStartHeight);


        if (x2 - x1 == 0)
            slope = 0.0f;
        else
            slope = static_cast<PREC>(y2 - y1) / (x2 - x1);

        // std::cout<<"slope: "<<slope<<std::endl;

        if ((-mHoughLineSlopeRange <= slope && slope < 0.0f) && abs(slope)>0.1)
        {
            leftLineSumX += static_cast<PREC>(x1 + x2) * 0.5f;
            leftLineIndices.emplace_back(i);
            cv::line(image, pt1, pt2, kGreen, 2);
            // r_chk = -1;
            // std::cout<<"LEFT slope: "<< slope <<std::endl;
        }
        else if ((0.0f < slope && slope <= mHoughLineSlopeRange)  && abs(slope)>0.1)
        {
            rightLineSumX += static_cast<PREC>(x1 + x2) * 0.5f;
            rightLineIndices.emplace_back(i);
            cv::line(image, pt1, pt2, kRed, 2);
            // l_chk = -1;
            // std::cout<<"RIGHT slope: "<< slope <<std::endl;
        }
    }

    auto numLeftLines = static_cast<uint32_t>(leftLineIndices.size());
    auto numRightLines = static_cast<uint32_t>(rightLineIndices.size());

    if (numLeftLines != 0 && numRightLines != 0)
    {
        auto leftAverageX = static_cast<PREC>(leftLineSumX / numLeftLines);
        auto rightAverageX = static_cast<PREC>(rightLineSumX / numRightLines);
        if (leftAverageX > rightAverageX)
        {
            leftLineIndices.clear();
            rightLineIndices.clear();
            //std::cout << "------Invalid Path!------" << std::endl;
        }
    }

    return { leftLineIndices, rightLineIndices };
}

template <typename PREC>
cv::Mat HoughTransformLaneDetector<PREC>::imageProcessing(const cv::Mat& image, int lowThresholdValue) {
    cv::Mat blur;
    cv::GaussianBlur(image, blur, cv::Size(5, 5), 0);
    cv::Mat hsv;
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat vChannel = channels[2];
    cv::Mat lane;
    cv::threshold(vChannel, lane, lowThresholdValue, 255, cv::THRESH_BINARY_INV);
    return lane;
}

template <typename PREC>
// std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::getLanePosition(const cv::Mat& image)
std::tuple<int32_t, int32_t, int32_t, int, int> HoughTransformLaneDetector<PREC>::getLanePosition(const cv::Mat& image)
{
    // cv::Mat mask_image = cv::imread("/home/nvidia/xycar_ws/src/LaneKeepingSystem/src/LaneKeepingSystem/mask_image.png", 0);
    
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    
    // cv::Mat non_lidar_image;
    // cv::bitwise_and(grayImage, mask_image, grayImage);

    cv::Mat blur;
    cv::GaussianBlur(grayImage, blur, cv::Size(5, 5), 1.);

    cv::Mat canny_image;
    cv::Canny(grayImage, canny_image, mCannyEdgeLowThreshold, mCannyEdgeHighThreshold);
    // cv::imshow("hough", canny_image);

    cv::Mat ROI = canny_image(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));
    Lines allLines;
    cv::HoughLinesP(ROI, allLines, kHoughRho, kHoughTheta, mHoughThreshold, mHoughMinLineLength, mHoughMaxLineGap);

    if (mDebugging)
        image.copyTo(mDebugFrame);

    if (allLines.empty())
        return { 0, mImageWidth, 0, l_chk, r_chk};

    const auto [leftLineIndices, rightLineIndices] = divideLines(image, allLines);

    auto leftPositionX = getLinePositionX(allLines, leftLineIndices, Direction::LEFT);
    auto rightPositionX = getLinePositionX(allLines, rightLineIndices, Direction::RIGHT);

    if(l_chk == 1 && r_chk == -1){
        rightPositionX = leftPositionX + 395;
        r_chk = 1;
    }
    else if (l_chk == -1 && r_chk == 1){
        leftPositionX = rightPositionX - 395;
        l_chk = 1;
    }
    // else if (l_chk == -1 && r_chk == -1){
    //     // leftPositionX = ;
    //     // rightPositionX = ;
    //     l_chk = 1;
    //     r_chk = 1;
    // }

    if (mDebugging)
        drawLines(allLines, leftLineIndices, rightLineIndices);

    // 정지선
    cv::Rect roiRect(100, 350, 410, 10);
    cv::Mat stoplineRoi = image(roiRect);
    cv::Mat lane = imageProcessing(stoplineRoi, 70);

    int32_t countNonZero = cv::countNonZero(lane);

    std::cout << "countNonZero: " << countNonZero << std::endl;
    // cv::imshow("roi", stoplineRoi);
    // cv::imshow("roilane", lane);
    
    // if (countNonZero > 1000) {
        
    //     std::cout << "stopline" << std::endl;
    // }

    // return { leftPositionX, rightPositionX };
    return std::make_tuple( leftPositionX, rightPositionX , countNonZero, l_chk, r_chk);
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawLines(const Lines& lines, const Indices& leftLineIndices, const Indices& rightLineIndices)
{
    auto draw = [this](const Lines& lines, const Indices& indices) {
        for (const auto index : indices)
        {
            const auto& line = lines[index];
            auto r = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto g = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto b = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();

            cv::line(mDebugFrame, { line[static_cast<uint8_t>(HoughIndex::x1)], line[static_cast<uint8_t>(HoughIndex::y1)] + mROIStartHeight },
                     { line[static_cast<uint8_t>(HoughIndex::x2)], line[static_cast<uint8_t>(HoughIndex::y2)] + mROIStartHeight }, { b, g, r }, kDebugLineWidth);
        }
    };

    draw(lines, leftLineIndices);
    draw(lines, rightLineIndices);
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawRectangles(int32_t leftPositionX, int32_t rightPositionX, int32_t estimatedPositionX)
{
    // 왼쪽 차선
    cv::rectangle(mDebugFrame, cv::Point(leftPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(leftPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    // 오른쪽 차선
    cv::rectangle(mDebugFrame, cv::Point(rightPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(rightPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    // 차선 중앙
    cv::rectangle(mDebugFrame, cv::Point(estimatedPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(estimatedPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kRed, kDebugLineWidth);
    // 화면 중앙
    cv::rectangle(mDebugFrame, cv::Point(mImageWidth / 2 - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(mImageWidth / 2 + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kBlue, kDebugLineWidth);
}

template class HoughTransformLaneDetector<float>;
template class HoughTransformLaneDetector<double>;
} // namespace Xycar
