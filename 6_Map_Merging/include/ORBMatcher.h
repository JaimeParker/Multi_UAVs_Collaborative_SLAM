//
// Created by hazyparker on 22-5-14.
//

#ifndef INC_6_MAP_MERGING_ORBMATCHER_H
#define INC_6_MAP_MERGING_ORBMATCHER_H

#include <opencv2/opencv.hpp>

using namespace std;

class ORBMatcher {
public:
    // get ORB features
    static void findFeatures(cv::Mat &img1, cv::Mat &img2);

    // compose Pose, R, t Matrix
    static void getPoseEstimation(vector<cv::KeyPoint> keyPoints1,
                                  vector<cv::KeyPoint> keyPoints2,
                                  vector<cv::DMatch> matches,
                                  cv::Mat &R, cv::Mat &t);
};


#endif //INC_6_MAP_MERGING_ORBMATCHER_H
