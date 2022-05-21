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

    // get landmark position
    static void getKeyPointPosition(const cv::Mat &K,
                                    const vector<cv::KeyPoint> &keypoint_1,
                                    const vector<cv::KeyPoint> &keypoint_2,
                                    const std::vector<cv::DMatch> &matches,
                                    const cv::Mat &R, const cv::Mat &t,
                                    vector<cv::Point3d> &points);

    // pixel to camera coordination
    static cv::Point2f pixel2cam(const cv::Point2d &point, const cv::Mat &K);

    // plot MapPoints
    static void showMapPoints(const vector<cv::Point3d> &MapPoints);
};


#endif //INC_6_MAP_MERGING_ORBMATCHER_H
