//
// Created by hazyparker on 22-5-14.
//

#ifndef INC_6_MAP_MERGING_ORBMATCHER_H
#define INC_6_MAP_MERGING_ORBMATCHER_H

#include <opencv2/opencv.hpp>

using namespace std;

class ORBMatcher {
public:
    /**
     * get ORB features
     * @param img1 image1 for detect
     * @param img2 image2 for detect
     */
    static void findFeatures(cv::Mat &img1, cv::Mat &img2);

    /**
     * compose Pose, R, t Matrix
     * @param keyPoints1 vector<cv::KeyPoint>
     * @param keyPoints2 vector<cv::KeyPoint>
     * @param matches vector<cv::DMatch>
     * @param R Rotation matrix R
     * @param t Transition matrix t
     */
    static void getPoseEstimation(vector<cv::KeyPoint> keyPoints1,
                                  vector<cv::KeyPoint> keyPoints2,
                                  vector<cv::DMatch> matches,
                                  cv::Mat &R, cv::Mat &t);

    /**
     * get landmark position
     * @param K cv::Mat, camera intrinsics matrix
     * @param keypoint_1 vector<cv::KeyPoint>
     * @param keypoint_2 vector<cv::KeyPoint>
     * @param matches vector<cv::DMatch>
     * @param R Rotation matrix R
     * @param t Transition matrix t
     * @param points vector<cv::Point3d>, hold MapPoint Position
     */
    static void getKeyPointPosition(const cv::Mat &K,
                                    const vector<cv::KeyPoint> &keypoint_1,
                                    const vector<cv::KeyPoint> &keypoint_2,
                                    const std::vector<cv::DMatch> &matches,
                                    const cv::Mat &R, const cv::Mat &t,
                                    vector<cv::Point3d> &points);

    /**
     * pixel to camera coordination
     * @param point cv::Point2d
     * @param K cv::Mat, camera intrinsics matrix
     * @return cv::Point2f, in camera's coordinate system
     */
    static cv::Point2f pixel2cam(const cv::Point2d &point, const cv::Mat &K);

    /**
     * plot MapPoints
     * @param MapPoints cv::Point3d, MapPoint position
     */
    static void showMapPoints(const vector<cv::Point3d> &MapPoints);

    static vector<cv::Point3d> shiftCoordinate(const cv::Mat &R, const cv::Mat &t,
                                const cv::Mat &K, const vector<cv::Point3d> &pos);
};


#endif //INC_6_MAP_MERGING_ORBMATCHER_H
