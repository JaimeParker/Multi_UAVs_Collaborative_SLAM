//
// Created by hazyparker on 22-5-12.
// This file is designed for Data Analysis and Flow in common SLAM threads
// However, we are using pictures here
//

// Project and cpp
#include "Data.h"
#include "ORBMatcher.h"
#include <iostream>

// ROS
#include <ros/ros.h>

// CV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

Data::Data(string &path1, string &path2, string &path3, string &path4) {
    Client0Img1 = imread(path1);
    Client0Img2 = imread(path2);
    Client1Img1 = imread(path3);
    Client1Img2 = imread(path4);

    K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    ORBMatcher::findFeatures(Client0Img1, Client0Img2);
}

void Data::getPose() {
    // Descriptors
    Mat descriptor0_1;
    Mat descriptor0_2;
    Mat descriptor1_1;
    Mat descriptor1_2;

    // normal matches
    vector<DMatch> matches0;
    vector<DMatch> matches1;

    // create ORB detectors
    Ptr<FeatureDetector> detector0 = ORB::create();
    Ptr<DescriptorExtractor> descriptor0 = ORB::create();
    Ptr<DescriptorMatcher> matcher0 = DescriptorMatcher::create("BruteForce-Hamming");
    Ptr<FeatureDetector> detector1 = ORB::create();
    Ptr<DescriptorExtractor> descriptor1 = ORB::create();
    Ptr<DescriptorMatcher> matcher1 = DescriptorMatcher::create("BruteForce-Hamming");

    Ptr<DescriptorMatcher> matcher2 = DescriptorMatcher::create("BruteForce-Hamming");

    // detect FAST points
    detector0->detect(Client0Img1, KeyPoint0_1);
    detector0->detect(Client0Img2, KeyPoint0_2);
    detector1->detect(Client1Img1, KeyPoint1_1);
    detector1->detect(Client1Img2, KeyPoint1_2);

    // compute BRIEF descriptor
    descriptor0->compute(Client0Img1, KeyPoint0_1, descriptor0_1);
    descriptor0->compute(Client0Img2, KeyPoint0_2, descriptor0_2);
    descriptor1->compute(Client1Img1, KeyPoint1_1, descriptor1_1);
    descriptor1->compute(Client1Img2, KeyPoint1_2, descriptor1_2);

    // match descriptors
    matcher0->match(descriptor0_1, descriptor0_2, matches0);
    matcher1->match(descriptor1_1, descriptor1_2, matches1);

    // get good matches
    auto min_max = minmax_element(matches0.begin(), matches0.end(),
                                  [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    for (int i = 0; i < descriptor0_1.rows; i++) {
        if (matches0[i].distance <= max(2 * min_dist, 30.0)) {
            Matches0.push_back(matches0[i]);
        }
    }
    auto min_max1 = minmax_element(matches1.begin(), matches1.end(),
                                  [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist1 = min_max1.first->distance;
    double max_dist1 = min_max1.second->distance;
    for (int i = 0; i < descriptor1_1.rows; i++) {
        if (matches1[i].distance <= max(2 * min_dist1, 30.0)) {
            Matches1.push_back(matches1[i]);
        }
    }

    // get Pose
    Mat R0, t0;
    Mat R1, t1;
    ORBMatcher::getPoseEstimation(KeyPoint0_1, KeyPoint0_2, Matches0, R0, t0);
    ORBMatcher::getPoseEstimation(KeyPoint1_1, KeyPoint1_2, Matches1, R1, t1);

    // get KP position
    vector<Point3d> map_points0, map_points1;
    ORBMatcher::getKeyPointPosition(K, KeyPoint0_1, KeyPoint0_2,
                                    matches0, R0, t0, map_points0);
    ORBMatcher::getKeyPointPosition(K, KeyPoint1_1, KeyPoint1_2,
                                    matches1, R1, t1, map_points1);

    // show test
    cout << "MP(0) size:" << map_points0.size() << endl;
    cout << "MP(1) size:" << map_points1.size() << endl;
//    ORBMatcher::showMapPoints(map_points0);
//    ORBMatcher::showMapPoints(map_points1);

    // detect transform from 1 to 3
    vector<DMatch> matches2, Matches2;
    matcher2->match(descriptor0_1, descriptor1_1, matches2);
    auto min_max2 = minmax_element(matches2.begin(), matches2.end(),
                                   [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist2 = min_max2.first->distance;
    double max_dist2 = min_max2.second->distance;
    for (int i = 0; i < descriptor1_1.rows; i++) {
        if (matches1[i].distance <= max(2 * min_dist2, 30.0)) {
            Matches2.push_back(matches1[i]);
        }
    }
    Mat R, t;
    ORBMatcher::getPoseEstimation(KeyPoint0_1, KeyPoint1_1, Matches2, R, t);

    // get KeyPoint position of Client0 in Client1
    vector<Point3d> map_point0_1;
    map_point0_1 = ORBMatcher::shiftCoordinate(R, t, K, map_points0);
    map_point0_1.insert(map_point0_1.end(), map_points1.begin(), map_points1.end());
    // FIXME: all map_point1 were added to map_point0_1, no comparison of these MPs

    // show combination map
    ORBMatcher::showMapPoints(map_point0_1);

}


inline cv::Scalar get_color(float depth) {
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) depth = up_th;
    if (depth < low_th) depth = low_th;
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}
