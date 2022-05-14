//
// Created by hazyparker on 22-5-12.
// This file is designed for Data Analysis and Flow in common SLAM threads
// However, we are using pictures here
//

// Project and Cpp
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

    //ORBMatcher::findFeatures(Client0Img1, Client0Img2);
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
    auto min_max1 = minmax_element(matches0.begin(), matches0.end(),
                                  [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist1 = min_max1.first->distance;
    double max_dist1 = min_max1.second->distance;
    for (int i = 0; i < descriptor1_1.rows; i++) {
        if (matches1[i].distance <= max(2 * min_dist1, 30.0)) {
            Matches1.push_back(matches1[i]);
        }
    }

    cout << Matches0.size();

    // get Pose
    Mat R0, t0;
    Mat R1, t1;
    ORBMatcher::getPoseEstimation(KeyPoint0_1, KeyPoint0_2, Matches0, R0, t0);
    ORBMatcher::getPoseEstimation(KeyPoint1_1, KeyPoint1_2, Matches1, R1, t1);

}
