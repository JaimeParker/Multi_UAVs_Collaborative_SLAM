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
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

Data::Data(string &path1, string &path2, string &path3, string &path4) {
    Client0Img1 = imread(path1);
    Client0Img2 = imread(path2);
    Client1Img1 = imread(path3);
    Client1Img2 = imread(path4);

    ORBMatcher::findFeatures(Client0Img1, Client0Img2);
}

void Data::getPose(Mat &img1, Mat &img2) {

}
