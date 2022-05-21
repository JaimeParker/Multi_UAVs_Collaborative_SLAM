//
// Created by hazyparker on 22-5-14.
//

#ifndef INC_6_MAP_MERGING_DATA_H
#define INC_6_MAP_MERGING_DATA_H

// CV
#include <opencv2/opencv.hpp>


using namespace std;

class Data {

public:
    // constructor function, loading images
    Data(string &path1, string &path2, string &path3, string &path4);

    // camera config
    cv::Mat K;

    // pictures needed in algorithm realization
    cv::Mat Client0Img1;
    cv::Mat Client0Img2;
    cv::Mat Client1Img1;
    cv::Mat Client1Img2;

    // KeyPoints
    vector<cv::KeyPoint> KeyPoint0_1;
    vector<cv::KeyPoint> KeyPoint0_2;
    vector<cv::KeyPoint> KeyPoint1_1;
    vector<cv::KeyPoint> KeyPoint1_2;


    // Depth Matches
    vector<cv::DMatch> Matches0;
    vector<cv::DMatch> Matches1;



    // pose estimation 2D-2D
    void getPose();

};


#endif //INC_6_MAP_MERGING_DATA_H
