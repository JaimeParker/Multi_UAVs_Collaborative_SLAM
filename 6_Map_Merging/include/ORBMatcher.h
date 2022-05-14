//
// Created by hazyparker on 22-5-14.
//

#ifndef INC_6_MAP_MERGING_ORBMATCHER_H
#define INC_6_MAP_MERGING_ORBMATCHER_H

#include <opencv2/opencv.hpp>

using namespace std;

class ORBMatcher {
public:
    static void findFeatures(cv::Mat &img1, cv::Mat &img2);
};


#endif //INC_6_MAP_MERGING_ORBMATCHER_H
