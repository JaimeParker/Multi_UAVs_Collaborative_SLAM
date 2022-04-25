#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

int main() {
    Mat srcImage = imread("../image/1.jpg");

    // set features number(max)
    int numFeatures = 100;
    Ptr<SIFT> detector = SIFT::create(numFeatures);

    vector<KeyPoint> key_points;
    detector->detect(srcImage, key_points, Mat());
    printf("Total key-points: %zu\n", key_points.size());

    Mat keypoint_img;
    drawKeypoints(srcImage, key_points, keypoint_img, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("keypoint_img", keypoint_img);

    waitKey(0);
    return 0;
}
