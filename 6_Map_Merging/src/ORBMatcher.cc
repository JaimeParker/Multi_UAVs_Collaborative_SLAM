//
// Created by hazyparker on 22-5-14.
//

#include "ORBMatcher.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>



using namespace std;
using namespace cv;


void ORBMatcher::findFeatures(Mat &img1, Mat &img2) {
    // Initialize
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // detect FAST KeyPoint
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img1, keypoints_1);
    detector->detect(img2, keypoints_2);

    // compute BRIEF descriptor
    descriptor->compute(img1, keypoints_1, descriptors_1);
    descriptor->compute(img2, keypoints_2, descriptors_2);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;


    // create window
    namedWindow("ORB features", WINDOW_NORMAL);
    namedWindow("all matches", WINDOW_NORMAL);
    namedWindow("good matches", WINDOW_NORMAL);

    Mat outimg1;
    drawKeypoints(img1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", outimg1);

    // Match KeyPoints, using Hamming distance
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

    // Choose KeyPoints matched
    auto min_max = minmax_element(matches.begin(), matches.end(),
                                  [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    // set standard for good matches
    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    // Draw matches
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img1, keypoints_1, img2, keypoints_2, matches, img_match);
    drawMatches(img1, keypoints_1, img2, keypoints_2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);

    cout << "Matches:" << matches.size() << endl;
}

void ORBMatcher::getPoseEstimation(vector<cv::KeyPoint> keyPoints1, vector<cv::KeyPoint> keyPoints2,
                                   vector<cv::DMatch> matches, Mat &R, Mat &t) {
    // 相机内参,TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keyPoints1[matches[i].queryIdx].pt);
        points2.push_back(keyPoints2[matches[i].trainIdx].pt);
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);
    //cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //-- 计算本质矩阵
    Point2d principal_point(325.1, 249.7);  //相机光心, TUM dataset标定值
    double focal_length = 521;      //相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    //cout << "essential_matrix is " << endl << essential_matrix << endl;

    //-- 计算单应矩阵
    //-- 但是本例中场景不是平面，单应矩阵意义不大
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    //cout << "homography_matrix is " << endl << homography_matrix << endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    // 此函数仅在Opencv3中提供
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}

void ORBMatcher::getKeyPointPosition(const Mat &K, const vector<cv::KeyPoint> &keypoint_1,
                                     const vector<cv::KeyPoint> &keypoint_2,
                                     const vector<cv::DMatch> &matches,
                                     const Mat &R, const Mat &t, vector<cv::Point3d> &points) {

    Mat T1 = (Mat_<float>(3, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0);

    Mat T2 = (Mat_<float>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    // define vector to hold MP position
    vector<Point2f> Pc1, Pc2;  // position of camera 1 and 2

    // pixel pos to camera pos
    for (DMatch match:matches){
        Pc1.push_back(pixel2cam(keypoint_1[match.queryIdx].pt, K));
        Pc2.push_back(pixel2cam(keypoint_2[match.trainIdx].pt, K));
    }

    //define Mat to hold MP
    Mat MP_pos4d;

    // triangulate
    triangulatePoints(T1, T2, Pc1, Pc2, MP_pos4d);

    // into 3 axis
    for (int i = 0; i < MP_pos4d.cols; i++){
        Mat x = MP_pos4d.col(i);

        x = x / x.at<float>(3, 0);
        Point3d p(
                x.at<float>(0, 0),
                x.at<float>(1, 0),
                x.at<float>(2, 0)
                );

        // save p to point
        points.push_back(p);
    }

}

cv::Point2f ORBMatcher::pixel2cam(const Point2d &point, const Mat &K) {
    return Point2f (
            (point.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (point.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
}

void ORBMatcher::showMapPoints(const vector<cv::Point3d> &MapPoints) {
    // create GUI window
    pangolin::CreateWindowAndBind("MapPoint", 800, 600);
    // start depth test
    glEnable(GL_DEPTH_TEST);

    // create a camera viewer
    pangolin::OpenGlRenderState cam_view(
            pangolin::ProjectionMatrix(800, 600, 420, 420, 320, 320, 0.2, 100),
            pangolin::ModelViewLookAt(0, 0, 0, 1, 1, 1, pangolin::AxisY)
            );

    // create inter viewer
    pangolin::Handler3D handler(cam_view);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -800.0f/600.0f)
            .SetHandler(&handler);

    while(!pangolin::ShouldQuit()){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
        d_cam.Activate(cam_view);

        glPointSize(2.0);

        glBegin(GL_POINTS);
        glColor3f(1, 0, 0);

        for (const auto & MapPoint : MapPoints){
            glVertex3d(MapPoint.x, MapPoint.y, MapPoint.z);
        }

        glEnd();
        pangolin::FinishFrame();
    }

}

vector<cv::Point3d> ORBMatcher::shiftCoordinate(const Mat &R, const Mat &t, const Mat &K,
                                                const vector<cv::Point3d> &pos) {
    // Pos_new = (RP + t)
    vector<Point3d> shiftedPos;
    Point3d p;

    Eigen::Matrix<double, 3, 3> Mat_R;
    Eigen::Matrix<double, 3, 1> Mat_t;
    Eigen::Matrix<double, 3, 1> Mat_P;
    Eigen::Matrix<double, 3, 3> Mat_K;
    Eigen::Matrix<double, 3, 1> Mat_Pos;

    cv2eigen(R, Mat_R);
    cv2eigen(t, Mat_t);

    for (const auto & po : pos){
        Mat_Pos << po.x, po.y, po.z;
        Mat_P = Mat_R * Mat_Pos + Mat_t;
        p.x = Mat_P(0);
        p.y = Mat_P(1);
        p.z = Mat_P(2);
        shiftedPos.push_back(p);
    }

    return shiftedPos;
}



