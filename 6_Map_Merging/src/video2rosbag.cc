//
// Created by hazyparker on 2022/4/15.
// https://blog.csdn.net/weixin_40830684/article/details/88650163
//

#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv){
    // ros init
    ros::init(argc, argv, "rosbag_recode_node");
    // create ros handle
    ros::NodeHandle nh;

    // define and open a ros bag
    rosbag::Bag bag;
    bag.open("file.bag", rosbag::bagmode::Write);  // set mode as write

    // provide input video
    cv::VideoCapture cap("path/file.mp4");

    // get frame of the video
    long totalFrameNum = cap.get(CAP_PROP_FRAME_COUNT);
    cout << "total frame:" << totalFrameNum << endl;

    long FrameCount = 0;
    while (ros::ok()) {
        // send each frame msg to frame(Mat)
        cv::Mat frame;
        cap >> frame;

        // if any frame is empty
        if (frame.empty()) {
            std::cout << "error, no image" << std::endl;
            return -1;
        }

        // set wait key
        cv::imshow("image", frame);
        cv::waitKey(25);  // 此处的waitKey很重要，影响了录制长度

        std_msgs::Header header;
        header.frame_id = "image_frame";
        header.stamp = ros::Time::now();

        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        // sensor_msgs::CompressedImagePtr compressed_image_msg = cv_bridge::CvImage(header, "bgr8", frame).toCompressedImageMsg();

        if(FrameCount % 2 == 0)
            bag.write("/camera/image_raw", ros::Time::now(), image_msg);
        // bag.write("image_raw/compressed", ros::Time::now(), compressed_image_msg);

        //ros::Time time;
        //time.fromNSec(timestamp); // 从图像数据集生成bag时，timestamp一般为图像名
        //bag.write("image_topic", time, msg);
        ++FrameCount;
    }

    // close ros bag
    bag.close();
    cap.release();

    return 0;
}

