# Frame
参考[ORB SLAM2源码解读(三)：Frame类](https://zhuanlan.zhihu.com/p/84201110)
帧也就是一帧图像，该类包含的操作就是对帧的处理，其主要包含：
* 读写该帧下对应的相机位姿
* 处理帧与特征点之间的关系
* 恢复深度，视传感器类型选择方法
![image1](https://pic1.zhimg.com/80/v2-c3d355e322ec1bbf0dcdd343a0b3839c_720w.jpg)
关注单目和双目相机的函数；
## 构造函数Frame
构造函数共有五个：
* 默认构造函数
* 拷贝构造函数
* 单目构造函数
* 双目构造函数
* RGB-D构造函数
### stereo Frame
```cpp
// for stereo
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
     /**
      * instruction of parameters
      * @param imLeft image of left eye
      * @param imRight image of right eye
      * @param timeStamp namely time stamp
      * @param extractorLeft ORB-extractor class, left image's ORB
      * @param extractorRight right image's ORB-extractor
      * @param voc ORB vocabulary
      * @param K Intrinsic matrix of camera
      * @param distCoef distortion coefficient
      * @param bf baseline, feature of stereo camera
      * @param thDepth depth(threshold depth, distinguish scene for far or close)
      */
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);  // extract ORB feature from left eye
    thread threadRight(&Frame::ExtractORB,this,1,imRight);  // extract ORB feature from right eye
    // thread::join https://www.cplusplus.com/reference/thread/thread/join/
    // the function of the two thread::join sentences below:
    // thread Frame must wait until ORB extraction of left and right images finished
    // a good example for better understanding: https://www.cnblogs.com/adorkable/p/12722209.html
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();  // mvKeys means vector of key-points, original for visualization

    if(mvKeys.empty())
        // usually mbKeys will not be empty in stereo situation
        // if so, return
        return;

    // un-distort key points
    UndistortKeyPoints();

    // compute matches between left and right images
    // if successes, depth info will be sent to mvDepth
    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    // namely, assign features into grid
    AssignFeaturesToGrid();
}
```
里面注释中解释了很多，参考[ORB SLAM2源码解读(三)：Frame类](https://zhuanlan.zhihu.com/p/84201110)
主要内容有：
* 提取双目左右的ORB特征点
* 关键帧去畸变
* 计算双目匹配
* 地图点对应，应该是相应的地图点变量，建立了同其他数据变量的联系
* 最后将特征点分配到网格中，可以设置网格内特征点上限，使特征点分布更均匀？

关于`join`函数的官方解释：https://www.cplusplus.com/reference/thread/thread/join/
一个很好的例子：https://www.cnblogs.com/adorkable/p/12722209.html

### monocular Frame
基本上和stereo的类似
```cpp
// constructor function for monocular sensor
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // be advised that ORB extractor right is a null ptr cause this is mono
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        // return if there is no key points
        return;

    // un-distort
    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    // set MapPoint variable
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}
```
都没有深究相机重新标定的这个if情况，一般也用不上吧（猜测）
## void Frame::ExtractORB(int flag, const cv::Mat &im)
提取ORB特征，根据`flag`标志位判断左眼右眼
```cpp
{
    // flag = 0, using left eye
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}
```
## void Frame::ComputeBoW()
```cpp
{
    // create bag if no BoW2 words found
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}
```
## SetPose(cv::Mat Tcw) && UpdatePoseMatrices()
```cpp
void SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}
```
设置相机位姿，从`Tcw`矩阵clone得到
```cpp
void Frame::UpdatePoseMatrices()
// Computes rotation, translation and camera center matrices from the camera pose.
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}
```
注释的含义是从相机位姿中计算旋转、变换和相机光心矩阵

## bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
判断一个MapPoint是否在当前视野中
## vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
获取特定区域内的特征点
## cv::Mat Frame::UnprojectStereo(const int &i)
计算特征点在三维空间的坐标