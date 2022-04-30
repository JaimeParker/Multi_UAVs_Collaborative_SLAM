# MapPoint类
参考自 [ORB SLAM2源码解读(二)：MapPoint类](https://zhuanlan.zhihu.com/p/84024378)
MapPoint是地图中的特征点，其自身的参数是三维坐标和描述子；
![image1](https://pic1.zhimg.com/80/v2-b01078589cf9aa13a03d4d39d61bb928_720w.jpg)
## MapPoint 构造函数
构造函数分两个，用于不同类型帧的处理：
首先是带关键帧参数的构造函数
### MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap)
```cpp
// constructor function of class MapPoint, using KeyFrame as a parameter
MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}
```
* `Pos`矩阵传给了`mWorldPos`矩阵成员，其在头文件中的定位是Position in absolute coordinates，位置的绝对坐标
* 构造了3*1矩阵`mNormalVector`
* 加互斥锁，防止id冲突
* 和关键帧相关的`MP`构造函数主要是突出地图点和关键帧之间的观测关系，即参考关键帧是哪一帧，该地图点被哪些关键帧观测到；一个地图点可能会被多个关键帧观测到，多个关键帧之间共同观测到地图点叫**共视关系**，`MapPoint`类用来维护共视关系；在进行局部BA优化时，只有具有共视关系的关键帧才会进入优化，其他关键帧的位姿不参与优化；
  另一个构造函数用于普通帧的处理；
### MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF)
```cpp
// another constructor function, using Frame and idxF(unknown)
MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);  // mWorldPos is a temporary variable, saving position
    cv::Mat Ow = pFrame->GetCameraCenter();  // Ow is position of camera center
    mNormalVector = mWorldPos - Ow;  // mNormalVector means relative position of camera center
    mNormalVector = mNormalVector/cv::norm(mNormalVector);  // normalization of this mat

    cv::Mat PC = Pos - Ow;  // PC might mean position current, Pos-Ow, relative position
    const float dist = cv::norm(PC);  // get distance between camera center and the point
    const int level = pFrame->mvKeysUn[idxF].octave;  // 
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);  // copy descriptors

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}
```
![image2](https://docs.opencv.org/2.4/_images/math/99ed6771acf6fa12588487d65a9526eb97d48f63.png)

来自[C++ OpenCV cv::norm()](https://cppsecrets.com/users/168511510411111711412197115105110104975764103109971051084699111109/C00-OpenCV-cvnorm.php)
* 作为普通帧，其位置减去了相机的位置，获得了相对相机的位置`PC`和`mNormalVector`，但正如其名，`mNormalVector`随后做了归一化处理，调用了范数函数进行；
* 随后对该帧进行了一些操作，获得了描述子，并且复制给了`mDescriptor`成员变量，其在头文件的定义是 Best descriptor to fast matching
* 关键帧的函数里没有进行相减的运算，可能关键帧构造函数是先从普通帧构造函数来的，所以没有相减和归一化；==也不太对；==
## AddObservation(KeyFrame* pKF, size_t idx)
博主的解释是该函数用来增加地图点的观测关系；
```cpp
    unique_lock<mutex> lock(mMutexFeatures);
    // if observation is established, then return
    if(mObservations.count(pKF))
        return;
    // else, add this observation
    mObservations[pKF]=idx;
```
* `mObservations`的定义为`std::map<KeyFrame*,size_t> mObservations;`，Keyframes observing the point and associated index in keyframe，观察到该点的关键帧和关键帧中的系数
* 如果已经存在观测关系，就返回
* 如果不存在观测关系，则添加该观测关系
```cpp
    // 2 for stereo camera, mvuRight exists
    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    // 1 for monocular camera, mvuRight doesn't exist
    else
        nObs++;
```
* `nObs`为观测次数
* 分单目双目添加观测次数
## void MapPoint::EraseObservation(KeyFrame* pKF)
函数意为删除观测关系；参数为关键帧指针；
```cpp
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        // if observation is established in this key-frame, namely the KeyFrame can see this point
        if(mObservations.count(pKF))
        {
            // same with the AddObservation(), different situation for stereo and monocular
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            // delete the observation relationship between KeyFrame and the MapPoint
            mObservations.erase(pKF);

            // if KeyFrame is reference KeyFrame, re-direct the mpRefKF
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;  // this is a bad observation
        }
    }

    // if bBad is true, this observation will be discarded, set a bad flag
    if(bBad)
        SetBadFlag();
}
```
## void MapPoint::SetBadFlag()
删除地图点，清除关键帧和地图中所有和该地图点对应的关联关系；
```cpp
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        // clear all observations of that MapPoint
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        // erase the match with MapPoint and KeyFrame
        pKF->EraseMapPointMatch(mit->second);
    }

    // delete MapPoint from Map
    mpMap->EraseMapPoint(this);
}
```









