# ORB SLAM2

* [官方的README文档](Instruction.md)
* [论文](paper)

## 1. 代码逻辑

### 1.1 整体框架

> 参考[ORB-SLAM2详解（二）代码逻辑](https://blog.csdn.net/u010128736/article/details/53169832)

代码逻辑即代码是按照怎样的轨迹被运行的，参考一个双目的例子[stereo_euroc.cc](Examples/Stereo/stereo_euroc.cc)；

分为`LoadImages`函数和`main`主函数两个函数；封装性很好，构建了SLAM类来完成整个SLAM的流程；

* 前者负责加载数据集的图片
* 后者负责构建各矩阵，整个SLAM系统的启动和结束

**创建SLAM系统，初始化线程，准备处理关键帧**

```c++
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
```

**传输图像到SLAM系统**

```c++
        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeftRect,imRightRect,tframe);
```

循环传输所有图片

**最后，关闭SLAM系统**

```c++
    // Stop all threads
    SLAM.Shutdown();
```

**保存相机运动轨迹**

```c++
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
```

### 1.2 System

**System析构函数**

* 判断相机类型
* 判断配置文件合法性
* 加载词典
* 创建关键帧数据库
* 创建地图
* 创建关键帧和地图的drawer对象
* 初始化跟踪进程
* 初始化并开始地图进程
* 初始化并开始回环检测进程
* 初始化并开始可视化进程
* 在各进程间设置点

```c++
 //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
```

基本上就是在创建对象，控制参数；

注意，system的cpp有：

```c++
#include <thread>
```

加上`ORB`本来就多线程，所以应该是有多线程出现的；

**TrackStereo函数**

检测是否是双目相机，模式的检测和重置；

```c++
    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
```

`mpTracker`是track对象，其调用`GrabImageStereo`函数：(`Tracking.cc`)

* 将左右相机的图像转为灰度图
* 把新值通过构造函数赋给`mCurrentFrame`
* 调用`Track()`函数
* 返回`mCurrentFrame.mTcw.clone()`

于是`Tcw`拿到了矩阵形式的相机位姿；

进而完成了`mTrackState`，`mTrackedMapPoints`，`mTrackedKeyPointsUn`三个`System`类成员的赋值；

### 1.3 数据流

在简单看了ORB的论文之后可能有更清楚的理解；现需要建立框图到代码和具体对象的联系；

根据论文流程图，Tracking进程主要有四部分内容：

```flow
st=>start: Frame Input
op1=>operation: Pre-process Input
op2=>operation: Pose Prediction or Relocalization
op3=>operation: Track Local Map
op4=>operation: New KF Decision
st->op1(right)->op2(right)->op3(right)->op4
```

```c++
    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
```

` mpKeyFrameDatabase`

```c++
    //Create the Map
    mpMap = new Map();
```

`mpMap`

```c++
    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
```

`mpFrameDrawer`

`mpMapDrawer`

```c++
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);
```

`mpTracker`

```c++
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);
```

`mpLocalMapper`

`mpLocalMapping`

```c++
    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
```

`mpLoopCloser`

`mptLoopClosing`

## 2. Tracking

* Pre-process Input
* Pose Prediction(Motion Model) or Relocalization
* Track Local Map
* New KeyFrame Decision

在`System.cc`中，Tracking前还有几条语句，按照顺序执行应在Tracking之前，一并放在这里处理；

<img src="https://img-blog.csdnimg.cn/20190515203756572.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L21veXUxMjM0NTY3ODk=,size_16,color_FFFFFF,t_70" alt="image_from_csdn" style="zoom: 67%;" />

CSDN博主的一篇[文章](https://blog.csdn.net/moyu123456789/article/details/90241784)里的框图和最终IEEE的框图有一些不一样的地方，在Tracking进程的第一步，两者分别为：

* Pre-process Input(IEEE)
* Extract ORB, Map Initialization

但是根据实际代码，在Tracking之前的部分有：

```c++
    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
```

* 其中，`mpVocabulary`是之前定义的类成员对象，与ORB词典有关；新建了一个`KeyFrameDatabase`对象；
* 新建地图，初始化
* 关键帧的画笔和地图的画笔的初始化

```c++
    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;
```

### 2.1 Pre-process Input

首先分析关键帧的database文件：

#### `KeyFrameDatabase`

```c++
class KeyFrameDatabase{
public:

    KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};
```

基本是按照SLAM顺序编写的函数，现阶段只需要：

```c++
    KeyFrameDatabase(const ORBVocabulary &voc);
   void add(KeyFrame* pKF);
```

构造函数和添加关键帧的函数；

```c++
KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc){
    mvInvertedFile.resize(voc.size());
}
```

`mvInvertedFile`来自头文件：

```c++
  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;
```

`stl_list`类型，`KeyFrame`也是自己定义的类型；

```c++
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}
```

用了一部分`DBoW2`的东西，暂时看不懂；

但是可以看出是一个简单的循环，`mBowVec`中的start到end循环了一遍，每一次循环都把`pKF`也就是关键帧加到关键帧列表`mvInvertedFile`中；

`vit->first`存疑；

详细的需要进入到`Frame.cc`中去研究其数据结构；

特别地，分析一下`Frame.cc`，学习其关键帧的数据结构和相关函数：

#### `Frame`

#### `Map`

按照原来的思路，继续学习`Tracking`的第一部分；

接下来是：

```c++
    //Create the Map
    mpMap = new Map();
```

在`Map.h`中包含了以下头文件：

```c++
#include "MapPoint.h"
#include "KeyFrame.h"
```

#### `Tracking`

在头文件的声明中：

```cpp
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
```

基本上是在用指针引用每一个类新创建的对象，在cpp中：

```cpp
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
```

构造函数，冒号后对一些具体的对象，分为类成员变量和const类型的变量；

首先是**从参数文件中加载相机的参数**：[相机内参的标定](https://zhuanlan.zhihu.com/p/448120739)

* 内参矩阵赋给类成员变量`mK`，Calibration matrix
* 设置畸变系数矩阵`mDistCoef`
* 设置`mbf`，基线距离，用像素单位，baseline times fx(approx.)，[ORB源码中相机的参数设置](https://blog.csdn.net/catpico/article/details/120688795)
* 设置帧率，fps
* 设置插入关键帧和检查重定位的关键帧最大和最小数量，最大就取帧率

第二步是**加载ORB的参数**：

```cpp
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
```

---

构造函数之后，设置了地图、回环和viewer：

```cpp
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}
```

---

之后是三个对于不同传感器的图像处理函数，对于双目，`GrabImageStereo`的作用如下：

`Tcw`拿到了矩阵形式的相机位姿；

进而完成了`mTrackState`，`mTrackedMapPoints`，`mTrackedKeyPointsUn`三个`System`类成员的赋值；

---

接下来进入一个200+的函数，`Track()`：

对于该函数的描述是，跟踪进程的主函数，不受传感器类型影响；

* 检查`mState`成员变量的值
* 如果image未好，设置为未初始化
* 互斥锁，给Map加锁，不可改变

```cpp
    // track the state currently
    mLastProcessedState=mState;
```

用于保存状态；

关于`Tracking states`，有：

```cpp
// Tracking states
enum eTrackingState{
    SYSTEM_NOT_READY=-1,
    NO_IMAGES_YET=0,
    NOT_INITIALIZED=1,
    OK=2,
    LOST=3
};
```

### 2.2 Pose Prediction(Motion Model) or Relocalization

#### `Tracking`

在该函数中，进行完第一部分的一些状态检查和初始化之后，接下来进入Tracking进程的第二部分，可以从变量的命名中看出来；

```cpp
 // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
```

通过运动模型或者重定位初始化估计相机位姿，如果跟踪丢失时；

---

逻辑变量`mbOnlyTracking`用于标记，如果为True，则意味着关闭建图进程，只用于定位；在这里只有false才能进入：

```cpp
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
```

可能会影响一些上一帧的地图点，该函数暂时不影响整体的Tracking流程

```cpp
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();
```

这里有一个判断：

```cpp
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
```

* 其中，头文件中给`mVelocity`定义为motion model，字面意思就是速度
* 另一个判断条件`mCurentFrame.mnId<mnLastRelocFrameId+2`，`mnId`意思即为当前的Frame ID
* 如果速度矩阵为空，或是ID小于上次重定位的帧ID+2，则进入该条件判断
* 剩下是几个bool类型的函数给`bOK`赋值，决定要不要用motion model，最终还是要转到last frame上

---

逻辑变量`mbOnlyTracking`的另一种选项，接下来是和上面的if平行的判断，else语句，此时进入定位进程：

```cpp
else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }
```

正常顺利情况下，`mState`不会Lost，如果丢失，则进入重定位：

```cpp
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                // if lost, use Re-localization to regain position
                bOK = Relocalization();
            }
```

如果顺利正常，`mState`正常，则根据逻辑变量`mbVO`的值来判断；

关于`mbVO`，定义于Tracking类的proteced类型变量：

```cpp
    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;
```

* 当系统仅有定位进程时，`mbVO`这个逻辑标志将被设置为true，意味着地图上没有匹配的点；
* 当有足够的匹配点时，跟踪进程将会继续；
* 在那种（？）情况下正在视觉里程计
* 系统会试着重定位并且使地图修复漂移的定位？

对于`mbVO`为false的情况：

```cpp
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
```

* 作者的注释中写道，上一帧已经跟踪到了足够的地图点；
* 之后同样进入了速度的选择，是选择motion model还是跟踪参考关键帧

对于`mbVO`为true的情况：



### 2.3 Track Local Map



### 2.4 New KeyFrame Decision

## 3. Mapping

## 4. Loop Closing



