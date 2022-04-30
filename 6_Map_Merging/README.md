# 地图融合

思考一下这一步该怎么做，自己还缺点什么；

首先，地图的融合，需要有说服力的验证：

* 首先应该用ORB跑一个自己录制视频的SLAM，把重建的地图展现出来，可以除了ORB自带的，有什么别的展现方式；
* 要完成上一步，需要知道ORB是怎样建图的，他在地图点都存储了哪些信息，其建图的可视化函数又是怎么写的；接口应该怎么找；
* 用自己的视频跑完一个SLAM主要就是学会接口设计，当然相机内参也得会得到；
* 最终要达到一个什么样的结果，就是手机分别录两个视频，最后完成建图的融合；
* 建图怎样算是融合，在一个窗口中，两个地图最终构建成了一个较好的融合地图；
* 感觉不只是最后建好图的融合，如果单纯这样，第二个相机一开始不知道与第一个相机的相对位置，可能需要逐帧对照？感觉需要仔细看看CCM的方法；

有了说服力的验证，就可以说有了一套方法：

* 在gazebo中进行双机实验，引入双机的基线

有点乱，感觉应该先了解了ORB的建图和MP数据结构，计算过程；再看CCM的拼合图像方法，写一个自己的方法，最后研究如何可视化；

## Collaborative Monocular SLAM with Multiple Micro Aerial Vehicles

https://ieeexplore.ieee.org/document/6696923

### System Overview

每架无人机通过视觉里程计跟踪自己的位置；指出任何可行的视觉里程计方案都可以使用；

![CsfM](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/6679723/6696319/6696923/6696923-fig-1-source-small.gif)

文章使用的是`CSfM`方案（作者自己的），需要在地面进行：

* 关键是图中的Place Recognizer，即相似场景识别，类似回环检测
* 每一个线程（无人机）都有自己的地图，当PR探测到重叠后，进行地图融合
* 然后两个线程同时读取并且更新地图
* A key-frame message only contains the extracted image features ( image coordinates
  and descriptors) along with a relative transformation to the previous key-frame. 用于传输的Key-Frame包含三个信息，特征点、描述子、和上一个关键帧的变换关系

总结下来就是，添加了一个**Place Recognizer**和**Overlap Detector**（位置互换）；

之后可以从`CSfM`的结构中得到一些启发：

![CSfM2](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/6679723/6696319/6696923/6696923-fig-2-source-small.gif)

可以看出，相比于ORB-SLAM的方案：

* 在某一帧确定为关键帧之后，首先进行`Overlap Detector`，如果探测到重叠，则进入`Place Recognize`模块
* 如果没有探测到重叠，则进入正常的优化，最后进入回环检测
* 探测到的若为重叠，则还需要一步几何验证，之后没有进行优化，直接进入`Map Merging`

==感觉上回环检测和重叠探测差不多，区别在哪？==

> The place recognizer accumulates the visual information (i.e., feature descriptors), from all key-frames in every map, and quickly detects whether a place has been visited before.

这里提了一下PR的工作原理，但用的是`ie`，使用所有关键帧特征点的描述子，快速探测是否场景重叠；

> If the overlap detector detects an overlap within the map of the same MAV, a loop-closure optimization is initiated. Conversely, if the overlap occurs with the map of another MAV, the affected frame handlers are temporarily suspended to allow merging of the maps into a single one

* 如果Overlap Detector探测到该场景的重叠是完完全全包含在本机地图中的，则会进入到回环检测
* 相反的，如果Overlap Detector探测到该场景是另一飞机出现过的场景，受影响的进程会被暂时推迟而开始地图融合，融合成一幅地图

==如果两个条件都满足？==

> After map merging, the frame handlers operate on the merged map. Specially-designed data structures and the use of C++ concurrency-control mechanisms allow multiple frame-handler threads to safely access and update the common map, which is also the key to real-time performance.

用多线程来实时拼合，保证实时性；

### Mapping Pipeline

### Map Overlap Detection and Processing

> A fundamental characteristic of the CSfM system is its ability to detect if a MAV reenters an environment that has already been visited, either by itself or by another MAV which results in a loop-closure optimization or a map merging respectively. Such overlaps are detected based on the keyframe appearance (i.e., feature descriptors) and subsequently geometrically verified.

`CSfM`系统的一个基本特征是能够探测到一架无人机是否进入了已经被访问过的场景，无论该场景是自己的还是另一架无人机访问过的；之后进入回环或地图融合；检测是基于关键帧特征点描述子和几何验证；

#### A. Appearance-based Overlap Detection

基于外观的重叠检测

> If a keyframe is accepted for inclusion in the map, a second overlap-detection thread is started, which calls the place-recognizer module (see Figure 2). The external place recognition module is the same for all frame handlers and relies on a bag-of-words [20] approach. The exact type of place recognizer in use depends on the employed local invariant point descriptor. We initally tested OpenSURF features [21], which allow the use of the OpenFABMAP place recognizer [22]. However, for increased speed, we decided to use BRISK features [23]. Since binary features have special clustering properties, a dedicated place-recognition module was implemented.

* 外部的重叠检测模块对于所有关键帧都适用并且基于`BoW`词袋模型，而准确的PR则是用不变的描述子
* `BRISK`描述子，和`ORB`方案不同，其是`BRIEF`的改进（[图像特征描述子之BRISK](https://senitco.github.io/2017/07/12/image-feature-brisk/)）；

#### B. Geometric Verification

几何验证

> Each time the place recognizer returns an overlap keyframe with similar appearance as the current keyframe, the overlap detector geometrically verifies the result by applying the Perspective-Three-Point (P3P) algorithm from our previous work [24]. The P3P algorithm derives the camera pose from at least three 3D-to-2D feature correspondences. These correspondences are established by identifying matching descriptors between map-points—which the overlap keyframe observes—and features in the current keyframe. To remove outliers, we integrated the P3P into a RANSAC [25] procedure. The output of RANSAC is then the rigid body transformation between the two keyframes.

* 当现关键帧进行重叠检测时，找到了一个关键帧与其具有相似的外观（闭环候选帧？），该结果就需要通过`P3P`验证
* `P3P`是三个匹配的3D特征点对去得到2D点对运动
* 这三个匹配的3D特征点对（具有深度）是从先前关键帧的地图点中根据验证描述子找到的+当前关键帧的特征点
* 去除不合格的，使用`P3P`融合`RANSAC`（随机抽样一致），最后的输出就是两个关键帧之间准确的变换过程
* `RANSAC`在`ORB-SLAM2`的回环检测`ComputeSim3`中也用到

==Sim3？P3P？==

#### C. Map Merging

> If the detected overlap occurred between two different maps, the similarity transformation {R, t, s} returned by the geometric verification step is used to merge the two maps into one. The factor s accounts for the different scale between the two maps and can be found by comparing the relative distances between any combination of 3D map-points which are common between the two maps. All frame handlers working on either of the two maps are temporarily suspended, and the entire candidate map for which an overlap was detected is subjected to the determined similarity transformation. To improve the measurements of points and avoid redundant information in the map, all map-points from each overlapping map region are reprojected into the keyframes from the other map and corresponding map-points are merged. A last important detail consists of applying the scale factor s to the scale difference factor (see Section III-D) of all frame handlers that were operating on the transformed map. This is necessary to ensure that the received relative position estimates from the VO are correctly scaled with respect to the map. The frame-handler threads are finally resumed, and now operate in parallel on the same map. At this stage, it is important to design the algorithm and data structures such that concurrent data access is possible (see Section V).

注意尺度；

> Note that the CSfM node creates references between two maps only when a loop closure is detected. However, in practice, the two maps may still contain overlaps in other regions if the place-recognition or the geometric-verification steps failed to detect them earlier. However, the CSfM system is still able to detect and incorporate them in a later stage in case a MAV retraverses the same environment.

有一些也是探测不到重叠的，但是重新回到该环境，有可能能探测到；

#### D. Loop Closure

## CCM-SLAM

https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21854



