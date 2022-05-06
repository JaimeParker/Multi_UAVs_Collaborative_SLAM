# Design Step
怎样设计； brain storm；

首先，两架无人机，跑两个ORB-SLAM2，如何做到启动两个ORB-SLAM2的（CCM没用ORB的窗口，可以看到其函数中也没有类似的Drawer）；

CCM能不能改成适用于双目的？



## 两个ORB-SLAM2端口
`CCM`是如何做到两个`ORB-SLAM2`的；

如果以同样的命令启动两个完全相同的ORB窗口，会报错由于相同ROS节点，这一点在其ROS启动文件中有涉及：

```cpp
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }  
```

所以多端口很简单，只要更改ROS节点名即可；或者加上参数，在传统的ORB启动方式：

```cpp
// Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings
```

后再加上一个`port`，ROS节点名也作相应改变；

### ClientNode

在`CCM-SLAM`的`ClientNode.cpp`中有ROS节点的初始化：

```cpp
int main(int argc, char **argv) {

    ros::init(argc, argv, "CSLAM client node");

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun cslam clientnode path_to_vocabulary path_to_cam_params" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle Nh;  // topic name will be: node name(only), like "/image_raw"
    ros::NodeHandle NhPrivate("~");  // topic node will be: node name + topic name, like "iris_0/image_raw"
    // reference: <https://blog.csdn.net/weixin_44401286/article/details/112204903>

    boost::shared_ptr<cslam::ClientSystem> pCSys{new cslam::ClientSystem(Nh,NhPrivate,argv[1],argv[2])};

    ROS_INFO("Started CSLAM client node...");

    ros::Rate r(params::timings::client::miRosRate);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
```

其中出现了一个：

```cpp
    boost::shared_ptr<cslam::ClientSystem> pCSys{new cslam::ClientSystem(Nh,NhPrivate,argv[1],argv[2])};
```

注意，在这里利用智能指针调用了其`ClientSystem`中的函数，在`ClientSystem`中通过读取文件的方式，生成了不同的ROS节点；

### Client System

从ROS参数服务器读取Client ID：

```cpp
    int ClientId;
    // get ClientID from launch file, actually ROS parameter server
    mNhPrivate.param("ClientId",ClientId,-1);
    mClientId = static_cast<size_t>(ClientId);  // assign ClientID to member of class ClientSystem, mClientID
```

标准的SLAM流程：

```cpp
	// load vocabulary
    this->LoadVocabulary(strVocFile);

    // Create KeyFrame Database
    mpKFDB.reset(new KeyFrameDatabase(mpVoc));

    // Create the Map
    mpMap.reset(new Map(mNh,mNhPrivate,mClientId,eSystemState::CLIENT));
    usleep(10000); //wait to avoid race conditions
```

端口（客户端、服务端）的建立：

```cpp
    // Initialize Agent
    mpAgent.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap,
                                    mClientId,mpUID,eSystemState::CLIENT,strCamFile,nullptr));
    usleep(10000); //wait to avoid race conditions
    mpAgent->InitializeThreads();
    usleep(10000); //wait to avoid race conditions
```

在这里引入了`ClientHandler`；

### Client Handler

在上面的代码`ClientSystem`中，对`ClientHandler`做了两件事：

* 对`mpAgent`对象创建了新的`ClientHandler`
* 对`mpAgent`对象进行了`ClientHandler`中的线程初始化

```cpp
ClientHandler::ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, dbptr pDB, mapptr pMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strCamFile, viewptr pViewer, bool bLoadMap)
    : mpVoc(pVoc),mpKFDB(pDB),mpMap(pMap),
      mNh(Nh),mNhPrivate(NhPrivate),
      mClientId(ClientId), mpUID(pUID), mSysState(SysState),
      mstrCamFile(strCamFile),
      mpViewer(pViewer),mbReset(false),
      mbLoadedMap(bLoadMap)
{
    // in case of nullptr
    if(mpVoc == nullptr || mpKFDB == nullptr || mpMap == nullptr || (mpUID == nullptr && mSysState == eSystemState::SERVER))
    {
        cout << ("In \" ClientHandler::ClientHandler(...)\": nullptr exception") << endl;
        throw estd::infrastructure_ex();
    }

    // in Map.h, set of Clients
    // list of ID
    mpMap->msuAssClients.insert(mClientId);

    // 3rd Party, g2o
    mg2oS_wcurmap_wclientmap = g2o::Sim3(); //identity transformation

    // load camera topic name to ROS parameter server
    // system state should be Client
    if(mSysState == eSystemState::CLIENT)
    {
        std::string TopicNameCamSub;
        // get camera topic name, link up with node name
        // load it in ROS parameter server
        mNhPrivate.param("TopicNameCamSub",TopicNameCamSub,string("nospec"));
        mSubCam = mNh.subscribe<sensor_msgs::Image>(TopicNameCamSub,10,boost::bind(&ClientHandler::CamImgCb,this,_1));
        // show camera topic, for debug if necessary
        cout << "Camera Input topic: " << TopicNameCamSub << endl;
    }
}
```

其主要内容是将图像的话题名加载到ROS的参数服务器上，以及一些基础的参数配置；

之后是初始化函数`InitializeThreads`

### Initialize Threads

其函数定义开始时，有一个宏定义判断

```cpp
#ifdef LOGGING
void ClientHandler::InitializeThreads(boost::shared_ptr<estd::mylog> pLogger)
#else
void ClientHandler::InitializeThreads()
#endif
```

如果LOGGING defined则直接用智能指针的plogger；

进入函数内，首先又是一个宏判断：

```cpp
    #ifdef LOGGING
    this->InitializeCC(pLogger);
    #else
    this->InitializeCC();
    #endif
```

指向了`InitializeCC`函数；

之后根据类型判断具体的初始化函数：

```cpp
    // choose Initializer: client or server
    if(mSysState == eSystemState::CLIENT)
    {
        this->InitializeClient();
    }
    else if(mSysState == eSystemState::SERVER)
    {
        this->InitializeServer(mbLoadedMap);
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
```

下面对`InitializeCC`，`InitializeClient`，`InitializeServer`三个函数进行分析；

### Initialize CC

