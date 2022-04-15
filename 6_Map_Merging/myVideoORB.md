# 使用本地视频跑ORB SLAM2

参考：

* https://blog.csdn.net/zhangqian_shai/article/details/88406981
* https://blog.csdn.net/u010128736/article/details/53079964
* https://zhuanlan.zhihu.com/p/29629824

## 1.  学习`mono_tum.cc`

该程序完成了数据集的读取，这就意味着需要自己制作数据集；

或者，把数据转化成`rosbag`的形式，后者看起来更加方便一点；但是可能文件夹会变得巨大；

### 1.1 制作`rosbag`

