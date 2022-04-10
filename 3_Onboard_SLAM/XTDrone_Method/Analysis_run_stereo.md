# Analysis on `xtdrone_run_stereo.sh`

`rosrun ORB_SLAM2 Stereo` add below sentences:

* ` ~/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt`, `ORB` staff
* `~/catkin_ws/src/ORB_SLAM2/Examples/Stereo/px4_sitl.yaml true`
* `/camera/left/image_raw:=/iris_0/stereo_camera/left/image_raw`
* `/camera/right/image_raw:=/iris_0/stereo_camera/right/image_raw`
* `/orbslam2/vision_pose/pose:=/iris_0/mavros/vision_pose/pose`

## `ORB SLAM2`  - `ORBvoc.txt`

 `ORB` staff, for data configuration

## `px4_sitl.yaml` true

### file `px4_sitl.yaml`

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV) 
Camera.fx: 376.0
Camera.fy: 376.0
Camera.cx: 375.87198639
Camera.cy: 240.0

Camera.k1: 0
Camera.k2: 0
Camera.p1: 0
Camera.p2: 0

Camera.width: 752
Camera.height: 480

# Camera frames per second 
Camera.fps: 20

# stereo baseline times fx
Camera.bf: 45.12

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# Close/Far threshold. Baseline times.
ThDepth: 48

#--------------------------------------------------------------------------------------------
# Stereo Rectification. Only if you need to pre-rectify the images.
# Camera.fx, .fy, etc must be the same as in LEFT.P
#--------------------------------------------------------------------------------------------
LEFT.height: 480
LEFT.width: 752
LEFT.D: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [-0.01, 0.004, 5e-5, -1e-4, 0.0]
LEFT.K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [376.0, 0.0, 376.0, 0.0, 376.0, 240.0, 0.0, 0.0, 1.0]
LEFT.R:  !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
LEFT.P:  !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [376.0, 0, 375.87198639, 0, 0, 376.0, 240.04484177, 0,  0, 0, 1, 0]

RIGHT.height: 480
RIGHT.width: 752
RIGHT.D: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [-0.01, 0.004, 5e-5, -1e-4, 0.0]
RIGHT.K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [376.0, 0.0, 376.0, 0.0, 376.0, 240.0, 0.0, 0.0, 1.0]
RIGHT.R:  !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
RIGHT.P:  !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [376.0, 0, 375.87198639, -45.12, 0, 376.0, 240.04484177, 0,  0, 0, 1, 0]

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1200

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize:2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
```

`yaml` is kind of `ros` parameters file, to dump, load, get or delete;

header said Camera Parameters, why called `px4_sitl`? how to connect with `px4`?

* no idea
* nothing connected with `px4`

**add camera parameters to `px4`**

## camera left and right configuration

* `/camera/left/image_raw:=/iris_0/stereo_camera/left/image_raw`
* `/camera/right/image_raw:=/iris_0/stereo_camera/right/image_raw`

topic names, try to use monocular camera, topic id might be similiar.

## `vision_pose`

 `/orbslam2/vision_pose/pose:=/iris_0/mavros/vision_pose/pose`

send vision pose that `orb slam2` get to `mavros`, link up with drone.

significant step to get drone's pose infomation.

