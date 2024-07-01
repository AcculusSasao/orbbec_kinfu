# orbbec_kinfu

Article: https://link.medium.com/lyUaxcNURKb

## Build

Put OrbbecSDKConfig.cmake to your OrbbecSDK directory.
```
mkdir -p build && cd build
cmake -DOrbbecSDK_DIR=<Your OrbbecSDK directory>
make -j4
cd ..
```

## Run

```
$ build/OrbbecKinfu --help
usage: build/OrbbecKinfu [options]
 -a [align_mode(2)]  0:Disabled, 1:HW, 2:SW
 -k [kinfu_mode(0)]  0:Disabled, 1:depth, 2:colored
 -kc                  coarse in colored kinfu
 -kr                  reset kinfu if ICP fails.
 -ks [kinfu_show_mode(0)]  0: render, 1: +3D_View, 2: +normals
 -md [max_depth_mm(5000)]  max depth in mm
 -cloff               set openCL off
 -ss [show_scale(0.50)]  window show scale
 
keys:
  ESC : quit app
  s : save depth.ply in depth-kinfu, color.ply in colored-kinfu
  r : reset kinfu
  f : freeze 3D View / restore
```

## Reference

### Orbbec Femto Bolt

https://github.com/orbbec/OrbbecSDK  

### cv::kinfu

cv::kinfu::KinFu Class Reference  
https://docs.opencv.org/4.x/d8/d1f/classcv_1_1kinfu_1_1KinFu.html  
cv::colored_kinfu::ColoredKinFu Class Reference  
https://docs.opencv.org/4.x/d5/d61/classcv_1_1colored__kinfu_1_1ColoredKinFu.html  
codes, sample codes  
https://github.com/opencv/opencv_contrib/tree/4.x/modules/rgbd  

### Kinect

Azure Kinect - OpenCV KinectFusion Sample  
https://github.com/microsoft/Azure-Kinect-Samples/tree/master/opencv-kinfu-samples  
