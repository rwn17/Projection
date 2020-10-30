# Calibration Manual for Lidar-Camera
This manual is based on the Autoware autonomous driving platform and only used within the lab. This manual is learned from [CSDN LINK](https://blog.csdn.net/AdamShan/article/details/81670732) and [blog link](https://www.cnblogs.com/dlonng/p/13091956.html). If you know Chinese, thoss blogs are more helpful. Also, if you If you think I am infringing, please contact me to delete it. My email is rwn17@mails.tsinghua.edu.cn.

## 1.Environmental setup

### 1.1 Download the  toolkit
My environment is Ubuntu 18.04, openCV 3.3.1, PCL 1.8;

Before download the calibration tool, we have to install **nlopt** package first, which is used to perform nonlinear optimization. Please follow the instructions on [nlopt](https://github.com/stevengj/nlopt).

Download the calibration tool that are seperated from Autoware automous driving platform. Just follow the instructions on [Autoware](https://github.com/XidianLemon/calibration_camera_lidar). If you hope to down load the whole Autoware platform, that's ok, but it takes longer time.

### 1.2 Preprocess the rosbag

Autoware will subscribe **"raw image"** type, rather than **"compressed image"**, so if you use the compressed the camera image in the rosbag, you have to republish it. The image_trasport package could help us to make it, just type:

```
rosrun image_transport republish compressed in:=COMPRESSED_TOPIC raw out:=camera/image_repub
```

The will build a node that been republishing the compressed topic.

Also, the autoware will only subscribe the topic "/points_raw" as lidar points. So when rosplaying your bag, you have to do:

```
rosbag play bagName.bag /LIDAR_TOPIC:=/points_raw
```


## 2 Calibrate


### 2.1 Start the toolkit
Run ```rosrun calibration_camera_lidar calibration_toolkit``` to start the toolkit, choose the image topic you republish and choose camera->Lidar mode. If everything is all right, you could see the gui like:
![cali GUI](https://img-blog.csdn.net/20180814171727556?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

The left picture is the camera image and the right picture is lidar points. Choose the proper pattern and size on the top.
![pattern&size](https://img-blog.csdn.net/20180814171714812?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

Noted that the pattern number means the size of "INTERIOR", if you have a 7*9 board, please type 6*8.

### 2.2 Camera-Lidar correspondence

If everything goes right. When you click "grab", the system could capture the image and Lidar Points like:
![capture](https://img-blog.csdn.net/2018081417182412?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
The left image named "image_0" should have colorde line on the edge of each individual block. And you may see nothing on the left image named "velodyne_0". Next let we find the lidar points on velodyne_0.

First right click the velodyne_0, and use your keyboard to change the view. The rules are:
![](https://img-blog.csdn.net/20180814171749184?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
![](https://img-blog.csdn.net/20180814171759608?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/basic_operations.png)

When you adjust to the proper view, you could left click the center of the board in velodyne_0 and you could see the area you click becomes red,just as the right picture shows.
![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/CalibrationToolKitExample.png)

If you click the wrong area, right click and you could choose the region again. After you get 7~10 correspondences, click the "calibrate" button, you could get the extrinsic parameter and intrinsic parameter. Then you could click the "project" button to project the area you choose in the lidar point to the image. If the red area is in the middle of the calibration board, the calibration is fine. Just as following pic shows:
[right cali](https://img-blog.csdn.net/20180814171900690?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
[wrong cali](https://img-blog.csdn.net/20180814171911776?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0FkYW1TaGFu/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

To find out more about this boolkit, please read the real usr manual, whose path is /calibration_camera_lidar/blob/master/ls_calibration/calibration_camera_lidar/CalibrationToolkit_Manual.pdf under the path you git clone [Autoware](https://github.com/XidianLemon/calibration_camera_lidar).

## 3.Verification
After you calibrate the parameter, you could project the lidar point to the image to check whether the calibration is right. I have provided an [image frame](https://github.com/rwn17/Calibration/blob/master/1599018242717845172.jpg) and an [lidar point frame](https://github.com/rwn17/Calibration/blob/master/1599018242.718678000.pcd) in the repo.  The projection.cpp could project the lidar point on the image. If everything goes right, the projection result would look like.

