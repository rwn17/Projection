# Projection points into camera

Project Lidar points into camera image. The lidar point is colored according to their depth. Read the parameters(extrinsic param, intrinsic param, file path) form .yaml file.

```
    git clone https://github.com/rwn17/Calibration.git
    cd Projection/
    CHANGE THE FILE_PATH IN .YAML
    mkdir build
    cd build
    cmake ..
    make
    ./project
 ```

Projection result is shown as:

![image](https://github.com/rwn17/Calibration/blob/master/image_project.jpg)
