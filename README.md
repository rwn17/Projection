# Projection points into camera

Project Lidar points into camera and verify the results of calibration. The lidar point is colored according to their depth in the coordinate of camera. Read the parameters(extrinsic param, intrinsic param, file path) form .yaml file.

```
    git clone https://github.com/rwn17/Calibration.git
    mkdir build
    cd build
    cmake ..
    make
    ./project
 ```
