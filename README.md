# Darknet YOLO with ROS 2

| ![Stop Sign Detection Simulation](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Capstone%20Project/media/stop_sign_detection_sim.gif) | ![Stop Sign Detection Robot](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Capstone%20Project/media/stop_sign_detection_real_robot.gif) | ![Stop Sign Detection RViz](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Capstone%20Project/media/stop_sign_detection_real_rviz.gif) |
|:---------------------------------:|:------------------------------------:|:------------------------------------:|
| Stop Sign Detection - Simulation | Stop Sign Detection - TurtleBot3 | Stop Sign Detection - Remote PC |

**Note:** The above demonstrations use [this repository](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems) for controlling TurtleBot3 to obey the stop sign.

## Build:
1. Make a directory `ROS2_WS` to act as your ROS 2 workspace.
    ```bash
    $ mkdir -p ~/ROS2_WS/src/
    ```
2. Clone this repository:
    ```bash
    $ git clone https://github.com/Tinker-Twins/YOLO-ROS-2.git
    ```
3. Install [`OpenCV`](https://opencv.org) (or [build from source](https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html)).
    ```bash
    $ sudo apt update
    $ sudo apt install libopencv-dev python3-opencv
    ```
4. Build the ROS packages (build in `Release` mode to maximize performance).
    ```bash
    $ cd ~/ROS2_WS
    $ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
5. Source the `setup.bash` file of your `ROS2_WS`.
    ```bash
    $ echo "source ~/ROS2_WS/install/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

## Execute:
```bash
$ ros2 launch darknet_ros darknet_ros.launch.py
```

## Configure:
- Installation configuration can be managed from `CMakeLists.txt`.
    - `darknet_ros/darknet_ros/darknet/CMakeLists.txt`
        ```cmake
        # CUDA/cuDNN Settings
        option(ENABLE_CUDA "Enable CUDA support" OFF)
        option(ENABLE_CUDNN "Enable CUDNN" OFF)
        option(ENABLE_CUDNN_HALF "Enable CUDNN Half precision" OFF)
        ```
    - `darknet_ros/darknet_ros/darknet_ros/CMakeLists.txt`
        ```cmake
        # CUDA/cuDNN Settings
        set(CUDA_ENABLE OFF)
        set(CUDNN_ENABLE OFF)
        set(FP16_ENABLE OFF)
        
        # YOLO Pre-Trained Model (Weights) Settings
        set(DOWNLOAD_YOLOV2_TINY OFF)
        set(DOWNLOAD_YOLOV3 OFF)
        set(DOWNLOAD_YOLOV4 OFF)
        set(DOWNLOAD_YOLOV4_CSP ON)
        set(DOWNLOAD_YOLOV4_TINY ON)
        set(DOWNLOAD_YOLOV4_MISH OFF)
        set(DOWNLOAD_YOLOV7_TINY ON)
        ```
- Names and other parameters of the publishers, subscribers and actions can be modified from `darknet_ros/config/ros.yaml`.
- Parameters related to YOLO object detection algorithm can be modified from `darknet_ros/darknet_ros/darknet_ros/config/yolo.yaml`.
- It is recommended to create a copy of the existing configuration file(s) as a template and do necessary modifications.
- Reference the updated configuration file(s) in `darknet_ros/darknet_ros/darknet_ros/launch/darknet_ros.launch.py`.
