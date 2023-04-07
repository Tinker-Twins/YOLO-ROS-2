# Darknet YOLO with ROS 2

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
