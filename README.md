# posture_stabilization

![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![C++](https://img.shields.io/badge/-C%2B%2B-00599C?style=plastic&logo=C%2B%2B)
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)

A collection of ROS1 packages pertaining to my final dissertation at University of Sheffield.

## Installation

* Install the python dependencies:

    ```bash
    sudo apt-get install \
        python3-pip \
        python3-vcstools \
        python3-rosinstall
    ```

* Get all the required repositories:

    ```bash
    rosinstall src /opt/ros/noetic actor_modelling.rosinstall
    ```

* Install the pip dependencies:

    ```bash
    pip install -r requirements.txt
    ```

* Install all the ros dependencies:

    ```bash
    rosdep install --from-paths src --ignore-src -r -yqqq
    ```

* Install all the qt5 dependencies:

    ```bash
    sudo apt-get install \
        libqt5datavisualization5-dev \
        libqt5charts5-dev
    ```

* Build the packages:

    ```bash
    catkin build gazebo_ros_actor_plugin calibration_imu actor_modelling
    ```

## Usage

* Launch the simulation for viewing the actor limbo:

    ```bash
    roslaunch actor_modelling animation_switch.launch
    ```
  To switch between the different animations, use the numerical keys 1-2.

* To view a live sync between NASA's Valkyrie robot and the XSens unit, launch the following:

    ```bash
    roslaunch actor_modelling torso_rviz.launch
    ```

* For calibrating the IMU, and estimating the hard-iron and soft-iron distortions, run the following:

    ```bash
    rosrun calibration_imu magnetometer
    ```
    The calibration GUI will appear, and you rotate the IMU away from any magnetic interference to collect data. Stop the data collection after an ellipsoid is formed around the data points.
