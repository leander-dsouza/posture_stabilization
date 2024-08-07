# posture_stabilization
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
    rosdep install --from-paths src --ignore-src -r -y
    ```

* Build the packages:

    ```bash
    catkin build gazebo_ros_actor_plugin actor_modelling
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
