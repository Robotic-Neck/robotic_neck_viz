# robotic_neck_viz
ROS2 package to visualize the robot kinematics with RVIZ for simulation or digital using TF2 and URDF.

<p align="center">
  <img width="640" height="380" src="/docs/imgs/digital_twin.jpeg">
</p>

## Dependencies
* Framework: [ROS2 Humble (desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* [launch_utils](https://github.com/MonkyDCristian/launch_utils)
* [python3-pykdl](https://packages.ubuntu.com/focal/python3-pykdl)

## Install and Compile
**Note:** Install [launch_utils](https://github.com/MonkyDCristian/launch_utils) in your workspace before follow this step
```
cd <path to your workspace>/src/
git clone https://github.com/Robotic-Neck/robotic_neck_viz.git
cd ..
colcon build --packages-select robotic_neck_viz
```

## Install python3-pykdl dependencie
```
sudo apt install python3-pykdl
```

## Install ROS packages dependencies with rosdep
Install, init and update rosdep, in case you haven't
```
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
```
Install dependencies  
```
cd <path to your workspace>/
rosdep install -i --from-path src --rosdistro humble -y
```

## Demo
```
ros2 launch robotic_neck_viz robotic_neck_urdf.launch.py
```
Now you can control the platform orientantation by rqt_reconfig.

<p align="left">
  <img width="480" height="270" src="/docs/imgs/rqt_reconfig.png">
</p>

## Documentation

The IK are getting using the [Python Kinematics and Dynamics Library](https://packages.ubuntu.com/focal/python3-pykdl) in the [neck_joint_publisher](/robotic_neck_viz/neck_joint_publisher.py) node. 
This node publish the [JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html), which have a list of the names of the joints and their revolution (in radians) or prismatic (in meter) positions.

<p align="left">
  <img width="270" height="480" src="/docs/imgs/joint_msg.png">
</p>

If you want to learn how set roll and pirch parameters by code you can check this [tutorial](https://roboticsbackend.com/ros2-global-parameters/) about global paramters.