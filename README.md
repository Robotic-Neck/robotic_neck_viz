# robotic_neck_viz
ROS2 package to visualize the robot kinematics with RVIZ for simulation or digital using TF2 and URDF.

<p align="center">
  <img width="640" height="380" src="/docs/imgs/digital_twin.png">
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
TODO
```

## Documentation
TODO
