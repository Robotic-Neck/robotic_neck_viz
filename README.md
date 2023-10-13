# robotic_neck_viz
ROS2 package to visualize the robot kinematics with RVIZ for simulation or digital using TF2 and URDF.

<p align="center">
  <img width="640" height="480" src="neck_mec_sim.png">
</p>

## Dependencies
* Framework: [ROS2 Humble (desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* [launch_utils](https://github.com/MonkyDCristian/launch_utils)
* [python3-pykdl](https://packages.ubuntu.com/focal/python3-pykdl)

## Install python3-pykdl
```
sudo apt install python3-pykdl
```

## Install and Compile
**Note:** Install [launch_utils](https://github.com/MonkyDCristian/launch_utils) in your workspace before follow this step
```
cd <path to your workspace>
git clone https://github.com/Robotic-Neck/robotic_neck_viz.git
cd ..
colcon build --packages-select neck_mec_sim
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
cd caleuche_ws/
rosdep install -i --from-path src --rosdistro humble -y
```

## Demo
```
ros2 launch neck_meck_sim main.launch.py
```

## Documentation
TODO

## Authors
* Cristian Nova (cristian.nova@uc.cl)
* Name 2 (TODO)
