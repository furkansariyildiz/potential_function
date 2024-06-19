# potential_function

<p align="center">
  <img src="documents/function.png" style="width: 50%; height: 50%"/>
</p>

### Clonning potential_function package to your workspace
```bash
cd ~ros2_ws/src
git clone git@github.com:furkansariyildiz/potential_function.git
```

### Building package
```bash
source /opt/ros/$ROS_ENV/setup.bash
colcon build --symlink-install --packages-select potential_function
```

### Running ros2 potential_function package
```bash
ros2 launch potential_function potential_function.launch.py
```

### Multirobot guideline
https://medium.com/@arshad.mehmood/efficient-deployment-and-operation-of-multiple-turtlebot3-robots-in-gazebos-f72f6a364620


<p align="center">
  <img src="documents/first_positions.png" alt="First position" style="width: 50%; height: 50%"/>
  <p>First position for all robots.</p>
</p>


<p align="center">
  <img src="documents/last_positions.png" alt="Last position" style="width: 50%; height: 50%"/>
  <p>Last position for all robots.</p>
</p>