Autonomous_Parking_ROS
======
## Contributor: Bin Xu, Bosch Tang, Nicewell Su, Yuzhang Liu, Yihui Wang
## Installation
- Clone the package into your_catkin_workspace/src
```
git clone https://github.com/binxxx/Autonomous_Parking_ROS.git
```
- Redirect to your_catkin_workspace, build your workspace
```
cd ..
catkin_make
```
## Usage
- Start whole simulation
```
roslaunch car_description final_project.launch
```
- To specify the goal and start planning
```
rosservice call /planner/trigger_planner "use_default: true
goal:
x: 0.0
y: 0.0
z: 0.0"
```
If you want to change the default goal, you can specify its x y position and z is its orientation, and change the "true" to "false".
