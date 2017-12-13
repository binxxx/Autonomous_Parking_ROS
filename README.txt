#Auto Parking
This project is aimed to implement A* base anytime search method to tackle modern parking space finding problem in a parking lot.
The following instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

#Prerequisites

	ROS-indigo

#Installing
Under the workspace directory run

	source devel/setup.bash
	catkin_make

#Deployment
Open another terminal and run the following command
	
	source devel/setup.bash
	roslaunch car_description final_project.launch

Now you should see the gazebo interface is opened and a dynamic obstacle is running on the map.
Open another terminal under the same directory and run
	
	source devel/setup.bash

Type in the following command and hit "tab" on your keyboard twice

	rosservice call /planner/trigger_planner 

you will see the following

	rosservice call /planner/trigger_planner "use_default: false
	goal:
	 x: 0.0
	 y: 0.0
	 z: 0.0"

For running the default goal pose where x y is 2D position and z is the car's orientation, change the "false" to "true".
Specify the x, y, z that you want the car to go. Then, hit "enter". Now you should see the car running in the gazebo interface. (Currently, our planner can only support to find the goal that is reachable)




