// #include "planner.h"
#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <stdlib.h>

#include <gazebo_interface/Motion.h>
#include <gazebo_interface/Environment.h>

#include <boost/thread/shared_mutex.hpp>

// Define the msg that we want to subscribe which is the environment info
gazebo_interface::Environment env;
boost::shared_mutex env_lock;

// Callback function for callBack function
void envCallback(const gazebo_interface::Environment::ConstPtr& envPtr){
	// ROS_INFO("[Planner] The planner heard the environment. ");
    boost::shared_lock<boost::shared_mutex> lock(env_lock);
    env = *envPtr;
}


//Run planner in main function and perform publisher and subscriber
int main(int argc, char **argv) {

	


	//Initialize ROS node
	ros::init(argc, argv, "Planner");
	ros::NodeHandle n;
	//Initialize publisher
	ros::Publisher pose_pub = n.advertise<gazebo_interface::Motion>("position_update", 1000);

	//Initialize subscriber
	ros::Subscriber envSub = n.subscribe("/gazebo_interface/env", 1000, envCallback);
	// ROS_INFO("[Planner] Subscriber initialized. ");

	//Initialize msg for publishing
	gazebo_interface::Motion firstPrimAndID;

	//lock for firstPrimAndID
	boost::shared_mutex fpid_lock;

	ros::Rate loop_rate(10);

	//Get map from server
	float** map;
	std::vector<float> map_incoming;
	int x_size;
	int y_size;
	int grid_size;
	if(!n.getParam("/global/map", map_incoming)){
		ROS_WARN("[Planner] Unable to load map.");
	}
	if(!n.getParam("/global/map_metric/x_size", x_size)){
		ROS_WARN("[Planner] Unable to load x_size.");
	}
	if(!n.getParam("/global/map_metric/y_size", y_size)){
		ROS_WARN("[Planner] Unable to load y_size.");
	}
	if(!n.getParam("/global/map_metric/grid_size", grid_size)){
		ROS_WARN("[Planner] Unable to load grid_size.");
	}

	ROS_INFO("[Planner] Initialize map.");
	//Generate map pointer
	map = new float*[y_size];
	for(int i = 0; i<y_size;i++){
		map[i] = new float[x_size];
	}

	//Assign the map into storage
	for(int i =0;i<y_size;i++){
		for(int j=0;j<x_size;j++){
			map[i][j] = map_incoming[i*x_size+j];
			// ROS_INFO("[Planner] Add map element %d,[%f]",i*x_size+j,map[i][j]);
		}
	}



	int count = 0;
	while (ros::ok()){
		
		//Assign value into the publisher msg
		{
	        // robot position
	        boost::shared_lock<boost::shared_mutex> lock(fpid_lock);
	        
	        //////////////////////////////
			// Assign values to message //
			// firstPrimAndID.cur_pos.x //
			// firstPrimAndID.cur_pos.y //
			// firstPrimAndID.cur_pos.z //
			// firstPrimAndID.primitive //
			//////////////////////////////
      	}
		

		pose_pub.publish(firstPrimAndID);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	







	//Release memory of the map
	for(int i=0;i<y_size;i++){
		delete [] map[i];
	}
	delete [] map;


	return 0;
}