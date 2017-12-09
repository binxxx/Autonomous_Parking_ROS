#ifndef PLANNER_H
#define PLANNER_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <unordered_map>
#include <chrono>
#include<algorithm>
#include "bintree.h"
#define INF 1000000000000.0
#define INVALID_PRIM -1


class planner
{
private:
	struct node
	{
		float x_, y_, theta_; // x_, y_ in gazobo coord, theta_ in radian
		float g_, h_, f_, v_; // f_ needs to be calculated evert time
		int prim_;
		node* parent_;

		node(float x, float y, float theta, float g, float h, float f)
		:x_(x), y_(y), theta_(theta), g_(g), h_(h), f_(f)
		{
			v_ = INF;
			parent_ = nullptr;
			prim_ = INVALID_PRIM;
		}
	};

	int** map_original;
	int map[330][380];
	int x_size;
	int y_size;

	float eps;

	int nprim; // number of primitives, 8
	std::vector<std::vector<std::vector<float>>> primArray;

	// car constants
	float car_wid, car_len;
	float alpha, beta; // radian
	float car_exlen;
	float car_diag;
	float car_exdiag;

	// start and goal position
	float start_x, start_y, start_theta;
	float goal_x, goal_y, goal_theta;
	// int start_ID;
	// int goal_ID;
	float cost_list[8];

	//open, INCONS and closed list;
	BinaryTree<float, node*> open;
	std::unordered_map<int, BinaryTree<float, node*>::NodeHandle> ID2node;
	std::unordered_map<int, node*> closed;
	std::unordered_map<int, node*> INCONS;
	std::unordered_map<int, bool> visited;

	// START AND GOAL
	node* start_node;
	node* goal_node;


	inline int getMapID(float x, float y, float theta); //in gazebo coord, theta in radian

	float computeHeuristic(float x, float y, float theta); // in gazebo coord, theta in radian

	void computePath();

	bool applyAction(float curx, float cury, float curtheta,
					float& succx, float& succy, float& succtheta, int prim);
					// in gazebo coord, theta in radian
					// what is nprim????

	bool checkFeasibility(float x, float y,float theta);

	void trackPath(node* tmp_node);

	bool checkGoal(node* node);

	void update_map(float x, float y, float theta);

	void refresh();

	void foot_print(float x, float y, float theta, bool ext, std::vector<int> & vec);

	inline bool cont2discrete(float x, float y, int &mx, int &my);





public:
	void setGoal(float x, float y, float theta);
	void setStart(float x, float y, float theta);
	//void setPrimitives(std::vector<std::vector<std::vector<float>>> prim);
	void ARAstar(float ob_x, float ob_y, float ob_theta);
	void initialize(int** map_input, std::vector<std::vector<std::vector<float>>> prim);

	float publish_x, publish_y;
	float publish_theta;
	int publish_primID;
	std::vector<int> prim_set;

	std::chrono::steady_clock::time_point t1;
	std::chrono::steady_clock::time_point t2;
	std::chrono::duration<double> time_in_planning;



};



#endif
