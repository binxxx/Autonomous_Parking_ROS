#include <planner.h>

void planner::initialize(int** map_input, std::vector<std::vector<std::vector<float>>> prim)
{
	map_original = map_input;
	primArray = prim;
	x_size = 380;
	y_size = 330;
	car_wid = 2.0;
	car_len = 5.0;
	car_exlen = car_len/2 + 0.5;
	alpha = atan(car_wid / car_len); // in radian
	beta = atan((car_wid/2) / car_exlen);
	car_diag = sqrt((car_wid/2)*(car_wid/2)+(car_len/2)*(car_len/2));
	car_exdiag = sqrt((car_wid/2)*(car_wid/2)+(car_exlen)*(car_exlen));

	cost_list[0] = 1.0;
	cost_list[1] = 0.5;
	cost_list[2] = 1.0;
	cost_list[3] = 0.5;
	cost_list[4] = 1.5;
	cost_list[5] = 1.5;
	cost_list[6] = 1.5;
	cost_list[7] = 1.5;

	eps = 10.0;

	nprim = 8;

	publish_primID = INVALID_PRIM;


}

void planner::setGoal(float x, float y, float theta)
{
	goal_x = x;
	goal_y = y;
	goal_theta = theta;
	goal_node = new node(x, y, theta, INF, 0.0, INF);
}

void planner::setStart(float x, float y, float theta) {
	start_x = x;
	start_y = y;
	start_theta = theta;
	float start_h = computeHeuristic(x, y, theta);
	start_node = new node(x, y, theta, 0, start_h, eps * start_h);
}

void planner::refresh()
{
	this->open = BinaryTree<float, node*>();
	ID2node.clear();
	closed.clear();
	INCONS.clear();
	visited.clear();
	eps = 10.0;
	goal_node->g_ = INF;
	goal_node->h_ = 0.0;
	goal_node->f_ = INF;
	t1 = std::chrono::steady_clock::now();
}

inline bool planner::cont2discrete(float x, float y, int &mx, int &my)
{
	mx = (int)(x/0.1);
	my = (int)(-y/0.1);

	if (mx<0 || mx>=x_size || my<0 || my>=y_size) return false;
	else return true;
}

void planner::foot_print(float x, float y, float theta, bool ext, std::vector<int> &vec)
{
	float point1_x = x + car_diag * cos(theta + M_PI + alpha);
	float point1_y = y + car_diag * sin(theta + M_PI + alpha);

	float vec_1_x = 0.1 * cos(theta);
	float vec_1_y = 0.1 * sin(theta);

	float vec_2_x = 0.1 * cos(theta + M_PI/2);
	float vec_2_y = 0.1 * sin(theta + M_PI/2);

	int hori_step;
	int ver_step;
	if (ext)
	{
		hori_step = (car_len/2 + car_exlen)/0.1;
	}
	else
	{
		hori_step = (car_len)/0.1;
	}
	ver_step = car_wid / 0.1;

	for (int i = 0; i < ver_step; i++)
	{
		for (int j = 0; j < hori_step; j++)
		{
			float current_x = point1_x + j * vec_1_x + i * vec_2_x;
			float current_y = point1_y + j * vec_1_y + i * vec_2_y;
			int mx, my;
			if (cont2discrete(current_x, current_y, mx, my)) //
			{
				// if valid
				vec.push_back(mx);
				vec.push_back(my);
			}
		}
	}
}



void planner::update_map(float x, float y, float theta)
{
	for (int i = 0; i < y_size; i++)
	{
		for (int j = 0; j < x_size; j++)
		{
			map[i][j] = map_original[i][j];
		}
	}
	std::vector<int> vec;
	foot_print(x,y, theta,true, vec);
	// ROS_INFO("updating map");
	// ROS_INFO("foot print size: %ld",vec.size());
	for (int i=0;i<vec.size()/2;i++)
	{
		map[vec[i*2+1]][vec[i*2]] = 1;
	}
}


int planner::getMapID(float x, float y, float theta)
{
	int mx, my;
	if (cont2discrete(x, y, mx, my))
	{
		return my * x_size * 36 + mx * 36 + (int)(theta / (2 * M_PI / 36)+0.5);
	}
	else
	{
		return -1;
	}
}


bool planner::checkGoal(node* node)
{
	// ROS_INFO("goal_x: %lf goal_y: %lf goal_theta: %lf", goal_x, goal_y, goal_theta);
	// ROS_INFO("potential node: goal_x: %lf goal_y: %lf goal_theta: %lf", node->x_, node->y_, node->theta_);

	return (getMapID(goal_x, goal_y, goal_theta) == getMapID(node->x_, node->y_, node->theta_));
}

bool planner::checkFeasibility(float x, float y,float theta)
{
	std::vector<int> vec;
	foot_print(x, y, theta, false, vec);
	for (int i = 0; i < vec.size()/2; i++)
	{
		if (map[vec[i*2+1]][vec[i*2]] == 1)
		{
			return false;
		}
	}
	return true;
}

float planner::computeHeuristic(float x, float y, float theta)
{
	return fabs(goal_x - x) + fabs(goal_y - y);
}


bool planner::applyAction(float curx, float cury, float curtheta,
					float &succx, float &succy, float &succtheta, int prim)
{
	// ROS_INFO("prim: %d", prim);
	int si = (int)(curtheta/(M_PI/18)+0.5);
	// ROS_INFO("si: %d", si);

	float x, y, theta;
	for (int i = 0; i < 6; i++)
	{
		// ROS_INFO("apply action %d", i);
		x = curx + primArray[si][prim][i*3];
		y = cury + primArray[si][prim][i*3+1];
		theta = primArray[si][prim][i*3+2];
		if (theta < 0) theta += 2 * M_PI;
		if (theta >= 2*M_PI) theta -= 2* M_PI;
		//ROS_INFO("checking feasibility");

		if (!checkFeasibility(x, y, theta))
		{
			return false;
		}
		if (x <= 0.0 || x>= 38.0 || y <= -33.0 || y >= 0)
		{
			return false;
		}

	}
	succx = x;
	succy = y;
	succtheta = theta;

	return true;
}

void planner::trackPath(node* tmp_node) {
	prim_set.clear();
	// std::unordered_map<int, node*>::const_iterator back_it = closed.find(getMapID(goal_x, goal_y, goal_theta));
	// node* tmp_node = back_it->second;
	//node* tmp_node = open.GetValue(open.First());
	int tmp_ID = getMapID(tmp_node->parent_->x_, tmp_node->parent_->y_, tmp_node->parent_->theta_);
	ROS_INFO("Prim: %d \n", tmp_node->prim_);
	prim_set.push_back(tmp_node->prim_);
	int start_ID = getMapID(start_x, start_y, start_theta);
	while (tmp_ID != start_ID) {
		tmp_node = tmp_node->parent_;
		tmp_ID = getMapID(tmp_node->parent_->x_, tmp_node->parent_->y_, tmp_node->parent_->theta_);
		prim_set.push_back(tmp_node->prim_);
	}
	// ROS_INFO("finish track path, prim id=%d", tmp_node->prim_);
	std::reverse(std::begin(prim_set), std::end(prim_set));
	publish_primID = tmp_node->prim_;

}

void planner::computePath()
{
	bool time_exit = false;
	int computepath_counter = 1;
	bool found = false;
	while (open.GetN() != 0 && goal_node -> f_ > open.GetValue(open.First())->f_) {
		// ROS_INFO("%d time in while loop",computepath_counter);
		computepath_counter+=1;
		// ROS_INFO("CLOSED SIZE: %ld", closed.size());

		t2 = std::chrono::steady_clock::now();
		time_in_planning = std::chrono::duration_cast<std::chrono::duration<double>> (t2-t1);
		if (time_in_planning.count() > 80.0)
		{
			time_exit = true;
			break;
		}

		// get the node with samllest f in open

		auto cur_nodeHd = open.First();
		node* cur_node = open.GetValue(open.First());
		// add this node to closed list
		cur_node->v_ = cur_node->g_;
		int cur_ID = getMapID(cur_node->x_, cur_node->y_, cur_node->theta_);
		closed[cur_ID] = cur_node;
		// erase this node from open
		open.Delete(cur_nodeHd);
		ID2node.erase(cur_ID);
		// check if this node is goal
		// if (checkGoal(cur_node)) {
		// 	found = true;
		// 	ROS_INFO("Path Found!");
		// 	break;
		// }
		// find successor
		for (int np = 0; np < nprim; np++) {
			float succ_x, succ_y, succ_theta;
			// ROS_INFO("before apply action");
			bool ret = applyAction(cur_node->x_, cur_node->y_, cur_node->theta_,
				                   succ_x, succ_y, succ_theta, np);
			// ROS_INFO("ret: %d", ret);
			if (ret) {
				int succ_ID = getMapID(succ_x, succ_y, succ_theta);
				std::unordered_map<int, bool>::const_iterator v_it = visited.find(succ_ID);
				node* succ_node;
				// never visited
				if (v_it == visited.end()) {
					// ROS_INFO("never visited");
					float succ_h = computeHeuristic(succ_x, succ_y, succ_theta);
					succ_node = new node(succ_x, succ_y, succ_theta, INF, succ_h, INF);

					// // liu's code
					// ROS_INFO("node: %d: x: %lf  y:%lf  theta:%lf", np, succ_node->x_, succ_node->y_, succ_node->theta_);
					// ROS_INFO("node id: %d", getMapID(succ_node->x_, succ_node->y_, succ_node->theta_));
					// // end liu's code
					succ_node->parent_ = cur_node;
					succ_node->prim_ = np;
					visited[succ_ID] = true;
				} else {
					// ROS_INFO("visited");
					// visited
					// find in closed or in open
					std::unordered_map<int, node*>::const_iterator closed_it = closed.find(succ_ID);
					if (closed_it != closed.end()) {
						// ROS_INFO("find in closed");
						succ_node = closed_it->second;
					}
					else
					{
						std::unordered_map<int, BinaryTree<float, node*>::NodeHandle>::const_iterator id2_it = ID2node.find(succ_ID);
						if (id2_it != ID2node.end()) {
							// ROS_INFO("find in open");
							auto hd = id2_it->second;
							succ_node = open.GetValue(hd);
						}
						else
						{
							// ROS_WARN("VISITED BUT NOT FOUND IN CLOSED OR OPEN");
						}
				  }


				}
				// check g value
				if (succ_node->g_ > cur_node->g_ + cost_list[np]) {
					succ_node->g_ = cur_node->g_ + cost_list[np];
					succ_node->f_ = succ_node->g_ + eps * succ_node->h_;
					succ_node->parent_ = cur_node;
					succ_node->prim_ = np;
					//if it's not in closed list
					std::unordered_map<int, node*>::const_iterator closed_it = closed.find(succ_ID);
					if (closed_it == closed.end()) {
						// check if it's in open
						std::unordered_map<int, BinaryTree<float, node*>::NodeHandle>::const_iterator id2_it = ID2node.find(succ_ID);
						if (id2_it == ID2node.end()) {
							// not in open, insert in open
							auto hd = open.Insert(succ_node->f_, succ_node);
							ID2node[succ_ID] = hd;
						} else {
							// in open, update
							auto tmp_hd = id2_it->second;
							open.Delete(tmp_hd);
							auto succ_nodeHd = open.Insert(succ_node->f_, succ_node);
							ID2node[succ_ID] = succ_nodeHd;
						}
					} else {
						// in closed list
						// check if it's in INCONS
						INCONS[succ_ID] = succ_node;
					}

				}
				if (succ_ID == getMapID(goal_x, goal_y, goal_theta)) {
					// ROS_INFO("succ_x: %lf, succ_y: %lf, succ_theta: %lf", succ_x, succ_y, succ_theta);
					// ROS_INFO("succ_f: %lf, succ_g: %lf, succ_h: %lf, ",succ_node->f_,succ_node->g_,succ_node->h_);
					goal_node->f_ = succ_node->f_;
					goal_node->g_ = succ_node->g_;
					goal_node->h_ = succ_node->h_; // yuzhang
					ROS_INFO("goal in open- asdfasfasdfsa asdfasd fa df a f ad a fs  fas d as fads f as fdsa f a sfd a");

				}
				// if ((succ_x < goal_x + 0.2) && (succ_x > goal_x - 0.2) && succ_y > goal_y - 0.2 && succ_y < goal_y + 0.2) {
				// 	// ROS_INFO("succ_x: %lf, succ_y: %lf, succ_theta: %lf", succ_x, succ_y, succ_theta);
				// 	// ROS_INFO("succ_f: %lf, succ_g: %lf, succ_h: %lf, ",succ_node->f_,succ_node->g_,succ_node->h_);
				// 	goal_node->f_ = succ_node->f_;
				// 	goal_node->g_ = succ_node->g_;
				// 	goal_node->h_ = succ_node->h_; // yuzhang
				// 	goal_x = succ_x;
				// 	goal_y = succ_y;
				// 	goal_theta = succ_theta;
				// 	ROS_INFO("goal in open- asdfasfasdfsa asdfasd fa df a f ad a fs  fas d as fads f as fdsa f a sfd a");
        //
				// }

			}
		}
	}

	if (time_exit)
	{
		ROS_INFO("EXIT DUE TO TIME!");
		return;
	}

	// ROS_INFO("open get N: %ld", open.GetN());
	// ROS_INFO("closed size: %ld", closed.size());
	// ROS_INFO("open first f: %lf, open first g: %lf, open first h: %lf",open.GetValue(open.First())->f_,open.GetValue(open.First())->g_,open.GetValue(open.First())->h_);
	if (open.GetN()) {
		auto it = ID2node.find(getMapID(goal_x, goal_y, goal_theta));
		if (it != ID2node.end())
		{
			trackPath(open.GetValue(it->second));
		}
		else
		{
			ROS_INFO("path not found");
			publish_primID = -1;  // -1
		}
		//checkGoal(open.GetValue(open.First()))
		//found = true;
		//ROS_INFO("Path Found!");
		//trackPath();
	}
	else
	{
		publish_primID = -1; // -1
		ROS_INFO("failed to find goal");
	}
	// if (found) {
	// 	trackPath();
	// } else {
	// 	publish_primID = -1;
	// }
}

void planner::ARAstar(float obx, float oby, float ob_theta) {
	refresh();
	// ROS_INFO("finish refresh");
	update_map(obx, oby, ob_theta);
	// ROS_INFO("finish update map");

	// ROS_INFO("start node x: %lf, start node y: %lf,start node theta: %lf", start_node->x_, start_node->y_,start_node->theta_);
	// ROS_INFO("goal node x: %lf, goal node y: %lf,goal node theta: %lf", goal_node->x_, goal_node->y_,goal_node->theta_);
	// ROS_INFO("Current #: %ld", open.GetN());
	auto start_handle = open.Insert(start_node->f_, start_node);
	ID2node[getMapID(start_x, start_y, start_theta)] = start_handle;
	// ROS_INFO("FUCKKKKKKKKKKKKKKKKKKK");

	// liu's code
  ROS_INFO("goal node id: %d", getMapID(goal_node->x_, goal_node->y_, goal_node->theta_));
	computePath();
	// ROS_INFO("finish compute path");

	if (publish_primID == -1) return;

	while (eps >= 4.0)
	{
		eps = eps - 3.0;
		ROS_INFO("CURRENT _________________________________________eps: %lf", eps);
		for (auto incons_node : INCONS)
		{
			auto incons_node_leaf = open.Insert(incons_node.second->f_, incons_node.second);
			int incons_node_key = getMapID(incons_node.second->x_, incons_node.second->y_, incons_node.second->theta_);
			ID2node[incons_node_key] = incons_node_leaf;
		}
		INCONS.clear();
		visited.clear();

		goal_node->f_ = goal_node->g_ + eps * goal_node->h_;
		for (auto open_node : ID2node)
		{
			node* temp_node;
			auto temp_node_hd = open_node.second;
			auto temp_node_id = open_node.first;
			temp_node = open.GetValue(temp_node_hd);
			open.Delete(temp_node_hd);
			temp_node->f_ = temp_node->g_ + eps * temp_node->h_;
			temp_node_hd = open.Insert(temp_node->f_, temp_node);
			ID2node[temp_node_id] = temp_node_hd;
		}
		closed.clear();
		computePath();
		if (publish_primID == -1) return;
	}
}
