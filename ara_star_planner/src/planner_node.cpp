/*
 * this is our planner
 */

#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/thread/shared_mutex.hpp>

#include <geometry_msgs/Point32.h>
#include <gazebo_interface/Motion.h>
#include <gazebo_interface/Environment.h>
#include <ara_star_planner/SetGoal.h>

#include <planner.h>

class ARAPlanner {
 private:
  // ROS
  ros::NodeHandle nh, pnh;
  ros::Publisher pose_pub;
  ros::Subscriber envSub;
  ros::ServiceServer trigger_planner;
  // environmental information
  gazebo_interface::Environment env;
  boost::shared_mutex env_lock;

  gazebo_interface::Motion firstPrimAndID;
  boost::shared_mutex fpid_lock;

  // planner
  geometry_msgs::Point32 goal;

  // map
  int x_size;
  int y_size;
  int grid_size;
  int ** map;

  // primitives
  int numof_dirs, primitives_per_dir, waypoints_per_primitive, waypoint_dim;
  std::vector<std::vector<std::vector<float> > > primitives;

  // seq
  unsigned int seq;
  geometry_msgs::Point32 current_position;
  unsigned int next_primitive;

  //Declare class
  planner LIF_planner;


 private:
  /*
   * Function @ envCallback
   * this is the call back function when we
   * get a frame of environment information
   */
  void envCallback(const gazebo_interface::Environment::ConstPtr& envPtr){
    // save the frame to local variable
    boost::upgrade_lock<boost::shared_mutex> lock(this->env_lock);
    boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(lock);
    this->env = *envPtr;
  }

  /*
   * Function @ get_map
   * this function get the 2D map from the server
   */
  bool get_map(void) {
    std::vector<float> map_incoming;

    if(!nh.getParam("/global/map", map_incoming)){
      ROS_FATAL("[Planner] Unable to load map.");
      return false;
    }
    if(!nh.getParam("/global/map_metric/x_size", this->x_size)){
      ROS_FATAL("[Planner] Unable to load x_size.");
      return false;
    }
    if(!nh.getParam("/global/map_metric/y_size", this->y_size)){
      ROS_FATAL("[Planner] Unable to load y_size.");
      return false;
    }
    if(!nh.getParam("/global/map_metric/grid_size", this->grid_size)){
      ROS_FATAL("[Planner] Unable to load grid_size.");
      return false;
    }

    ROS_INFO("[Planner] Initializing map...");
    //Generate map pointer
    this->map = new int* [this->y_size];
    for(int i = 0; i < this->y_size; i++){
      this->map[i] = new int[this->x_size];
    }

    //Assign the map into storage
    for(int i =0; i < this->y_size; i++){
      for(int j=0; j < this->x_size; j++){
        this->map[i][j] = (int)map_incoming[i * this->x_size + j];
      }
    }

    ROS_INFO("[Planner] Done.");
    return true;
  }

  /*
   * Function @ get_primitives
   * this function fetches and interpolates
   * motion primitives from the parameter server
   */
  bool get_primitives(void) {
    // how many dirs? primitives per dir?
    if (!this->nh.getParam("/global/primitives/num_of_dirs", this->numof_dirs)) {
      ROS_ERROR("[Planner] # of primitives not specified. Assume to be 36?");
      this->numof_dirs = 36;
    }
    if (!this->nh.getParam("/global/primitives/primitives_per_dir", this->primitives_per_dir)) {
      ROS_ERROR("[Planner] # of primitives per direction not specified. Assume to be 8?");
      this->primitives_per_dir = 8;
    }
    if (!this->nh.getParam("/global/primitives/waypoints_per_primitive", this->waypoints_per_primitive)) {
      ROS_ERROR("[Planner] # of waypoints per primitive not specified. Assume to be 6?");
      this->waypoints_per_primitive = 6;
    }
    if (!this->nh.getParam("/global/primitives/wapoint_dimension", this->waypoint_dim)) {
      ROS_ERROR("[Planner] waypoint dimension not specified. Assume to be 3?");
      this->waypoint_dim = 3;
    }
    std::vector<float> primitives_array;
    if (!this->nh.getParam("/global/primitives/motion_primitives", primitives_array)) {
      ROS_FATAL("[Planner] primitives array not found! No motion primitives are used!!");
      return false;
    }

    // checking validness of this primitive array
    ROS_INFO("[Planner] Checking validness of primitives_array...");
    if (primitives_array.size() != (this->numof_dirs * this->primitives_per_dir * this->waypoints_per_primitive * this->waypoint_dim)) {
      ROS_FATAL("[Planner] primitives_array not valid. Size should be (%d x %d x %d x %d)", this->numof_dirs, this->primitives_per_dir, this->waypoints_per_primitive, this->waypoint_dim);
      return false;
    }
    ROS_INFO("[Planner] Done.");

    // got primitives, parse this huge array
    ROS_INFO("[Planner] Interpreting this HUGE primitive matrix...");
    std::vector<float>::const_iterator contents = primitives_array.begin();
    for (int dir_index = 0; dir_index < this->numof_dirs; dir_index ++) {
      std::vector<std::vector<float> > primitives_each_direction;
      for (int primitive_index = 0; primitive_index < this->primitives_per_dir; primitive_index ++) {
        std::vector<float> trajectory(contents, contents + this->waypoints_per_primitive * this->waypoint_dim);
        primitives_each_direction.push_back(trajectory);
        contents += this->waypoints_per_primitive * this->waypoint_dim;
      }

      // done for this direction
      this->primitives.push_back(primitives_each_direction);
    }
    ROS_INFO("[Planner] Done.");

    return true;
  }

  //Call the publisher
  void forward(void) {
    gazebo_interface::Motion command;

    // header
    command.header.seq = ++ this->seq;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "";

    // current position of the robot (currently not used)
    command.cur_pos = this->current_position;

    // primitive (this is what matters)
    command.primitive = this->next_primitive;

    // publish
    ROS_INFO("[Planner] Next motion primitive is %d.", this->next_primitive);
    this->pose_pub.publish(command);

    return;
  }

  float distance2goal(geometry_msgs::Point32 &cur) {
    return sqrt(pow((cur.x - this->goal.x), 2) + pow((cur.y - this->goal.y), 2));
  }

  bool goal_reached(geometry_msgs::Point32 &cur) {
    return (fabs(cur.x - this->goal.x) < 0.5 && fabs(cur.y - this->goal.y) < 0.5
              && fabs(cur.z - this->goal.z) < 0.1745);
  }

 public:
  /* Constructor */
  ARAPlanner() : pnh("~"), seq(0) {
    ROS_INFO("[Planner] Initializing ARAPlanner...");

    ROS_INFO("[Planner] Retrieving map...");
    if (!this->get_map()) {
      ROS_FATAL("[Planner] Unable to get map. Shutdown this planner!!");
      ros::shutdown();
    }
    ROS_INFO("[Planner] Done.");

    ROS_INFO("[Planner] Retrieving motion primitives...");
    if (!this->get_primitives()) {
      ROS_FATAL("[Planner] Unable to get motion primitives. Shutdown this planner!!");
      ros::shutdown();
    }
    ROS_INFO("[Planner] Done.");

    ROS_INFO("[Planner] Retrieving default goal position...");
    if (!this->pnh.getParam("goal_x", this->goal.x)) {
      ROS_WARN("[Planner] Default goal_x not specified.");
    }
    if (!this->pnh.getParam("goal_y", this->goal.y)) {
      ROS_WARN("[Planner] Default goal_y not specified.");
    }
    if (!this->pnh.getParam("goal_yaw", this->goal.z)) {
      ROS_WARN("[Planner] Default goal_yaw not specified.");
    }

    ROS_INFO("[Planner] Initialize LIF_planner...");
    this->LIF_planner.initialize(this->map, this->primitives);

    ROS_INFO("[Planner] Initializing services...");
    this->trigger_planner = this->nh.advertiseService("trigger_planner", &ARAPlanner::CallPlanner, this);
    ROS_INFO("[Planner] Done.");

    ROS_INFO("[Planner] Initializing topics...");
    // publish to gazebo_interface
    this->pose_pub = this->nh.advertise<gazebo_interface::Motion>("position_update", 5);
    // subscribe to environment topic
    this->envSub = nh.subscribe("/gazebo_interface/env", 5, &ARAPlanner::envCallback, this);
    ROS_INFO("[Planner] Done.");

    ROS_INFO("[Planner] OK! ARAPlanner is happy, wait for my plans, mate!");
  }

  bool CallPlanner(ara_star_planner::SetGoal::Request &req,
                    ara_star_planner::SetGoal::Response &res) {
    if (!req.use_default) {
      this->goal = req.goal;
    }

    // pre-processing the goal (theta should be within (0 - 2 * M_PI))
    if (this->goal.z < 0) this->goal.z += 2 * M_PI;
    else if (this->goal.z >= 2 * M_PI)  this->goal.z -= 2 * M_PI;

    // set the goal for the planner
    LIF_planner.setGoal(this->goal.x, this->goal.y, this->goal.z);

    bool reached = false;
    ros::Rate rate(1.0);
    while(ros::ok() && !reached) {
      // get the 'up-to-date' env info
      geometry_msgs::Point32 car_pos, obs_pos;
      {
        boost::shared_lock<boost::shared_mutex> lock(this->env_lock);
        car_pos = this->env.robot_pos;
        obs_pos = this->env.dynamic_obs_pos;
      }



      // did we get to the goal
      if (this->goal_reached(car_pos)) {
        reached = true;
        break;
      } else {
        ROS_INFO("[Planner] Distance remain: %lf.", this->distance2goal(car_pos));
      }
      LIF_planner.setStart(car_pos.x, car_pos.y, car_pos.z);
      // call the planner here
      ROS_INFO("[Planner] Start planner [From @(%lf, %lf, %lf) To @(%lf, %lf, %lf)]",
                car_pos.x, car_pos.y, car_pos.z, this->goal.x, this->goal.y, this->goal.z);
      LIF_planner.ARAstar(obs_pos.x, obs_pos.y, obs_pos.z);
      ROS_INFO("[Planner] Finish planning. ");
      // if the primitive generate by the planner is -1, remain the same position
      // if(LIF_planner.publish_primID != -1){
      //   ROS_INFO("[Planner] Calculated primitive is %d", LIF_planner.publish_primID);
      // 	this->current_position.x = LIF_planner.publish_x;
  	  //   this->current_position.y = LIF_planner.publish_y;
  	  //  	this->current_position.z = LIF_planner.publish_theta;
  	  //   this->next_primitive = LIF_planner.publish_primID;
      // 	this->forward();
      // } else {
      //   ROS_WARN("[Planner] Motion primitive given -1.");
      // }
      if (LIF_planner.prim_set.empty()) {
        ROS_WARN("[Planner] Trajectory empty!");
      } else {
        ROS_INFO("[Planner] Trajectory length: %ld.", LIF_planner.prim_set.size());
        for (int trj_index = 0; trj_index < LIF_planner.prim_set.size(); trj_index ++) {
          ROS_INFO("[Planner] -------------------------------------");
          ROS_INFO("[Planner] Executing primitive (%d out of %ld)...", trj_index, LIF_planner.prim_set.size());
          this->next_primitive = LIF_planner.prim_set[trj_index];
          this->forward();
          rate.sleep();
        }
      }

      break;
      // wait for the update to be ready
      rate.sleep();
    }

    if (!reached) {
      ROS_INFO("[Planner] ROS node shutdown, give up planning.");
      res.success = false;
      res.reason = "ROS shutdown, give up the planner.";
    } else {
      ROS_INFO("[Planner] Done.");
      res.success = true;
      res.reason = "Done.";
    }

    return true;
  }

  void CleanUp(void) {
    //Release memory of the map
    for(int i=0;i<this->y_size;i++){
      delete [] this->map[i];
    }
    delete [] this->map;
  }
};


//Run planner in main function and perform publisher and subscriber
int main(int argc, char **argv) {
  //Initialize ROS node
  ros::init(argc, argv, "Planner");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  ARAPlanner planner;

  ros::waitForShutdown();

  // clean up the resource
  planner.CleanUp();
  spinner.stop();

  return 0;
}
