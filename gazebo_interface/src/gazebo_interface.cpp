/*
 * this is the GAZEBO_INTERFACE where we can update
 * the position of our little roobt using the trajectory
 * designed by our planner
 */

#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Point32.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <actionlib/server/simple_action_server.h>
#include <gazebo_interface/executeAction.h>

#include <gazebo_interface/Environment.h>
#include <gazebo_interface/DyObsUpdate.h>
#include <gazebo_interface/Motion.h>

class GazeboUpdater {
 private:
  // ros node handler
  ros::NodeHandle nh, pnh;

  // publish to the gazebo state topic
  boost::thread gazebo_state_updater;
  ros::Publisher model_state;
  std::string model_name, reference_name;

  // update dynamic obstacle position using service
  ros::ServiceClient obs_pos_setter;

  // action server (update by series of waypoints)
  actionlib::SimpleActionServer<gazebo_interface::executeAction> trj_updater;
  std::string action_server_name;
  gazebo_interface::executeFeedback feedback;
  gazebo_interface::executeResult result;
  double execution_freq;

  // update by a single waypoint
  int numof_dirs, primitives_per_dir, waypoints_per_primitive, waypoint_dim;
  std::vector<std::vector<std::vector<float> > > primitives;
  ros::Subscriber pos_updater;
  boost::mutex car_update_action_lock;
  ros::Subscriber obs_pos_updater;

  // publish the environment
  double env_update_freq;
  ros::Publisher environment;
  boost::thread publishers_group;
  boost::shared_mutex cur_pos_update_lock;
  geometry_msgs::Point32 cur_pos;
  boost::shared_mutex obs_pos_update_lock;
  geometry_msgs::Point32 obs_pos;

 private:
  /* Function @ update_gazebo
   * this will keep updating the position
   * of our little car within the GAZEBO
   */
  void update_gazebo(void) {
    ROS_INFO("[update_gazebo] Hello there! I'll update our little car!");
    // set up frequency
    ros::Rate rate(10);
    // set up desired state
    gazebo_msgs::ModelState state;
    state.model_name = this->model_name;
    state.reference_frame = this->reference_name;

    geometry_msgs::Point32 next;
    while (ros::ok()) {
      {
        boost::shared_lock<boost::shared_mutex> lock(this->cur_pos_update_lock);
        next = this->cur_pos;
      }
      // set position
      state.pose.position.x = next.x;
      state.pose.position.y = next.y;
      state.pose.position.z = 0.05; // what does here mean?

      // set orientation
      tf::Quaternion rot = tf::createQuaternionFromRPY(0, 0, next.z);
      tf::quaternionTFToMsg(rot, state.pose.orientation);

      // update
      this->model_state.publish(state);

      rate.sleep();
    }

    ROS_INFO("[update_gazebo] Exit.");
    return;
  }

  /*
   * Function @ publishers
   * publish environment status in another thread
   */
  void publishers(void) {
    ROS_INFO("[publishers] Hi there! I'll publish the status of the environment!");
    ros::Rate rate(this->env_update_freq);
    unsigned int seq = 0;
    while (ros::ok()) {
      // publish the current dynamic environment
      gazebo_interface::Environment env;
      env.header.seq = seq ++;
      env.header.stamp = ros::Time::now();
      env.header.frame_id = "world";

      {
        // robot position
        boost::shared_lock<boost::shared_mutex> lock(this->cur_pos_update_lock);
        env.robot_pos = this->cur_pos;
      }
      {
        // dynamic obstacle position
        boost::shared_lock<boost::shared_mutex> lock(this->obs_pos_update_lock);
        env.dynamic_obs_pos = this->obs_pos;
      }

      // publish
      this->environment.publish(env);

      rate.sleep();
    }

    ROS_INFO("[publishers] Exit.");
    return;
  }

  /*
   * Function @ threads_start
   * this starts helper threads
   */
  void threads_start(void) {
    this->publishers_group = boost::thread(&GazeboUpdater::publishers, this);
    ROS_INFO("[gazebo_interface] publishers spawned.");
    this->gazebo_state_updater = boost::thread(&GazeboUpdater::update_gazebo, this);
    ROS_INFO("[gazebo_interface] state updater spawned.");
    return;
  }

  /*
   * Function @ get_primitives
   * this function fetches and interpolates
   * motion primitives from the parameter server
   */
  bool get_primitives(void) {
    // how many dirs? primitives per dir?
    if (!this->nh.getParam("/global/primitives/num_of_dirs", this->numof_dirs)) {
      ROS_ERROR("[gazebo_interface] # of primitives not specified. Assume to be 36?");
      this->numof_dirs = 36;
    }
    if (!this->nh.getParam("/global/primitives/primitives_per_dir", this->primitives_per_dir)) {
      ROS_ERROR("[gazebo_interface] # of primitives per direction not specified. Assume to be 8?");
      this->primitives_per_dir = 8;
    }
    if (!this->nh.getParam("/global/primitives/waypoints_per_primitive", this->waypoints_per_primitive)) {
      ROS_ERROR("[gazebo_interface] # of waypoints per primitive not specified. Assume to be 6?");
      this->waypoints_per_primitive = 6;
    }
    if (!this->nh.getParam("/global/primitives/wapoint_dimension", this->waypoint_dim)) {
      ROS_ERROR("[gazebo_interface] waypoint dimension not specified. Assume to be 3?");
      this->waypoint_dim = 3;
    }
    std::vector<float> primitives_array;
    if (!this->nh.getParam("/global/primitives/motion_primitives", primitives_array)) {
      ROS_FATAL("[gazebo_interface] primitives array not found! No motion primitives are used!!");
      return false;
    }

    // checking validness of this primitive array
    ROS_INFO("[gazebo_interface] Checking validness of primitives_array...");
    if (primitives_array.size() != (this->numof_dirs * this->primitives_per_dir * this->waypoints_per_primitive * this->waypoint_dim)) {
      ROS_FATAL("[gazebo_interface] primitives_array not valid. Size should be (%d x %d x %d x %d)", this->numof_dirs, this->primitives_per_dir, this->waypoints_per_primitive, this->waypoint_dim);
      return false;
    }
    ROS_INFO("[gazebo_interface] Done.");

    // got primitives, parse this huge array
    ROS_INFO("[gazebo_interface] Interpreting this HUGE primitive matrix...");
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
    ROS_INFO("[gazebo_interface] Done.");

    return true;
  }

  /*
   * Function @ animation
   * this functions takes the current position and a
   * motion primitive to generate a path representing
   * this motion primitive. NOTE: this function will
   * update the current position
   */
  bool animation(const unsigned int primitive, const geometry_msgs::Point32 &cur_pos, bool override = false) {
    // what direction are we facing?
    float absolute_rad = 0;
    this->cur_pos.z < 0? absolute_rad = this->cur_pos.z + 2 * M_PI : absolute_rad = this->cur_pos.z;
    ROS_INFO("[gazebo_interface] Current x position is %lf.", this->cur_pos.x);
    ROS_INFO("[gazebo_interface] Current y position is %lf.", this->cur_pos.y);
    ROS_INFO("[gazebo_interface] Absolute rad is %lf.", absolute_rad);
    int dir_index = (int)((absolute_rad * 180.0 / M_PI) / 10.0 + 0.5);
    // check index validness
    if (7 < primitive || primitive < 0) {
      ROS_ERROR("[gazebo_interface] Motion primitive out of range. [Given %d]", primitive);
      return false;
    }
    ROS_INFO("[gazebo_interface][animation] Decoded direction index: %d.", dir_index);
    std::vector<float> trajectory = this->primitives[dir_index][primitive];
    // for (std::vector<float>::iterator it = trajectory.begin(); it != trajectory.end(); ++ it) {
    //   ROS_INFO("%lf", *it);
    // }

    // start execution
    ros::Rate rate(this->execution_freq);
    for (int update_index = 1; update_index < this->waypoints_per_primitive; update_index ++) {
      {
        boost::upgrade_lock<boost::shared_mutex> lock(this->cur_pos_update_lock);
        boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(lock);
        this->cur_pos.x += (trajectory[update_index * this->waypoint_dim] - trajectory[(update_index - 1) * this->waypoint_dim]);
        this->cur_pos.y += (trajectory[update_index * this->waypoint_dim + 1] - trajectory[(update_index - 1) * this->waypoint_dim + 1]);
        this->cur_pos.z = trajectory[update_index * this->waypoint_dim + 2];
      }
      rate.sleep();
    }
    return true;
  }

 public:
  GazeboUpdater(std::string action_name) :
    nh("gazebo_interface"),
    pnh("~"),
    action_server_name(action_name),
    trj_updater(nh, action_name, boost::bind(&GazeboUpdater::executeCb, this, _1), false) {
    ROS_INFO("[gazebo_interface] Initializing gazebo interface...");
    // initialize this node
    if (!this->pnh.getParam("execution_frequency", this->execution_freq)) {
      ROS_WARN("[gazebo_interface] Action Execution Frequency not specified! Assume 1.0 HZ.");
      this->execution_freq = 1.0;
    }
    if (!this->pnh.getParam("env_update_frequency", this->env_update_freq)) {
      ROS_WARN("[gazebo_interface] Environment Update Frequency not specified! Assume 100 HZ");
      this->env_update_freq = 100.0;
    }
    std::string pos_topic_name, obs_pos_topic_name;
    if (!this->pnh.getParam("position_update_topic", pos_topic_name)) {
      ROS_WARN("[gazebo_interface] Which topic should I listen to for position update? Assume '/planner/position_update'.");
      pos_topic_name = "/planner/position_update";
    }
    if (!this->pnh.getParam("obs_pos_update_topic", obs_pos_topic_name)) {
      ROS_WARN("[gazebo_interface] Which topic should I listen to for dynamic obstacle position update? Assume '/obs_planner/position_update'.");
      obs_pos_topic_name = "/obs_planner/position_update";
    }
    if (!this->pnh.getParam("model_name", this->model_name)) {
      ROS_WARN("[gazebo_interface] Model name not specified! Assume our car to be 'audi_bot'?");
      this->model_name = "audi_bot";
    }
    float cur_x, cur_y, cur_yaw;
    if (!this->pnh.getParam("model_init_x", cur_x)) {
      ROS_WARN("[gazebo_interface] Model initial x_pos not specified! Assume to be 30.5(m)?");
      cur_x = 30.5;
    }
    if (!this->pnh.getParam("model_init_y", cur_y)) {
      ROS_WARN("[gazebo_interface] Model initial y_pos not specified! Assume to be 0.0(m)?");
      cur_y = 0.0;
    }
    if (!this->pnh.getParam("model_init_yaw", cur_yaw)) {
      ROS_WARN("[gazebo_interface] Model initial yaw not specified! Assume to be -90.0(deg)?");
      cur_yaw = -1.570796326794897;
    }
    this->cur_pos.x = cur_x; this->cur_pos.y = cur_y; this->cur_pos.z = cur_yaw;
    if (!this->pnh.getParam("model_reference_name", this->reference_name)) {
      ROS_WARN("[gazebo_interface] Reference name not specified! Assume it to be ''?");
      this->reference_name = "";
    }

    // initialize the primitives
    ROS_INFO("[gazebo_interface] Initializing motion primitives...");
    if (!this->get_primitives()) {
      ROS_FATAL("[gazebo_interface] Primitives not detected. Shut down the node!");
      ros::shutdown();
      return;
    }
    ROS_INFO("[gazebo_interface] Done.");

    // initizlize topics and services
    this->pos_updater = this->nh.subscribe(pos_topic_name, 5, &GazeboUpdater::pos_updateCb, this);
    this->model_state = this->nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

    this->environment = this->nh.advertise<gazebo_interface::Environment>("env", 1);

    this->obs_pos_updater = this->nh.subscribe(obs_pos_topic_name, 5, &GazeboUpdater::obs_pos_updateCb, this);
    this->obs_pos_setter = this->nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ROS_INFO("[gazebo_interface] Done.");

    // start publisher thread
    ROS_INFO("[gazebo_interface] Initializing helper threads...");
    this->threads_start();
    ROS_INFO("[gazebo_interface] Done.");

    // start the action server here
    ROS_INFO("[gazebo_interface] Starting the action server...");
    this->trj_updater.start();
    ROS_INFO("[gazebo_interface] Initialization done. Everyone is happy!");
  }

  ~GazeboUpdater() {
    // join the publishing thread
    ROS_INFO("[gazebo_interface] Joining helper threads...");
    this->publishers_group.join();
    this->gazebo_state_updater.join();
    ROS_INFO("[gazebo_interface] Done.");
  }

  void executeCb(const gazebo_interface::executeGoalConstPtr &goal) {
    // check the validness of the trajectory
    ROS_INFO("[gazebo_interface] Plan received.");
    ROS_INFO("[gazebo_interface] Checking validness of this plan...");
    std::vector<float> trajectory = goal->trajectory;
    int trj_dim = goal->dimension;
    if ((trajectory.size() % trj_dim) != 0) {
      ROS_ERROR("[gazebo_interface] Plan invalid, size doesn't match dimensionality!");
      this->result.success = false;
      this->result.percentage = 0.0;
      this->result.reason = "Plan invalid, size doesn't match dimensionality.";
      this->trj_updater.setAborted(this->result);
      return;
    }
    ROS_INFO("[gazebo_interface] Done.");

    // get execution lock
    ROS_INFO("[gazebo_interface] Retrieving execution lock...");
    this->car_update_action_lock.lock();
    ROS_INFO("[gazebo_interface] Execution lock retrieved. Start execution...");

    // start execution
    ros::Rate execution_rate(this->execution_freq);
    int trj_amount = (int)(trajectory.size() / trj_dim);
    int waypoint_index = 0;
    bool success = true;
    for (waypoint_index; waypoint_index < trj_amount; waypoint_index ++) {
      // are we preempted?
      if (this->trj_updater.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted...", this->action_server_name.c_str());
        // set the action state to preempted
        this->trj_updater.setPreempted();
        success = false;
        break;
      }

      // for each movemont
      float nextX = trajectory[trj_dim * waypoint_index];
      float nextY = trajectory[trj_dim * waypoint_index + 1];
      float nextTheta = trajectory[trj_dim * waypoint_index + 2];
      geometry_msgs::Point32 next;
      next.x = nextX; next.y = nextY; next.z = nextTheta;

      // update the gazebo
      {
        boost::upgrade_lock<boost::shared_mutex> lock(this->cur_pos_update_lock);
        boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(lock);
        this->cur_pos = next;
      }
      // give feedback to action client
      this->feedback.percentage = (waypoint_index + 1) * 1.0 / trj_amount;
      this->feedback.current_position = next;
      this->trj_updater.publishFeedback(this->feedback);

      // wait for the next move
      execution_rate.sleep();
    }

    // went through the trajectory
    ROS_INFO("[gazebo_interface] Trajectory went through!");
    if (success) {
      this->result.success = true;
      this->result.percentage = waypoint_index * 1.0 / trj_amount;
      this->result.reason = "Success.";
      this->trj_updater.setSucceeded(this->result);
    }
    ROS_INFO("[gazebo_interface] Releasing execution lock...");
    this->car_update_action_lock.unlock();
    ROS_INFO("[gazebo_interface] Done.");

    return;
  }

  void pos_updateCb(const gazebo_interface::Motion::ConstPtr &next) {
    ROS_INFO("[gazebo_interface] Next position received.");
    this->car_update_action_lock.lock();
    this->animation(next->primitive, next->cur_pos, false);
    this->car_update_action_lock.unlock();
    return;
  }

  void obs_pos_updateCb(const gazebo_interface::DyObsUpdate::ConstPtr &next) {
    // wait for the service to be available
    this->obs_pos_setter.waitForExistence();

    gazebo_msgs::SetModelState srv;

    gazebo_msgs::ModelState state;
    state.model_name = next->model_name;
    state.reference_frame = next->frame_name;
    state.pose.position.x = next->position.x;
    state.pose.position.y = next->position.y;
    state.pose.position.z = 0.05;
    tf::Quaternion rot = tf::createQuaternionFromRPY(0, 0, next->position.z);
    tf::quaternionTFToMsg(rot, state.pose.orientation);

    srv.request.model_state = state;

    // call the service
    if (!this->obs_pos_setter.call(srv)) {
      ROS_ERROR("[gazebo_interface] Failed to update position for dynamic obstacle.");
    } else {
      // update the environment
      {
        boost::upgrade_lock<boost::shared_mutex> lock(this->obs_pos_update_lock);
        boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(lock);
        this->obs_pos = next->position;
      }
    }

    return;
  }
};

int main(int argc, char **argv) {
  // initizlize this node
  ros::init(argc, argv, "gazebo_interface");
  ros::AsyncSpinner spinner(3);
  spinner.start();

  GazeboUpdater updater("model_updater");

  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
