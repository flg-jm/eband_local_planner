/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering  
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_collision_velocity_filter
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fhg.de
 *
 * Date of creation: February 2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include <eband_local_planner/collision_velocity_filter.h>

namespace eband_local_planner{
	
// Constructor
CollisionVelocityFilter::CollisionVelocityFilter(double virt_mass, costmap_2d::Costmap2DROS* costmap_ros)
{
  // create node handle
  nh_ = ros::NodeHandle("~");

  m_mutex = PTHREAD_MUTEX_INITIALIZER;
  
  // node handle to get footprint from parameter server
  std::string costmap_parameter_source;
  if(!nh_.hasParam("costmap_parameter_source")) ROS_WARN("Checking default source [/local_costmap_node/costmap] for costmap parameters");
  nh_.param("costmap_parameter_source",costmap_parameter_source, std::string("/local_costmap_node/costmap"));

  ros::NodeHandle local_costmap_nh_(costmap_parameter_source); 	

  // get footprint of the robot from costmap
  setFootprint(costmap_ros);
  
  // implementation of topics to publish (command for base and list of relevant obstacles)
  topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
  topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::GridCells>("relevant_obstacles", 1);

  // subscribe to the costmap to receive inflated cells
  obstacles_sub_ = nh_.subscribe<nav_msgs::GridCells>("/local_costmap_node/costmap/obstacles", 1, boost::bind(&CollisionVelocityFilter::obstaclesCB, this, _1));

  // read parameters from parameter server
  // parameters from costmap
  if(!local_costmap_nh_.hasParam(costmap_parameter_source+"/global_frame")) ROS_WARN("Used default parameter for global_frame [/base_link]");
  local_costmap_nh_.param(costmap_parameter_source+"/global_frame", global_frame_, std::string("/base_link"));

  if(!local_costmap_nh_.hasParam(costmap_parameter_source+"/robot_base_frame")) ROS_WARN("Used default parameter for robot_frame [/base_link]");
  local_costmap_nh_.param(costmap_parameter_source+"/robot_base_frame", robot_frame_, std::string("/base_link"));

  if(!nh_.hasParam("influence_radius")) ROS_WARN("Used default parameter for influence_radius [1.5 m]");
  nh_.param("influence_radius", influence_radius_, 1.0);
  closest_obstacle_dist_ = influence_radius_;
  closest_obstacle_angle_ = 0.0;

  // parameters for obstacle avoidence and velocity adjustment
  if(!nh_.hasParam("stop_threshold")) ROS_WARN("Used default parameter for stop_threshold [0.1 m]");
  nh_.param("stop_threshold", stop_threshold_, 0.1);

  if(!nh_.hasParam("obstacle_damping_dist")) ROS_WARN("Used default parameter for obstacle_damping_dist [5.0 m]");
  nh_.param("obstacle_damping_dist", obstacle_damping_dist_, 7.0);
  if(obstacle_damping_dist_ <= stop_threshold_) {
    obstacle_damping_dist_ = stop_threshold_ + 0.01; // set to stop_threshold_+0.01 to avoid divide by zero error
    ROS_WARN("obstacle_damping_dist <= stop_threshold -> robot will stop without decceleration!");
  }

  if(!nh_.hasParam("use_circumscribed_threshold")) ROS_WARN("Used default parameter for use_circumscribed_threshold_ [0.2 rad/s]");
  nh_.param("use_circumscribed_threshold", use_circumscribed_threshold_, 0.20);

  if(!nh_.hasParam("pot_ctrl_kv")) ROS_WARN("Used default parameter for pot_ctrl_kv [1.0]");
  nh_.param("pot_ctrl_kv", kv_, 3.5);

  if(!nh_.hasParam("pot_ctrl_kp")) ROS_WARN("Used default parameter for pot_ctrl_kp [2.0]");
  nh_.param("pot_ctrl_kp", kp_, 4.0);

  virt_mass_ = virt_mass;

  last_time_ = ros::Time::now().toSec();

}

// Destructor
CollisionVelocityFilter::~CollisionVelocityFilter(){}

// joystick_velocityCB reads twist command from joystick
void CollisionVelocityFilter::filterVelocity(bool car, double& v_max, geometry_msgs::Twist& desired_vel){
  //v_max_ = v_max;

  pthread_mutex_lock(&m_mutex);

  last_vel_ = desired_vel;

  pthread_mutex_unlock(&m_mutex);

  // check for relevant obstacles
  obstacleHandler();
  
  // if closest obstacle is within stop_threshold, then do not move
  if( closest_obstacle_dist_ < stop_threshold_ ) {
    last_vel_.linear.x = 0;
    last_vel_.linear.y = 0;
    last_vel_.angular.z = 0;
    v_max_ = 0;
  }else{
	// adjust velocity if we are about to run in an obstacle
	performControllerStep();
  }
  	//ROS_ERROR("Closestobstacle: (dist,angle) = (%f,%f)",closest_obstacle_dist_,closest_obstacle_angle_);
	//ROS_ERROR("Vmax = %f",v_max_);
  desired_vel = last_vel_;
  v_max = v_max_;
}

void CollisionVelocityFilter::setFootprint(costmap_2d::Costmap2DROS* costmap_ros){
  robot_footprint_ = costmap_ros->getRobotFootprint();
  if(robot_footprint_.size() > 4)
    ROS_WARN("You have set more than 4 points as robot_footprint, cob_collision_velocity_filter can deal only with rectangular footprints so far!");
 
  footprint_front_ = 0.0;
  footprint_rear_ = 0.0;
  footprint_left_ = 0.0;
  footprint_right_ = 0.0;

  for(unsigned int i=0; i<robot_footprint_.size(); i++) {
    if(robot_footprint_[i].x > footprint_front_) footprint_front_ = robot_footprint_[i].x;
    if(robot_footprint_[i].x < footprint_rear_) footprint_rear_ = robot_footprint_[i].x;
    if(robot_footprint_[i].y > footprint_left_) footprint_left_ = robot_footprint_[i].y;
    if(robot_footprint_[i].y < footprint_right_) footprint_right_ = robot_footprint_[i].y;
  }
  
}
    
// obstaclesCB reads obstacles from costmap
void CollisionVelocityFilter::obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles){
  pthread_mutex_lock(&m_mutex);

  if(obstacles->cells.size()!=0) costmap_received_ = true;
  last_costmap_received_ = * obstacles;

  if(stop_threshold_ < obstacles->cell_width / 2.0f || stop_threshold_ < obstacles->cell_height / 2.0f)
    ROS_WARN("You specified a stop_threshold that is smaller than resolution of received costmap!");

  pthread_mutex_unlock(&m_mutex);
}


// sets corrected velocity of joystick command
void CollisionVelocityFilter::performControllerStep() {

  double dt;
  double vx_max, vy_max;
  geometry_msgs::Twist cmd_vel;

  cmd_vel = last_vel_;
  dt = ros::Time::now().toSec() - last_time_;
  last_time_ = ros::Time::now().toSec();

  double vel_angle = atan2(cmd_vel.linear.y,cmd_vel.linear.x);
  vx_max = v_max_ * fabs(cos(vel_angle));
  if (vx_max > fabs(cmd_vel.linear.x)) vx_max = fabs(cmd_vel.linear.x);
  vy_max = v_max_ * fabs(sin(vel_angle));
  if (vy_max > fabs(cmd_vel.linear.y)) vy_max = fabs(cmd_vel.linear.y);

  //Slow down in any way while approximating an obstacle:
  /*if(closest_obstacle_dist_ < influence_radius_) {
    double vx_d, vy_d, vx_factor, vy_factor;
    double kv_obst=kv_, vx_max_obst=vx_max, vy_max_obst=vy_max;

    //implementation for linear decrease of v_max:
    double obstacle_linear_slope_x = vx_max / (obstacle_damping_dist_ - stop_threshold_);
    vx_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope_x;
    if(vx_max_obst > vx_max) vx_max_obst = vx_max;
    else if(vx_max_obst < 0.0f) vx_max_obst = 0.0f;

    double obstacle_linear_slope_y = vy_max / (obstacle_damping_dist_ - stop_threshold_);
    vy_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope_y;
    if(vy_max_obst > vy_max) vy_max_obst = vy_max;
    else if(vy_max_obst < 0.0f) vy_max_obst = 0.0f;

    //Translational movement
    //calculation of v factor to limit maxspeed
    double closest_obstacle_dist_x = closest_obstacle_dist_ * cos(closest_obstacle_angle_);
    double closest_obstacle_dist_y = closest_obstacle_dist_ * sin(closest_obstacle_angle_);
    vx_d = kp_/kv_obst * closest_obstacle_dist_x;
    vy_d = kp_/kv_obst * closest_obstacle_dist_y;
    vx_factor = vx_max_obst / sqrt(vy_d*vy_d + vx_d*vx_d);
    vy_factor = vy_max_obst / sqrt(vy_d*vy_d + vx_d*vx_d);
    if(vx_factor > 1.0) vx_factor = 1.0;
    if(vy_factor > 1.0) vy_factor = 1.0;

    //cmd_vel.linear.x += vx_factor * kp_/kv_obst * closest_obstacle_dist_x;
    //cmd_vel.linear.y += vy_factor * kp_/kv_obst * closest_obstacle_dist_y;

	cmd_vel.linear.x *= vx_factor;
    cmd_vel.linear.y *= vy_factor;
    cmd_vel.angular.z *= vy_factor;
    
    //cmd_vel.linear.x = last_vel_.linear.x + F_x / virt_mass_ * dt;
    //cmd_vel.linear.y = last_vel_.linear.y + F_y / virt_mass_ * dt;

  }*/
  double acc_max = 0.3;
  v_max_ = sqrt(2.0 * (closest_obstacle_dist_-stop_threshold_) * acc_max);
  if(closest_obstacle_dist_ < influence_radius_) {
	   double ang_pseudo_dist = cmd_vel.angular.z * sqrt(footprint_front_*footprint_front_+footprint_left_*footprint_left_);
	   double abs_cmd_vel = sqrt(cmd_vel.linear.x*cmd_vel.linear.x+cmd_vel.linear.y*cmd_vel.linear.y+ang_pseudo_dist*ang_pseudo_dist);
	   if (abs_cmd_vel > v_max_)
	   {
		   double scale = v_max_/abs_cmd_vel;
		   cmd_vel.linear.x *= scale;
		   cmd_vel.linear.y *= scale;
		   cmd_vel.angular.z *= scale;
	   }
  }
	
  pthread_mutex_lock(&m_mutex);
  last_vel_ = cmd_vel;
  pthread_mutex_unlock(&m_mutex);

  return;
}

void CollisionVelocityFilter::obstacleHandler() {
  pthread_mutex_lock(&m_mutex);
  if(!costmap_received_) {
    ROS_WARN("No costmap has been received by cob_collision_velocity_filter, the robot will drive without obstacle avoidance!");
    closest_obstacle_dist_ = influence_radius_;

    pthread_mutex_unlock(&m_mutex);
    return;
  }
  closest_obstacle_dist_ = influence_radius_;
  pthread_mutex_unlock(&m_mutex);

  double cur_distance_to_center, cur_distance_to_border;
  double obstacle_theta_robot, obstacle_delta_theta_robot, obstacle_dist_vel_dir;
  bool cur_obstacle_relevant;
  geometry_msgs::Point cur_obstacle_robot;
  geometry_msgs::Point zero_position;
  zero_position.x=0.0f;  
  zero_position.y=0.0f;
  zero_position.z=0.0f;
  bool use_circumscribed=true, use_tube=true;

    
  //Calculate corner angles in robot_frame:
  double corner_front_left, corner_rear_left, corner_rear_right, corner_front_right;
  corner_front_left = atan2(footprint_left_, footprint_front_);
  corner_rear_left = atan2(footprint_left_, footprint_rear_);
  corner_rear_right = atan2(footprint_right_, footprint_rear_);
  corner_front_right = atan2(footprint_right_, footprint_front_);

  //Decide, whether circumscribed or tube argument should be used for filtering:
  //if(fabs(last_vel_.linear.x) <= 0.005f && fabs(last_vel_.linear.y) <= 0.005f) {
    //use_tube = false;
    //disable tube filter at very slow velocities
  //}
  if(!use_tube) {
    if( fabs(last_vel_.angular.z) <= 0.01f) {
      use_circumscribed = false;
    } //when tube filter inactive, start circumscribed filter at very low rot-velocities
  } else {
    if( fabs(last_vel_.angular.z) <= use_circumscribed_threshold_) {
      use_circumscribed = false;
    } //when tube filter running, disable circum-filter in a wider range of rot-velocities
  }

  //Calculation of tube in driving-dir considered for obstacle avoidence
  double velocity_angle=0.0f, velocity_ortho_angle;
  double corner_angle, delta_corner_angle;
  double ortho_corner_dist;
  double tube_left_border = 0.0f, tube_right_border = 0.0f;
  double tube_left_origin = 0.0f, tube_right_origin = 0.0f;
  double corner_dist, circumscribed_radius = 0.0f;

  for(unsigned i = 0; i<robot_footprint_.size(); i++) {
    corner_dist = sqrt(robot_footprint_[i].x*robot_footprint_[i].x + robot_footprint_[i].y*robot_footprint_[i].y);
    if(corner_dist > circumscribed_radius) circumscribed_radius = corner_dist;
  }

  if(use_tube) {
    //use commanded vel-value for vel-vector direction.. ?
    velocity_angle = atan2(last_vel_.linear.y, last_vel_.linear.x);
    velocity_ortho_angle = velocity_angle + M_PI / 2.0f;

    for(unsigned i = 0; i<robot_footprint_.size(); i++) {
      corner_angle = atan2(robot_footprint_[i].y, robot_footprint_[i].x);
      delta_corner_angle = velocity_ortho_angle - corner_angle;
      corner_dist = sqrt(robot_footprint_[i].x*robot_footprint_[i].x + robot_footprint_[i].y*robot_footprint_[i].y);
      ortho_corner_dist = cos(delta_corner_angle) * corner_dist;

      if(ortho_corner_dist < tube_right_border) {
        tube_right_border = ortho_corner_dist;
        tube_right_origin = sin(delta_corner_angle) * corner_dist;
      } else if(ortho_corner_dist > tube_left_border) {
        tube_left_border = ortho_corner_dist;
        tube_left_origin = sin(delta_corner_angle) * corner_dist;
      }
    }
  }

  //find relevant obstacles
  pthread_mutex_lock(&m_mutex);
  relevant_obstacles_.header = last_costmap_received_.header;
  relevant_obstacles_.cell_width = last_costmap_received_.cell_width;
  relevant_obstacles_.cell_height = last_costmap_received_.cell_height;
  relevant_obstacles_.cells.clear();

  for(unsigned int i = 0; i < last_costmap_received_.cells.size(); i++) {
    cur_obstacle_relevant = false;
    cur_distance_to_center = getDistance2d(zero_position, last_costmap_received_.cells[i]);
    //check whether current obstacle lies inside the circumscribed_radius of the robot -> prevent collisions while rotating
    if(use_circumscribed && cur_distance_to_center <= circumscribed_radius) {
      cur_obstacle_robot = last_costmap_received_.cells[i];

      if( obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y) ) {
        cur_obstacle_relevant = true;
        relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
        obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
      }

      //for each obstacle, now check whether it lies in the tube or not:
    } else if(use_tube && cur_distance_to_center < influence_radius_) {
      cur_obstacle_robot = last_costmap_received_.cells[i];

      if( obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y) ) {
        obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
        obstacle_delta_theta_robot = obstacle_theta_robot - velocity_angle;
        obstacle_dist_vel_dir = sin(obstacle_delta_theta_robot) * cur_distance_to_center;

        if(obstacle_dist_vel_dir <= tube_left_border && obstacle_dist_vel_dir >= tube_right_border) {
          //found obstacle that lies inside of observation tube

          if( sign(obstacle_dist_vel_dir) >= 0) { 
            if(cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_left_origin) {
              //relevant obstacle in tube found
              cur_obstacle_relevant = true;
              relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
            }
          } else { // obstacle in right part of tube
            if(cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_right_origin) {
              //relevant obstacle in tube found
              cur_obstacle_relevant = true;
              relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
            }
          }
        }
      }
    }
    
    if(cur_obstacle_relevant) { //now calculate distance of current, relevant obstacle to robot
      if(obstacle_theta_robot >= corner_front_right && obstacle_theta_robot < corner_front_left) {
        //obstacle in front:
        cur_distance_to_border = cur_distance_to_center - fabs(footprint_front_) / fabs(cos(obstacle_theta_robot));
      } else if(obstacle_theta_robot >= corner_front_left && obstacle_theta_robot < corner_rear_left) {
        //obstacle left:
        cur_distance_to_border = cur_distance_to_center - fabs(footprint_left_) / fabs(sin(obstacle_theta_robot));
      } else if(obstacle_theta_robot >= corner_rear_left || obstacle_theta_robot < corner_rear_right) {
        //obstacle in rear:
        cur_distance_to_border = cur_distance_to_center - fabs(footprint_rear_) / fabs(cos(obstacle_theta_robot));
      } else {
        //obstacle right:
        cur_distance_to_border = cur_distance_to_center - fabs(footprint_right_) / fabs(sin(obstacle_theta_robot));
      }

      if(cur_distance_to_border < closest_obstacle_dist_) {
        closest_obstacle_dist_ = cur_distance_to_border;
        closest_obstacle_angle_ = obstacle_theta_robot;
      }
    }	
  }
  pthread_mutex_unlock(&m_mutex);
}


double CollisionVelocityFilter::getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double CollisionVelocityFilter::sign(double x) {
  if(x >= 0.0f) return 1.0f;
  else return -1.0f;
}

bool CollisionVelocityFilter::obstacleValid(double x_obstacle, double y_obstacle) {
  if(x_obstacle<footprint_front_ && x_obstacle>footprint_rear_ && y_obstacle>footprint_right_ && y_obstacle<footprint_left_) {
    ROS_WARN("Found an obstacle inside robot_footprint: Skip!");
    return false;
  }

  return true;
}

}
