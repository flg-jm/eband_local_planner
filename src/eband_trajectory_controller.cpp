/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Christian Connette
*********************************************************************/

#include <eband_local_planner/eband_trajectory_controller.h>
#include <tf/transform_datatypes.h>


namespace eband_local_planner{

using std::min;
using std::max;


EBandTrajectoryCtrl::EBandTrajectoryCtrl() : costmap_ros_(NULL), initialized_(false), band_set_(false), visualization_(false) {}


EBandTrajectoryCtrl::EBandTrajectoryCtrl(std::string name, costmap_2d::Costmap2DROS* costmap_ros, double center_ax_dist)
  : costmap_ros_(NULL), initialized_(false), band_set_(false), visualization_(false)
{
	// initialize planner
	initialize(name, costmap_ros, center_ax_dist);

  // Initialize pid object (note we'll be further clamping its output)
  // TODO See #1, #2
  pid_.initPid(1, 0, 0, 10, -10);
}


EBandTrajectoryCtrl::~EBandTrajectoryCtrl() {}

void EBandTrajectoryCtrl::configure_callback(eband_local_planner::EBandLocalPlannerConfig &config, uint32_t level) 
{
	ROS_INFO_STREAM("Max Vel: " << config.max_vel_lin);
	max_vel_lin_ = config.max_vel_lin;
	max_vel_th_ = config.max_vel_th;

	tolerance_trans_ = config.xy_goal_tolerance;
	tolerance_rot_ = config.yaw_goal_tolerance;
	tolerance_timeout_ = config.tolerance_timeout;

	k_p_ = config.k_prop;
	k_nu_ = config.k_damp;

	ctrl_freq_ = config.Ctrl_Rate;

	acc_max_ = config.max_acceleration;
	virt_mass_  = config.virtual_mass;

	acc_max_trans_ = config.max_translational_acceleration;
	acc_max_rot_ = config.max_rotational_acceleration;

	rotation_correction_threshold_ = config.rotation_correction_threshold;
	
}


void EBandTrajectoryCtrl::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros, double center_ax_dist)
{

	// check if trajectory controller is already initialized
	if(!initialized_)
	{
		// create Node Handle with name of plugin (as used in move_base for loading)
		ros::NodeHandle node_private("~/" + name);

		dyn_server_ = new dynamic_reconfigure::Server<EBandLocalPlannerConfig>(node_private);
		dynamic_reconfigure::Server<eband_local_planner::EBandLocalPlannerConfig>::CallbackType dyn_callback_ = boost::bind(&EBandTrajectoryCtrl::configure_callback, this, _1, _2);
  		dyn_server_->setCallback(dyn_callback_);
		
		// read parameters from parameter server
		//node_private.param("max_vel_lin", max_vel_lin_, 0.75);
		//node_private.param("max_vel_th", max_vel_th_, 1.0);


		// requirements for ackermann cinematics
		center_ax_dist_ = center_ax_dist;
		node_private.param("max_steering_angle", max_steering_angle_, 0.7);
		turning_radius_ = 2*center_ax_dist_/tan(max_steering_angle_);

	    node_private.param("angle_correction", angle_correction_, false);
	    node_private.param("forward_driving_flag", forward_, 1.0);
	    node_private.param("y_correction", y_correction_, false);
	    	    
		node_private.param("xy_goal_tolerance", tolerance_trans_, 0.02);
		node_private.param("yaw_goal_tolerance", tolerance_rot_, 0.04);
		node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

		/*node_private.param("k_prop", k_p_, 4.0);
		node_private.param("k_damp", k_nu_, 3.5);

		node_private.param("Ctrl_Rate", ctrl_freq_, 10.0); // TODO retrieve this from move base parameters

		node_private.param("max_acceleration", acc_max_, 0.5);
		node_private.param("virtual_mass", virt_mass_, 0.75);

		node_private.param("max_translational_acceleration", acc_max_trans_, 0.5);
		node_private.param("max_rotational_acceleration", acc_max_rot_, 1.5);

    node_private.param("rotation_correction_threshold", rotation_correction_threshold_, 0.5);

    // diffferential drive parameters
    node_private.param("differential_drive", differential_drive_hack_, true);*/

		// copy adress of costmap and Transform Listener (handed over from move_base)
		costmap_ros_ = costmap_ros;

		//start-/stop-smoothing parameters
		node_private.param("smoothing_enabled", smoothing_enabled_, true);
		node_private.param("start_smooth_iter", start_smoothing_border_, 10);
		node_private.param("stop_smoothing_dist_to_goal", stop_smoothing_dist_, 0.2);

		start_position_counter_ = 0;

		// Initialize the collision velocity filter
		cvf_ = boost::shared_ptr<CollisionVelocityFilter>(new CollisionVelocityFilter(costmap_ros, center_ax_dist));
				
		// tolerance should depend on turning radius
			tolerance_trans_ *= (1+turning_radius_);
			degrees_ = 0;
			factor_ = 1.1;

		// init velocity for interpolation
		last_vel_.linear.x = 0.0;
		last_vel_.linear.y = 0.0;
		last_vel_.linear.z = 0.0;
		last_vel_.angular.x = 0.0;
		last_vel_.angular.y = 0.0;
		last_vel_.angular.z = 0.0;

		// set the general reference frame to that in which the band is given
		geometry_msgs::Pose2D tmp_pose2D;
		tmp_pose2D.x = 0.0;
		tmp_pose2D.y = 0.0;
		tmp_pose2D.theta = 0.0;
		Pose2DToPose(ref_frame_band_, tmp_pose2D);

		// set initialized flag
		initialized_ = true;
	}
	else
	{
		ROS_WARN("This planner has already been initialized, doing nothing.");
	}
}


void EBandTrajectoryCtrl::setVisualization(boost::shared_ptr<EBandVisualization> target_visual)
{
	target_visual_ = target_visual;

	visualization_ = true;
}

bool EBandTrajectoryCtrl::setBand(const std::vector<Bubble>& elastic_band)
{
	elastic_band_ = elastic_band;
	band_set_ = true;
	return true;
}


bool EBandTrajectoryCtrl::setOdometry(const nav_msgs::Odometry& odometry)
{
	odom_vel_.linear.x = odometry.twist.twist.linear.x;
	odom_vel_.linear.y = odometry.twist.twist.linear.y;
	odom_vel_.linear.z = 0.0;
	odom_vel_.angular.x = 0.0;
	odom_vel_.angular.y = 0.0;
	odom_vel_.angular.z = odometry.twist.twist.angular.z;

	return true;
}

// Return the angular difference between the direction we're pointing
// and the direction we want to move in
double angularDiff (const geometry_msgs::Twist& heading,
                    const geometry_msgs::Pose& pose)
{
  const double pi = 3.14159265;
  const double t1 = atan2(heading.linear.y, heading.linear.x);
  const double t2 = tf::getYaw(pose.orientation);
  const double d = t1-t2;

  if (fabs(d)<pi)
    return d;
  else if (d<0)
    return d+2*pi;
  else
    return d-2*pi;
}


bool EBandTrajectoryCtrl::getTwist(geometry_msgs::Twist& twist_cmd, bool& goal_reached)
{
  goal_reached = false;

	// init twist cmd to be handed back to caller
	geometry_msgs::Twist robot_cmd, bubble_diff, control_deviation;
	robot_cmd.linear.x = 0.0;
	robot_cmd.linear.y = 0.0;
	robot_cmd.linear.z = 0.0;
	robot_cmd.angular.x = 0.0;
	robot_cmd.angular.y = 0.0;
	robot_cmd.angular.z = 0.0;

	// make sure command vector is clean
	twist_cmd = robot_cmd;
	control_deviation = robot_cmd;

	// check if plugin initialized
	if(!initialized_)
	{
		ROS_ERROR("Requesting feedforward command from not initialized planner. Please call initialize() before using this planner");
		return false;
	}

	// check if there is a plan at all (minimum 1 frame in this case, as robot + goal = plan)
	if( (!band_set_) || (elastic_band_.size() < 2) )
	{
		ROS_WARN("Requesting feedforward command from empty band.");
		return false;
	}
	
	// calc intersection of bubble-radius with sequence of vector connecting the bubbles

	// get distance to target from bubble-expansion
	double scaled_radius = 0.7 * elastic_band_.at(0).expansion;

	// get difference and distance between bubbles in odometry frame
	double bubble_distance, ang_pseudo_dist;
	bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(0).center.pose,
												elastic_band_.at(1).center.pose,
													ref_frame_band_);
	ang_pseudo_dist = bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();
	bubble_distance = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) + (bubble_diff.linear.y * bubble_diff.linear.y) +
						(ang_pseudo_dist * ang_pseudo_dist) );

	if(visualization_)
	{
		target_visual_->publishBubble("ctrl_target", 1, target_visual_->blue, elastic_band_.at(0));
		target_visual_->publishBubble("ctrl_target", 2, target_visual_->blue, elastic_band_.at(1));
	}

	// by default our control deviation is the difference between the bubble centers
	//double abs_ctrl_dev;
	control_deviation = bubble_diff;


	// yet depending on the expansion of our bubble we might want to adapt this point
	if(scaled_radius < bubble_distance)
	{
		// triviale case - simply scale bubble_diff
		double scale_difference = scaled_radius / bubble_distance;
		bubble_diff.linear.x *= scale_difference;
		bubble_diff.linear.y *= scale_difference;
		bubble_diff.angular.z *= scale_difference;
		// set controls
		control_deviation = bubble_diff;
	}
	
	// if scaled_radius = bubble_distance -- we have nothing to do at all

	if(scaled_radius > bubble_distance)
	{
		// o.k. now we have to do a little bit more -> check next but one bubble
		if(elastic_band_.size() > 2)
		{
			// get difference between next and next but one bubble
			double next_bubble_distance;
			geometry_msgs::Twist next_bubble_diff;
			next_bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(1).center.pose,
															elastic_band_.at(2).center.pose,
																ref_frame_band_);
			ang_pseudo_dist = next_bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();
			next_bubble_distance = sqrt( (next_bubble_diff.linear.x * next_bubble_diff.linear.x) +
											(next_bubble_diff.linear.y * next_bubble_diff.linear.y) +
												(ang_pseudo_dist * ang_pseudo_dist) );

			if(scaled_radius > (bubble_distance + next_bubble_distance) )
			{
				// we should normally not end up here - but just to be sure
				control_deviation.linear.x = bubble_diff.linear.x + next_bubble_diff.linear.x;
				control_deviation.linear.y = bubble_diff.linear.y + next_bubble_diff.linear.y;
				control_deviation.angular.z = bubble_diff.angular.z + next_bubble_diff.angular.z;
				// done
				if(visualization_)
					target_visual_->publishBubble("ctrl_target", 3, target_visual_->red, elastic_band_.at(2));
			}
			else
			{
				if(visualization_)
					target_visual_->publishBubble("ctrl_target", 3, target_visual_->red, elastic_band_.at(2));

				// we want to calculate intersection point of bubble ...
				// ... and vector connecting the following bubbles
				double b_distance, cosine_at_bub;
				double vec_prod, norm_vec1, norm_vec2;
				double ang_pseudo_dist1, ang_pseudo_dist2;

				// get distance between next bubble center and intersection point
				ang_pseudo_dist1 = bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();
				ang_pseudo_dist2 = next_bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();
				// careful! - we need this sign because of the direction of the vectors and the definition of the vector-product
				vec_prod = - ( (bubble_diff.linear.x * next_bubble_diff.linear.x) +
								(bubble_diff.linear.y * next_bubble_diff.linear.y) +
									(ang_pseudo_dist1 * ang_pseudo_dist2) );

				norm_vec1 = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) +
								(bubble_diff.linear.y * bubble_diff.linear.y) +
									(ang_pseudo_dist1 * ang_pseudo_dist1) );

				norm_vec2 = sqrt( (next_bubble_diff.linear.x * next_bubble_diff.linear.x) +
								(next_bubble_diff.linear.y * next_bubble_diff.linear.y) +
									(ang_pseudo_dist2 * ang_pseudo_dist2) );

				// cosine-rule
				cosine_at_bub = vec_prod / norm_vec1 / norm_vec2;
				// consider the next but one bubble in the omnidirectional case and in the carlike case if there is no turnaround needed
				if (cosine_at_bub < 0.0)
				{
					b_distance = bubble_distance * cosine_at_bub + sqrt( scaled_radius*scaled_radius -
								bubble_distance*bubble_distance * (1.0 - cosine_at_bub*cosine_at_bub) );
								
					// get difference vector from next_bubble to intersection point
					double scale_next_difference = b_distance / next_bubble_distance;
					next_bubble_diff.linear.x *= scale_next_difference;
					next_bubble_diff.linear.y *= scale_next_difference;
					next_bubble_diff.angular.z *= scale_next_difference;
	
					// and finally get the control deviation
					control_deviation.linear.x = bubble_diff.linear.x + next_bubble_diff.linear.x;
					control_deviation.linear.y = bubble_diff.linear.y + next_bubble_diff.linear.y;
					control_deviation.angular.z = bubble_diff.angular.z + next_bubble_diff.angular.z;
				}
			}
		}
	}



	if(visualization_)
	{
		// compose bubble from ctrl-target
		geometry_msgs::Pose2D tmp_bubble_2d, curr_bubble_2d;
		geometry_msgs::Pose tmp_pose;
		// init bubble for visualization
		Bubble new_bubble = elastic_band_.at(0);
		PoseToPose2D(elastic_band_.at(0).center.pose, curr_bubble_2d);
		tmp_bubble_2d.x = curr_bubble_2d.x + control_deviation.linear.x;
		tmp_bubble_2d.y = curr_bubble_2d.y + control_deviation.linear.y;
		tmp_bubble_2d.theta = curr_bubble_2d.theta + control_deviation.angular.z;
		Pose2DToPose(tmp_pose, tmp_bubble_2d);
		new_bubble.center.pose = tmp_pose;
		new_bubble.expansion = 0.1; // just draw a small bubble
		target_visual_->publishBubble("ctrl_target", 0, target_visual_->red, new_bubble);
	}


    const geometry_msgs::Point& goal = (--elastic_band_.end())->center.pose.position;
    const double dx = elastic_band_.at(0).center.pose.position.x - goal.x;
    const double dy = elastic_band_.at(0).center.pose.position.y - goal.y;
    const double dist_to_goal = sqrt(dx*dx + dy*dy);
    
    // Assuming we're far enough from the final goal, make sure to rotate so
    // we're facing the right way 
    /*if (dist_to_goal > rotation_correction_threshold_)
    {
    
      const double angular_diff = angularDiff(control_deviation, elastic_band_.at(0).center.pose);
      // TODO See Issue #1, #2
      const double vel = pid_.updatePid(-angular_diff, ros::Duration(1/ctrl_freq_));
      //const double vel = 0;
      const double mult = fabs(vel) > max_vel_th_ ? max_vel_th_/fabs(vel) : 1.0;
      control_deviation.angular.z = vel*mult;
      const double abs_vel = fabs(control_deviation.angular.z);

      ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                "Angular diff is %.2f and desired angular "
                                "vel is %.2f.  Initial translation velocity "
                                "is %.2f, %.2f", angular_diff,
                                control_deviation.angular.z,
                                control_deviation.linear.x,
                                control_deviation.linear.y);
      const double trans_mult = max(0.01, 1.0 - abs_vel/max_vel_th_); // There are some weird tf errors if I let it be 0
      control_deviation.linear.x *= trans_mult;
      control_deviation.linear.y *= trans_mult;
      ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                "Translation multiplier is %.2f and scaled "
                                "translational velocity is %.2f, %.2f",
                                trans_mult, control_deviation.linear.x,
                                control_deviation.linear.y);
    }
    else
      ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                "Not applying angle correction because "
                                "distance to goal is %.2f", dist_to_goal);
    */                               
        

	
	// now convert into robot-body frame
	control_deviation = transformTwistFromFrame1ToFrame2(control_deviation, ref_frame_band_, elastic_band_.at(0).center.pose);
		
	getTwistAckermann(control_deviation, dist_to_goal);
	
	
	// now the actual control procedure start (using attractive Potentials)
	geometry_msgs::Twist desired_velocity, currbub_maxvel_dir;
	double desvel_abs, desvel_abs_trans, currbub_maxvel_abs;
	double scale_des_vel;
	desired_velocity = robot_cmd;
	currbub_maxvel_dir = robot_cmd;
	
	// calculate "equilibrium velocity" (Khatib86 - Realtime Obstacle Avoidance)
	desired_velocity.linear.x = k_p_/k_nu_ * control_deviation.linear.x;
	desired_velocity.linear.y = k_p_/k_nu_ * control_deviation.linear.y;
	desired_velocity.angular.z = k_p_/k_nu_ * control_deviation.angular.z;
	
	
	// get max vel for current bubble
	int curr_bub_num = 0;
	currbub_maxvel_abs = getBubbleTargetVel(curr_bub_num, elastic_band_, currbub_maxvel_dir);
	
	// if neccessarry scale desired vel to stay lower than currbub_maxvel_abs
	ang_pseudo_dist = desired_velocity.angular.z * costmap_ros_->getCircumscribedRadius();
	desvel_abs = sqrt( (desired_velocity.linear.x * desired_velocity.linear.x) +
							(desired_velocity.linear.y * desired_velocity.linear.y) +
								(ang_pseudo_dist * ang_pseudo_dist) );
	if(desvel_abs > currbub_maxvel_abs)
	{
		scale_des_vel = currbub_maxvel_abs / desvel_abs;
		desired_velocity.linear.x *= scale_des_vel;
		desired_velocity.linear.y *= scale_des_vel;
		desired_velocity.angular.z *= scale_des_vel;
	}

	// make sure to stay within velocity bounds for the robot
	desvel_abs_trans = sqrt( (desired_velocity.linear.x * desired_velocity.linear.x) + (desired_velocity.linear.y * desired_velocity.linear.y) );
	// for translation
	if(desvel_abs_trans > max_vel_lin_)
	{
		scale_des_vel = max_vel_lin_ / desvel_abs_trans;
		desired_velocity.linear.x *= scale_des_vel;
		desired_velocity.linear.y *= scale_des_vel;
		// to make sure we are staying inside the bubble also scale rotation
		desired_velocity.angular.z *= scale_des_vel;
	}

	// for rotation
	if(fabs(desired_velocity.angular.z) > max_vel_th_)
	{
		scale_des_vel = max_vel_th_ / fabs(desired_velocity.angular.z);
		desired_velocity.angular.z *= scale_des_vel;
		// to make sure we are staying inside the bubble also scale translation
		desired_velocity.linear.x *= scale_des_vel;
		desired_velocity.linear.y *= scale_des_vel;
	}
	
	// calculate resulting force (accel. resp.) (Khatib86 - Realtime Obstacle Avoidance)
	geometry_msgs::Twist acc_desired;
	acc_desired = robot_cmd;
	acc_desired.linear.x = (1.0/virt_mass_) * k_nu_ * (desired_velocity.linear.x - last_vel_.linear.x);
	acc_desired.linear.y = (1.0/virt_mass_) * k_nu_ * (desired_velocity.linear.y - last_vel_.linear.y);
	acc_desired.angular.z = (1.0/virt_mass_) * k_nu_ * (desired_velocity.angular.z - last_vel_.angular.z);

	// constrain acceleration
	double scale_acc;
	double abs_acc_trans = sqrt( (acc_desired.linear.x*acc_desired.linear.x) + (acc_desired.linear.y*acc_desired.linear.y) );
	if(abs_acc_trans > acc_max_trans_)
	{
		scale_acc = acc_max_trans_ / abs_acc_trans;
		acc_desired.linear.x *= scale_acc;
		acc_desired.linear.y *= scale_acc;
		// again - keep relations - stay in bubble
		acc_desired.angular.z *= scale_acc;
	}

	if(fabs(acc_desired.angular.z) > acc_max_rot_)
	{
		scale_acc = fabs(acc_desired.angular.z) / acc_max_rot_;
		acc_desired.angular.z *= scale_acc;
		// again - keep relations - stay in bubble
		acc_desired.linear.x *= scale_acc;
		acc_desired.linear.y *= scale_acc;
	}

	// and get velocity-cmds by integrating them over one time-step
	last_vel_.linear.x = last_vel_.linear.x + acc_desired.linear.x / ctrl_freq_;
	last_vel_.linear.y = last_vel_.linear.y + acc_desired.linear.y / ctrl_freq_;
	last_vel_.angular.z = last_vel_.angular.z + acc_desired.angular.z / ctrl_freq_;


	// we are almost done now take into account stick-slip and similar nasty things

	// last checks - limit current twist cmd (upper and lower bounds)
	last_vel_ = limitTwist(last_vel_);

	// smooth the velocty at the start/end of trajectory
	if(smoothing_enabled_ && !y_correction_)
	{
		// smoothing at start of trajectory
		if(start_position_counter_ <= start_smoothing_border_)
		{
			ROS_DEBUG_STREAM("Smoothing start from: " << last_vel_);
			last_vel_.linear.x *= (double) start_position_counter_/start_smoothing_border_;
			last_vel_.linear.y *= (double) start_position_counter_/start_smoothing_border_;
			last_vel_.angular.z *= (double) start_position_counter_/start_smoothing_border_;
			ROS_DEBUG_STREAM("Smoothing start to: " << last_vel_);
			ROS_DEBUG_STREAM("Start Smoothing factor: " << (double) start_position_counter_/start_smoothing_border_);

			start_position_counter_++;
		}
		// smoothing at end of trajectory
		ROS_DEBUG("Bubble_Dist: %f",dist_to_goal);
		if(dist_to_goal < stop_smoothing_dist_)
		{
			ROS_DEBUG_STREAM("Smoothing stop from: " << last_vel_);
			// Calculate orientation difference to goal orientation (not captured in bubble_diff)
        	double robot_yaw = tf::getYaw(elastic_band_.at(0).center.pose.orientation);
        	double goal_yaw = tf::getYaw(elastic_band_.at((int)elastic_band_.size() - 1).center.pose.orientation);
        	float orientation_diff = angles::normalize_angle(goal_yaw - robot_yaw);		
			ROS_DEBUG_STREAM("trans dist_to_goal: " << dist_to_goal);
			ROS_DEBUG_STREAM("Stop Trans-Smoothing factor: " << dist_to_goal/stop_smoothing_dist_);
			if(dist_to_goal+fabs(orientation_diff) < stop_smoothing_dist_)
			{
				// linear smoothting
				last_vel_.linear.x *= (dist_to_goal+fabs(orientation_diff))/stop_smoothing_dist_;
				last_vel_.linear.y *= (dist_to_goal+fabs(orientation_diff))/stop_smoothing_dist_;
				// angular smoothting only if the orientation diff is small enough
				last_vel_.angular.z *= (dist_to_goal+fabs(orientation_diff))/stop_smoothing_dist_;
				ROS_DEBUG_STREAM("rot dist_to_goal: " << orientation_diff);
				ROS_DEBUG_STREAM("Stop Rot-Smoothing factor: " << fabs(orientation_diff)/stop_smoothing_dist_);
			}

			ROS_DEBUG_STREAM("Smoothing stop to: " << last_vel_);
			if((dist_to_goal < tolerance_trans_) && (fabs(orientation_diff) < tolerance_rot_))
			{
				// set goal_reached to make sure, the robot stops at the end of trajectory
				goal_reached = true;
			}
		}
	}

	// finally set robot_cmd (to non-zero value)
	robot_cmd = last_vel_;


	// check whether we reached the end of the band
	int curr_target_bubble = 1;
	while(fabs(bubble_diff.linear.x) <= tolerance_trans_ &&
			fabs(bubble_diff.linear.y) <= 2*tolerance_trans_ &&
			fabs(bubble_diff.angular.z) <= tolerance_rot_)
	{
		if(curr_target_bubble < ((int) elastic_band_.size()) - 1)
		{
                  curr_target_bubble++;
                  // transform next target bubble into robot-body frame
                  // and get difference to robot bubble
                  bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(0).center.pose, elastic_band_.at(curr_target_bubble).center.pose,
                                                            ref_frame_band_);
		}
		else
		{
                  ROS_DEBUG_THROTTLE_NAMED (1.0, "controller_state",
                                            "Goal reached with distance %.2f, %.2f, %.2f"
                                            "; sending zero velocity",
                                            bubble_diff.linear.x, bubble_diff.linear.y,
                                            bubble_diff.angular.z);
                  // goal position reached
                  robot_cmd.linear.x = 0.0;
                  robot_cmd.linear.y = 0.0;
                  robot_cmd.angular.z = 0.0;
                  // reset velocity
                  last_vel_.linear.x = 0.0;
                  last_vel_.linear.y = 0.0;
                  last_vel_.angular.z = 0.0;
                  start_position_counter_ = 0;
                  goal_reached = true;
                  break;
		}
	}
	
	
	// FILTER
	//ROS_WARN("Twist vor Filter: (%f, %f, %f)",robot_cmd.linear.x,robot_cmd.linear.y,robot_cmd.angular.z);
	cvf_->filterVelocity(robot_cmd);
	//ROS_WARN("Twist nach Filter: (%f, %f, %f)",robot_cmd.linear.x,robot_cmd.linear.y,robot_cmd.angular.z);		
		
	// if Ackermann cinematics is desired check again the twist to hold the constraints
	if (fabs(robot_cmd.linear.x) < fabs(robot_cmd.angular.z)*turning_radius_)
	{
		// keep minimal turning radius
		robot_cmd.angular.z *= fabs(robot_cmd.linear.x/robot_cmd.angular.z)/turning_radius_;
	}
	// keep relation between angular velocity and translational velocity in y direction
	robot_cmd.linear.y = robot_cmd.angular.z*center_ax_dist_;
	

	twist_cmd = robot_cmd;
	last_vel_ = robot_cmd;
	
	// in the case we are still in the correcting mode. turn it off when the goal is reached
	if (goal_reached)
	{
		y_correction_ = false;
		angle_correction_ = false;
	}
	
	return true;
}

bool EBandTrajectoryCtrl::getTwistAckermann(geometry_msgs::Twist& control_deviation, double dist_to_goal)
{
	v_max_ = cvf_->getMaximumVelocity();
	double obstacle_dist, obstacle_angle;
	cvf_->getClosestObstacle(obstacle_dist, obstacle_angle);
	
	geometry_msgs::Twist cdev;
	cdev = control_deviation;

	geometry_msgs::Point H1, H2;
	
	H1.x = - center_ax_dist_;
	H1.y = 0.0;
	H2.x = cdev.linear.x - center_ax_dist_*cos(cdev.angular.z);
	H2.y = cdev.linear.y - center_ax_dist_*sin(cdev.angular.z);
	
	double theta_reachable = angles::normalize_angle(2*atan((H2.y-H1.y)/(H2.x-H1.x)));
	// welcher winkel ist größer?
	double angle = cdev.angular.z;
	//if (fabs(theta_reachable)>fabs(angle) && fabs(H2.x-H1.x) < tolerance_trans_ && dist_to_goal > 0.5)
		//angle = theta_reachable;
	
	// Angle correction
	if(fabs(H2.x-H1.x) < 0.9*turning_radius_*fabs(angle) && !angle_correction_ && !y_correction_)
	{
		degrees_ = 0;
		angle_correction_ = true;
		if (fabs(last_vel_.linear.x) > 0.001)
			forward_ = sign(last_vel_.linear.x);
		else
			forward_ = 1;
	}
	else if(angle_correction_ && (degrees_ >= 50 || v_max_ < 0.01))
	{
			degrees_ = 0;
			forward_ *= -1.0;
	}
	if((fabs(H2.x-H1.x) > 1.1*turning_radius_*fabs(angle) || fabs(angle) < tolerance_rot_) && !stuck_)
	//if((fabs(H2.x-H1.x) > factor_*turning_radius_*fabs(angle) || fabs(angle) < tolerance_rot_ ))
	{
		angle_correction_ = false;
	}
	
	if (obstacle_dist > 0.3)
	{
		stuck_ = false;
	}
	
	// Y correction
	float turning = 1;
	if (fabs(cdev.linear.x) <= tolerance_trans_ && fabs(cdev.linear.y) > tolerance_trans_ && fabs(cdev.angular.z) <= tolerance_rot_ && !y_correction_ && fabs(last_vel_.linear.x) < 0.1)
	{
		degrees_ = 0;
		obst_dist_at_start_ = obstacle_dist;
		y_correction_ = true;
		angle_correction_ = false;
		if (fabs(last_vel_.linear.x) > 0.001)
			forward_ = sign(last_vel_.linear.x);
		else
			forward_ = -1;
	}
	//else if(y_correction_ && ( (degrees_ >= 15 && v_max_ < 0.22) || degrees_ >= 25) )
	else if(y_correction_ && ( obstacle_dist < 0.65*obst_dist_at_start_ || degrees_ >= 25) )
	{
		turning = -1;
	}
	if(y_correction_ && (degrees_ >= 50 || v_max_ < 0.001))
	{
		y_correction_ = false;
	}
		
	
	double dist_to_next_pose = getDistance2d(H1,H2);
	//if ((dist_to_next_pose < tolerance_trans_ || !checkReachability(H1,H2, cdev.angular.z)) && !y_correction_)
	//if (!checkReachability(H1,H2, cdev.angular.z) && !y_correction_ && dist_to_goal > 2*turning_radius_)
	
	if (y_correction_)
	{
		ROS_WARN("y correction");
		degrees_ ++;
		cdev.angular.z = forward_*turning*0.15*sign(cdev.linear.y);
		cdev.linear.x = forward_*fabs(cdev.angular.z)*turning_radius_;
	}
	else if (angle_correction_)
	{
		ROS_WARN("Angle correction");
		degrees_ ++;
		cdev.angular.z = 0.2*sign(angle);
		cdev.linear.x = forward_*turning_radius_*fabs(cdev.angular.z);
	}
	else if (dist_to_goal < 1*tolerance_trans_)
	{
		//ROS_WARN("direkt");
		if (fabs(cdev.linear.x) < turning_radius_*fabs(cdev.angular.z))
		{
			if (fabs(cdev.linear.x) < 0.001)
				cdev.linear.x = turning_radius_*fabs(cdev.angular.z);
			else
				cdev.linear.x *= turning_radius_*fabs(cdev.angular.z/cdev.linear.x);
		}
	}
	else if (v_max_ < 0.001)
	{
		angle_correction_ = true;
		stuck_ = true;
		forward_ = 1;
	}
	else
	{	
		double theta_reachable = angles::normalize_angle(2*atan((H2.y-H1.y)/(H2.x-H1.x)));	
		double radius, arclength, kr;
		if (fabs(theta_reachable) > 0.001)
		{
			radius = (H2.x-H1.x)/sin(theta_reachable);
			arclength = radius*theta_reachable;
			kr = 1.0/radius;
		}
		else
		{
			arclength = H2.x-H1.x;
			kr = theta_reachable/arclength;
			radius = (H2.x-H1.x)/sin(0.001);
		}
		
		/*if (fabs(theta_reachable - cdev.angular.z) > tolerance_rot_)
		{
			geometry_msgs::Point L, R;
			double turn_rad = 1.2*turning_radius_;
			L.x = H2.x - turning_radius_*sin(cdev.angular.z);
			L.y = H2.y + turning_radius_*cos(cdev.angular.z);
			R.x = H2.x + turning_radius_*sin(cdev.angular.z);
			R.y = H2.y - turning_radius_*cos(cdev.angular.z);

			double alpha_L = angles::normalize_angle(2.0*atan((L.y - turn_rad)/(L.x - H1.x)));
			double alpha_R = angles::normalize_angle(2.0*atan((R.y + turn_rad)/(R.x - H1.x)));
			
			double radius_L, arclength_L;
			double radius_R, arclength_R;
			
			if (fabs(alpha_L) < 0.001)
			{
				arclength_L = L.x - H1.x;
			}
			else
			{
				radius_L = L.x/sin(alpha_L)+turn_rad;
				arclength_L = alpha_L*radius_L;
			}
				
			if (fabs(alpha_R) < 0.001)
			{
				arclength_R = R.x - H1.x;
			}
			else
			{
				radius_R = R.x/sin(alpha_R)-turn_rad;
				arclength_R = alpha_R*radius_R;
			}
			
			if ((fabs(arclength_R) < fabs(arclength_L)) && arclength*arclength_R > 0)
			{
				kr = alpha_R/arclength_R;
			}
			else
			{
				kr = alpha_L/arclength_L;
			}
		}*/
			
		double a = 0.1;
		
		if (dist_to_goal < 1.0)
			a = 0.3;
		if (y_correction_)
			a = 0.3;
			
		/*if (!checkReachability(H1,H2, cdev.angular.z))
		{
			a = -1.0;
			ROS_ERROR("nicht erreichbar");
		}*/
		
		theta_reachable = angles::normalize_angle(theta_reachable + a*(theta_reachable - cdev.angular.z));
		/*
		if(korrektur && rechts && theta_reachable/arclength > kr)
		ROS_ERROR("Hat gereicht");
			//kr = theta_reachable/arclength;
		else if(korrektur && !rechts && theta_reachable/arclength < kr)
		ROS_ERROR("Hat gereicht");
			//kr = theta_reachable/arclength;
		else
			ROS_ERROR("Hat nicht gereicht");
		*/	
		//if(!korrektur)
			kr = theta_reachable/arclength;
		
		
		if (fabs(radius) < 1.1*turning_radius_)
		{
			ROS_WARN("Gib alles");
			kr = 1/radius;
		}
		
		if (fabs(kr) > 1.0/turning_radius_)
			kr = sign(kr)/turning_radius_;
		
		cdev.linear.x = arclength;
		cdev.angular.z = angles::normalize_angle(arclength*kr);
		
	}
	
	cdev.linear.y = cdev.angular.z * center_ax_dist_;
	
	double slowdown = 1;
	if (y_correction_)
		slowdown = 0.3;
	if (dist_to_goal < 1.0)
		slowdown = 0.8;

	cdev.linear.x *= slowdown;
	cdev.linear.y *= slowdown;
	cdev.angular.z *= slowdown;

	control_deviation = cdev;
	
	//ROS_ERROR("Ackermann-twist: (%f, %f, %f)",control_deviation.linear.x,control_deviation.linear.y,control_deviation.angular.z);
	
	return true;
}


double EBandTrajectoryCtrl::getBubbleTargetVel(const int& target_bub_num, const std::vector<Bubble>& band, geometry_msgs::Twist& VelDir)
{
	// init reference for direction vector
	VelDir.linear.x = 0.0;
	VelDir.linear.y = 0.0;
	VelDir.linear.z = 0.0;
	VelDir.angular.x = 0.0;
	VelDir.angular.y = 0.0;
	VelDir.angular.z = 0.0;

	// if we are looking at the last bubble - target vel is always zero
	if(target_bub_num >= ((int) band.size() - 1))
		return 0.0;


	// otherwise check for max_vel calculated from current bubble size
	double v_max_curr_bub, v_max_next_bub;
	double bubble_distance, angle_to_pseudo_vel, delta_vel_max;
	geometry_msgs::Twist bubble_diff;

	// distance for braking s = 0.5*v*v/a
	v_max_curr_bub = sqrt(2 * elastic_band_.at(target_bub_num).expansion * acc_max_);

	// get distance to next bubble center
	ROS_ASSERT( (target_bub_num >= 0) && ((target_bub_num +1) < (int) band.size()) );
	bubble_diff = getFrame1ToFrame2InRefFrame(band.at(target_bub_num).center.pose, band.at(target_bub_num + 1).center.pose,
												ref_frame_band_);
	angle_to_pseudo_vel = bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();

	bubble_distance = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) + (bubble_diff.linear.y * bubble_diff.linear.y) +
							(angle_to_pseudo_vel * angle_to_pseudo_vel) );

	// calculate direction vector - norm of difference
	VelDir.linear.x =bubble_diff.linear.x/bubble_distance;
	VelDir.linear.y =bubble_diff.linear.y/bubble_distance;
	VelDir.angular.z =bubble_diff.angular.z/bubble_distance;

	// next bubble center inside this bubble - take into account restrictions on next bubble
	int next_bub_num = target_bub_num + 1;
	geometry_msgs::Twist dummy_twist;
	v_max_next_bub = getBubbleTargetVel(next_bub_num, band, dummy_twist); // recursive call

	// if velocity at next bubble bigger (or equal) than our velocity - we are on the safe side
	if(v_max_next_bub >= v_max_curr_bub)
		return v_max_curr_bub;


	// otherwise max. allowed vel is next vel + plus possible reduction on the way between the bubble-centers
	
	delta_vel_max = sqrt(2 * bubble_distance * acc_max_);
	if (v_max_curr_bub > v_max_next_bub + delta_vel_max)
		v_max_curr_bub = v_max_next_bub + delta_vel_max;
	
	ROS_DEBUG("Max velocity: %f, %f", v_max_curr_bub, elastic_band_.at(target_bub_num).expansion);

	return v_max_curr_bub;
}


geometry_msgs::Twist EBandTrajectoryCtrl::getFrame1ToFrame2InRefFrame(const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& ref_frame)
{

	geometry_msgs::Pose2D frame1_pose2D, frame2_pose2D, ref_frame_pose2D;
	geometry_msgs::Pose2D frame1_pose2D_rf, frame2_pose2D_rf;
	geometry_msgs::Twist frame_diff;

	// transform all frames to Pose2d
	PoseToPose2D(frame1, frame1_pose2D);
	PoseToPose2D(frame2, frame2_pose2D);
	PoseToPose2D(ref_frame, ref_frame_pose2D);

	// transform frame1 into ref frame
	frame1_pose2D_rf.x = (frame1_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) +
							(frame1_pose2D.y - ref_frame_pose2D.y) * sin(ref_frame_pose2D.theta);
	frame1_pose2D_rf.y = -(frame1_pose2D.x - ref_frame_pose2D.x) * sin(ref_frame_pose2D.theta) +
							(frame1_pose2D.y - ref_frame_pose2D.y) * cos(ref_frame_pose2D.theta);
	frame1_pose2D_rf.theta = frame1_pose2D.theta - ref_frame_pose2D.theta;
	frame1_pose2D_rf.theta = angles::normalize_angle(frame1_pose2D_rf.theta);
	// transform frame2 into ref frame
	frame2_pose2D_rf.x = (frame2_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) +
							(frame2_pose2D.y - ref_frame_pose2D.y) * sin(ref_frame_pose2D.theta);
	frame2_pose2D_rf.y = -(frame2_pose2D.x - ref_frame_pose2D.x) * sin(ref_frame_pose2D.theta) +
							(frame2_pose2D.y - ref_frame_pose2D.y) * cos(ref_frame_pose2D.theta);
	frame2_pose2D_rf.theta = frame2_pose2D.theta - ref_frame_pose2D.theta;
	frame2_pose2D_rf.theta = angles::normalize_angle(frame2_pose2D_rf.theta);

	// get differences
	frame_diff.linear.x = frame2_pose2D_rf.x - frame1_pose2D_rf.x;
	frame_diff.linear.y = frame2_pose2D_rf.y - frame1_pose2D_rf.y;
	frame_diff.linear.z = 0.0;
	frame_diff.angular.x = 0.0;
	frame_diff.angular.y = 0.0;
	frame_diff.angular.z = frame2_pose2D_rf.theta - frame1_pose2D_rf.theta;
	// normalize angle
	frame_diff.angular.z = angles::normalize_angle(frame_diff.angular.z);

	return frame_diff;
}

geometry_msgs::Twist EBandTrajectoryCtrl::getFrame1ToFrame2InRefFrameNew(const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& ref_frame)
{

  double x1 = frame1.position.x - ref_frame.position.x;
  double y1 = frame1.position.y - ref_frame.position.y;
  double x2 = frame2.position.x - ref_frame.position.x;
  double y2 = frame2.position.y - ref_frame.position.y;
  double yaw_ref = tf::getYaw(ref_frame.orientation);

  double x_diff = x2 - x1;
  double y_diff = y2 - y1;
  double theta_diff = atan2(y_diff, x_diff);

  // Now project this vector on to the reference frame
  double rotation = angles::normalize_angle(yaw_ref);
  double x_final = x_diff * cos(rotation) + y_diff * sin(rotation);
  double y_final = - x_diff * sin(rotation) + y_diff * cos(rotation);

  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = x_final;
  twist_msg.linear.y = y_final;
  twist_msg.angular.z = angles::normalize_angle(theta_diff - yaw_ref);

  return twist_msg;
}


geometry_msgs::Twist EBandTrajectoryCtrl::transformTwistFromFrame1ToFrame2(const geometry_msgs::Twist& curr_twist,
															const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2)
{
	geometry_msgs::Pose2D frame1_pose2D, frame2_pose2D;
	geometry_msgs::Twist tmp_transformed;
	double delta_ang;

	tmp_transformed = curr_twist;

	// transform all frames to Pose2d
	PoseToPose2D(frame1, frame1_pose2D);
	PoseToPose2D(frame2, frame2_pose2D);

	// get orientation diff of frames
	delta_ang = frame2_pose2D.theta - frame1_pose2D.theta;
	delta_ang = angles::normalize_angle(delta_ang);

	// tranform twist
	tmp_transformed.linear.x = curr_twist.linear.x * cos(delta_ang) + curr_twist.linear.y * sin(delta_ang);
	tmp_transformed.linear.y = -curr_twist.linear.x * sin(delta_ang) + curr_twist.linear.y * cos(delta_ang);

	return tmp_transformed;
}


geometry_msgs::Twist EBandTrajectoryCtrl::limitTwist(const geometry_msgs::Twist& twist)
{
	geometry_msgs::Twist res = twist;

	//make sure to bound things by our velocity limits
	double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
	if (lin_overshoot > 1.0) 
	{
		res.linear.x /= lin_overshoot;
		res.linear.y /= lin_overshoot;
		// keep relations
		res.angular.z /= lin_overshoot;
	}

	if (fabs(res.angular.z) > max_vel_th_)
	{
		double scale = max_vel_th_/fabs(res.angular.z);
		//res.angular.z = max_vel_th_ * sign(res.angular.z);
		res.angular.z *= scale;
		// keep relations
		res.linear.x *= scale;
		res.linear.y *= scale;
	}

	ROS_DEBUG("Angular command %f", res.angular.z);
	return res;
}


bool EBandTrajectoryCtrl::checkReachability(geometry_msgs::Point P1, geometry_msgs::Point P2, double theta)
{

		geometry_msgs::Point L1,R1,L2,R2;
		L1.x = P1.x;
		L1.y = P1.y + turning_radius_;
		R1.x = P1.x;
		R1.y = P1.y - turning_radius_;
		L2.x = P2.x - turning_radius_*sin(theta);
		L2.y = P2.y + turning_radius_*cos(theta);
		R2.x = P2.x + turning_radius_*sin(theta);
		R2.y = P2.y - turning_radius_*cos(theta);
		
		double distLR = sqrt((L1.x-R2.x) * (L1.x-R2.x) + (L1.y-R2.y) * (L1.y-R2.y));
		if(distLR < 2.0*turning_radius_)
			return false;
		
		distLR = sqrt((L2.x-R1.x) * (L2.x-R1.x) + (L2.y-R1.y) * (L2.y-R1.y));
		if(distLR < 2.0*turning_radius_)
			return false;
	
	// everything fine - bubbles are reachable
	return true;
}

}
