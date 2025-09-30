/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

#include <plan_manage/kino_replan_fsm.h>
#include <traj_utils/polynomial_traj.h>   // ✅ PolynomialTraj 변환 유틸

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  /* fsm params */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }
  
  /* initialize modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));
  odom_yaw_ = 0;

  // ✅ Initialize ThetaStarGJR
  thetastar_gjr_.setEnvironment(planner_manager_->edt_environment_);
  thetastar_gjr_.init();

  /* timers and subscribers */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  poly_pub_    = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/planning/polynomial", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered new goal!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 0;
  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.4, Eigen::Vector4d(0, 1, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d ypr = uav_utils::quaternion_to_ypr(odom_orient_);
  odom_yaw_ = ypr(0) * 180 / M_PI;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "waiting for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_ || !trigger_) return;
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }
    case WAIT_TARGET: {
      if (!have_target_) return;
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;
    }
    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      bool success = callKinodynamicReplan();
      if (success) changeFSMExecState(EXEC_TRAJ, "FSM");
      else changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;
    }
    case EXEC_TRAJ: {
      if ((end_pt_ - odom_pos_).norm() < 0.5) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      break;
    }
    case REPLAN_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) changeFSMExecState(EXEC_TRAJ, "FSM");
      else changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  if (!have_target_ || last_path_.empty()) return;

  for (auto& p : last_path_) {
    double dist = planner_manager_->edt_environment_->evaluateCoarseEDT(p, -1.0);
    if (dist < 0.3) {  // threshold
      ROS_WARN("[ThetastarGJR] Collision detected along path. Triggering replan.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      return;
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  // Run ThetaStarGJR search
  int result = thetastar_gjr_.search(start_pt_, end_pt_, false, 0.0);

  if (result == ThetastarGJR::REACH_END) {
    last_path_ = thetastar_gjr_.getPath();

    if (last_path_.empty()) {
      ROS_WARN("[ThetastarGJR] Path is empty.");
      return false;
    }

    // ✅ RViz에 시각화 (green line)
    visualization_->displayPathList(last_path_, 0.15,
                                    Eigen::Vector4d(0.0, 1.0, 0.0, 1.0),
                                    "gjr_path");

    // ✅ Trajectory 변환 후 퍼블리시
    PolynomialTraj poly_traj;
    poly_traj = PolynomialTraj::fitFromWaypoints(last_path_, 5.0);  // 5초짜리 fitting 예시

    quadrotor_msgs::PolynomialTrajectory trajMsg;
    trajMsg.header.stamp = ros::Time::now();
    trajMsg.header.frame_id = "world";
    trajMsg.trajectory_id = ros::Time::now().toNSec();

    trajMsg.num_order = 5;
    trajMsg.num_segment = poly_traj.getPieceNum();
    trajMsg.time = poly_traj.getTimes();

    for (int dim = 0; dim < 3; ++dim) {
      auto coef = poly_traj.getCoef(dim);
      for (auto& seg : coef) {
        for (auto& c : seg) {
          if (dim == 0) trajMsg.coef_x.push_back(c);
          if (dim == 1) trajMsg.coef_y.push_back(c);
          if (dim == 2) trajMsg.coef_z.push_back(c);
        }
        trajMsg.order.push_back(poly_traj.getOrder());
      }
    }

    poly_pub_.publish(trajMsg);

    ROS_INFO("[ThetastarGJR] Jump path planning success. Path length = %zu", last_path_.size());
    return true;
  } else {
    ROS_WARN("[ThetastarGJR] Jump path planning failed.");
    return false;
  }
}

}  // namespace fast_planner
