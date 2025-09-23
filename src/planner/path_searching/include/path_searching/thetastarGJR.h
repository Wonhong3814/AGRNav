/**
* This file is part of Fast-Planner.
*/

#ifndef _THETASTARGJR_H
#define _THETASTARGJR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include "plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>
#include <queue>
#include <traj_utils/polynomial_traj.h>

#include <path_searching/astar.h> 
#include <plan_env/edt_environment.h>

namespace fast_planner {

class ThetastarGJR {
private:
  /* ---------- main data structure ---------- */
  std::vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable0 expanded_nodes_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::vector<NodePtr> path_nodes_;

  /* ---------- record data ---------- */
  EDTEnvironment::Ptr edt_environment_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search (Astar와 동일 키 유지) */
  double lambda_heu_;
  double margin_;
  int    allocate_num_;
  double tie_breaker_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* extra (Astar에 있는 것들 호환) */
  double weight_goal_;
  double aerial_penalty_, ground_judge_;

  /* Theta* + Jump 전용 파라미터 */
  double epsilon_;
  double jump_forward_, jump_diag_, jump_apex_;
  int    jump_samples_;
  double jump_penalty_;
  double terminate_cells_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(NodePtr end_node);

  /* heuristic function (Astar와 동일 원형) */
  double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

public:
  ThetastarGJR(){};
  ~ThetastarGJR();

  enum { REACH_END = 1, NO_PATH = 2 };

  /* main API (Astar와 동일 시그니처) */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic = false,
             double time_start = -1.0);
  void setEnvironment(const EDTEnvironment::Ptr& env);
  std::vector<Eigen::Vector3d> getPath();
  std::vector<NodePtr> getVisitedNodes();

  /* 아래 3개는 Astar 인터페이스 호환용: 내부 로직은 동일하게 둠 */
  bool checkMotionPrimitive(PolynomialTraj traj);
  double scoreMotionPrimitive(PolynomialTraj traj, Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos);
  std::vector<Eigen::Vector3d> sampleMotionPrimitive(PolynomialTraj traj, double td);

  typedef std::shared_ptr<ThetastarGJR> Ptr;
};

}  // namespace fast_planner

#endif  // _THETASTARGJR_H_

