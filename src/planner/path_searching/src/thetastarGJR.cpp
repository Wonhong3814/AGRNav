/**
* This file is part of Fast-Planner.
* (same license header as project)
*/

#include <path_searching/thetastarGJR.h>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace fast_planner
{

// ==================== Theta* & Jump 파라미터 (ROS 파라미터로 설정) ====================
namespace {
  // 점프/스무딩 파라미터 기본값
  double g_theta_eps     = 1.0;   // tie-breaker 보정(필요시)
  double g_jump_forward  = 1.4;
  double g_jump_diag     = 0.926;
  double g_jump_apex     = 2.1;
  int    g_jump_samples  = 7;     // 최소 4
  double g_jump_penalty  = 0.20;
  double g_term_cells    = 10.0;

  constexpr double PARABOLA_COEFF = 4.0;
  constexpr int    Z_SEARCH_UP    = 3;
  constexpr int    Z_SEARCH_DOWN  = 3;
}

// ==================== 유틸 ====================
static bool hasLineOfSight(const EDTEnvironment::Ptr& env,
                           const Eigen::Vector3d& a,
                           const Eigen::Vector3d& b,
                           double res)
{
  if (!env) return false;
  Eigen::Vector3d d = b - a;
  double len = d.norm();
  if (len < 1e-9) return true;
  int steps = std::max(2, (int)std::ceil(len / res));
  Eigen::Vector3d step = d / (double)steps;
  Eigen::Vector3d p = a;
  for (int i = 0; i <= steps; ++i) {
    if (env->sdf_map_->getInflateOccupancy(p) != 0) {
      return false;
    }
    p += step;
  }
  return true;
}

static bool checkJumpArc(const EDTEnvironment::Ptr& env,
                         const Eigen::Vector3d& from,
                         const Eigen::Vector3d& delta_world,
                         double res,
                         double jump_apex,
                         int jump_samples,
                         Eigen::Vector3d& out_target,
                         std::vector<Eigen::Vector3d>* out_samples)
{
  if (!env) return false;

  const double horiz_len = delta_world.head<2>().norm();
  if (horiz_len <= 1e-9) return false;

  const double desired_step_xy = res;
  int num_front_by_res = static_cast<int>(std::ceil(0.5 * horiz_len / desired_step_xy));
  int num_front = std::max(3, std::min(50, std::max(jump_samples, num_front_by_res)));

  std::vector<Eigen::Vector3d> samples;
  samples.reserve(num_front + 2);

  for (int i = 2; i <= num_front; ++i) {
    const double t = 0.5 * (static_cast<double>(i) / static_cast<double>(num_front)); // 0~0.5
    const Eigen::Vector3d horiz_world = delta_world * t;
    const double z = PARABOLA_COEFF * jump_apex * t * (1.0 - t);
    Eigen::Vector3d p = from + horiz_world + Eigen::Vector3d(0.0, 0.0, z);

    //if (env->sdf_map_->getInflateOccupancy(p) != 0) return false;
    double dist = env->evaluateCoarseEDT(p, -1.0);
    if (dist < 0.0 || dist < 0.5) return false; // margin �

    samples.push_back(p);
  }

  out_target = from + delta_world;

  // 간단 착지 z 스냅
  auto try_probe = [&](double z)->bool{
    Eigen::Vector3d probe = out_target;
    probe.z() = z;
    return env->sdf_map_->getInflateOccupancy(probe) == 0;
  };

  bool snapped = false;
  double snap_z = from.z();
  if (try_probe(snap_z)) {
    out_target.z() = snap_z; snapped = true;
  }

  for (int k=1; !snapped && k<=Z_SEARCH_DOWN; ++k) { if (try_probe(from.z() - k*res)) { out_target.z() = from.z() - k*res; snapped = true; } }
  for (int k=1; !snapped && k<=Z_SEARCH_UP;   ++k) { if (try_probe(from.z() + k*res)) { out_target.z() = from.z() + k*res; snapped = true; } }


  if (!snapped) {
    //if (env->sdf_map_->getInflateOccupancy(out_target) != 0) return false;
    double dist = env->evaluateCoarseEDT(out_target, -1.0); 
    if (dist < 0.0 || dist < 0.5) return false;
  }

  if (out_samples) *out_samples = samples;
  return true;
}

// ==================== ThetastarGJR 구현 ====================

ThetastarGJR::~ThetastarGJR() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

void ThetastarGJR::setParam(ros::NodeHandle& nh) {
  // Astar와 동일 키 유지
  nh.param("astar/resolution_astar", resolution_, -1.0);
  nh.param("astar/time_resolution",  time_resolution_, -1.0);
  nh.param("astar/lambda_heu",       lambda_heu_, -1.0);
  nh.param("astar/margin",           margin_, -1.0);
  nh.param("astar/allocate_num",     allocate_num_, -1);
  nh.param("astar/weight_goal",      weight_goal_, -1.0);
  nh.param("astar/aerial_penalty",   aerial_penalty_, 0.0);
  nh.param("astar/ground_judge",     ground_judge_, 0.0);
  tie_breaker_ = 1.0 + 1.0 / 10000;

  // Theta* + Jump 파라미터 (별도 네임스페이스)
  nh.param("thetastargjr/epsilon",          epsilon_,        1.0);
  nh.param("thetastargjr/jump_forward",     jump_forward_,   1.4);
  nh.param("thetastargjr/jump_diag",        jump_diag_,      0.926);
  nh.param("thetastargjr/jump_apex",        jump_apex_,      2.1);
  nh.param("thetastargjr/jump_samples",     jump_samples_,   7);
  nh.param("thetastargjr/jump_penalty",     jump_penalty_,   0.20);
  nh.param("thetastargjr/terminate_cells",  terminate_cells_,10.0);

  // 내부 static에도 복사(코드 간소화)
  g_theta_eps    = epsilon_;
  g_jump_forward = jump_forward_;
  g_jump_diag    = jump_diag_;
  g_jump_apex    = jump_apex_;
  g_jump_samples = std::max(4, jump_samples_);
  g_jump_penalty = jump_penalty_;
  g_term_cells   = terminate_cells_;
}

void ThetastarGJR::init() {
  inv_resolution_      = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->getMapRegion(origin_, map_size_3d_);

  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

void ThetastarGJR::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void ThetastarGJR::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++) {
    NodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

Eigen::Vector3i ThetastarGJR::posToIndex(Eigen::Vector3d pt) {
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  return idx;
}

int ThetastarGJR::timeToIndex(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

double ThetastarGJR::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  double h;
  int diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double ThetastarGJR::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  return tie_breaker_ * (dx + dy + dz);
}

double ThetastarGJR::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

int ThetastarGJR::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic, double time_start) {
  if (!edt_environment_) {
    ROS_ERROR("[ThetastarGJR] EDT environment is null");
    return NO_PATH;
  }
  if (resolution_ <= 0.0 || allocate_num_ <= 0) {
    ROS_ERROR("[ThetastarGJR] invalid params (resolution/allocate_num)");
    return NO_PATH;
  }

  reset();

  // 시작 노드
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  if (cur_node->position[2] < 0) cur_node->position[2] = 0;
  if (cur_node->position[2] >= ground_judge_) {
    cur_node->g_score         += aerial_penalty_ * cur_node->position[2] / 2.0;
    cur_node->penalty_g_score  = aerial_penalty_ * cur_node->position[2] / 2.0;
    cur_node->motion_state     = 1; // flying
  } else {
    cur_node->motion_state     = 0; // rolling
    cur_node->penalty_g_score  = 0;
  }

  Eigen::Vector3i end_index;
  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic) {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  } else
    expanded_nodes_.insert(cur_node->index, cur_node);

  NodePtr terminate_node = NULL;

  // 26-방향 이웃
  static const std::vector<Eigen::Vector3d> neighbor_dirs = [](){
    std::vector<Eigen::Vector3d> v;
    v.reserve(26);
    for (int dx=-1; dx<=1; ++dx)
      for (int dy=-1; dy<=1; ++dy)
        for (int dz=-1; dz<=1; ++dz) {
          if (dx==0 && dy==0 && dz==0) continue;
          v.emplace_back(dx,dy,dz);
        }
    return v;
  }();

  // 점프 프리미티브 (body == world 가정)
  const std::vector<Eigen::Vector3d> jump_prims = {
      {  g_jump_forward,  0.0, 0.0},
      { -g_jump_forward,  0.0, 0.0},
      {  0.0,  g_jump_forward, 0.0},
      {  0.0, -g_jump_forward, 0.0},
      {  g_jump_diag,  g_jump_diag, 0.0},
      { -g_jump_diag,  g_jump_diag, 0.0},
      {  g_jump_diag, -g_jump_diag, 0.0},
      { -g_jump_diag, -g_jump_diag, 0.0}
  };

  const double term_r2 = std::pow(resolution_ * g_term_cells, 2);

  // ---------- 탐색 루프 ----------
  while (!open_set_.empty()) {
    cur_node = open_set_.top();
    open_set_.pop();

    // 종료 판정
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                     abs(cur_node->index(1) - end_index(1)) <= 1 &&
                     abs(cur_node->index(2) - end_index(2)) <= 1;

    if (reach_end || (cur_node->position - end_pt).squaredNorm() <= term_r2) {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;
      return REACH_END;
    }

    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    const Eigen::Vector3d cur_pos = cur_node->position;

    // (1) Theta* 확장
    for (const auto& d : neighbor_dirs) {
      Eigen::Vector3d d_pos = d * resolution_;
      Eigen::Vector3d pro_pos = cur_pos + d_pos;

      if (edt_environment_->sdf_map_->getInflateOccupancy(pro_pos) != 0) continue;

      Eigen::Vector3i pro_id = posToIndex(pro_pos);
      int pro_t_id = dynamic ? timeToIndex(cur_node->time + 1.0) : 0;

      NodePtr pro_node =
          dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

      if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) continue;

      // LOS가 되면 parent→neighbor로 스무딩, 아니면 current→neighbor
      double g_cand = std::numeric_limits<double>::infinity();
      Node* parent_for_best = cur_node;

      if (cur_node->parent != NULL) {
        if (hasLineOfSight(edt_environment_, cur_node->parent->position, pro_pos, resolution_)) {
          double dist_parent = (cur_node->parent->position - pro_pos).norm();
          g_cand = cur_node->parent->g_score + dist_parent;
          parent_for_best = cur_node->parent;
        }
      }
      if (!std::isfinite(g_cand)) {
        double dist_direct = d_pos.norm();
        g_cand = cur_node->g_score + dist_direct;
      }

      // 고도 패널티
      double penalty_g_score = 0.0;
      bool next_motion_state = false;
      if (pro_pos[2] > ground_judge_) {
        g_cand -= cur_node->penalty_g_score;
        penalty_g_score = aerial_penalty_ * pro_pos[2] / 2.0;
        g_cand += penalty_g_score;
        next_motion_state = true;
      }

      double f_cand = g_cand + lambda_heu_ * getEuclHeu(pro_pos, end_pt);

      if (pro_node == NULL) {
        if (use_node_num_ == allocate_num_) {
          cout << "run out of memory." << endl;
          return NO_PATH;
        }
        pro_node = path_node_pool_[use_node_num_++];
        pro_node->index      = pro_id;
        pro_node->position   = pro_pos;
        pro_node->f_score    = f_cand;
        pro_node->g_score    = g_cand;
        pro_node->parent     = parent_for_best;
        pro_node->node_state = IN_OPEN_SET;
        pro_node->motion_state    = next_motion_state;
        pro_node->penalty_g_score = penalty_g_score;

        if (dynamic) {
          pro_node->time     = cur_node->time + 1.0;
          pro_node->time_idx = timeToIndex(pro_node->time);
          expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
        } else {
          expanded_nodes_.insert(pro_id, pro_node);
        }
        open_set_.push(pro_node);
      } else if (pro_node->node_state == IN_OPEN_SET) {
        if (g_cand < pro_node->g_score) {
          pro_node->position   = pro_pos;
          pro_node->f_score    = f_cand;
          pro_node->g_score    = g_cand;
          pro_node->parent     = parent_for_best;
          pro_node->motion_state    = next_motion_state;
          pro_node->penalty_g_score = penalty_g_score;
          if (dynamic) pro_node->time = cur_node->time + 1.0;
        }
      }
    }

    // (2) 점프 확장
    for (const auto& db : jump_prims) {
      Eigen::Vector3d landing;
      std::vector<Eigen::Vector3d> arc;
      if (!checkJumpArc(edt_environment_, cur_pos, db,
                        resolution_, g_jump_apex, g_jump_samples,
                        landing, &arc)) {
        continue;
      }
      if (edt_environment_->sdf_map_->getInflateOccupancy(landing) != 0) continue;

      Eigen::Vector3i land_id = posToIndex(landing);
      int land_t_id = dynamic ? timeToIndex(cur_node->time + 1.0) : 0;

      NodePtr jump_node =
        dynamic ? expanded_nodes_.find(land_id, land_t_id) : expanded_nodes_.find(land_id);

      if (jump_node != NULL && jump_node->node_state == IN_CLOSE_SET) continue;

      double move_cost = (landing - cur_pos).head<2>().norm();
      double g_cand = cur_node->g_score + move_cost + g_jump_penalty;

      double penalty_g_score = 0.0;
      bool next_motion_state = true;
      if (landing[2] > ground_judge_) {
        g_cand -= cur_node->penalty_g_score;
        penalty_g_score = aerial_penalty_ * landing[2] / 2.0;
        g_cand += penalty_g_score;
      }

      double f_cand = g_cand + lambda_heu_ * getEuclHeu(landing, end_pt);

      if (jump_node == NULL) {
        if (use_node_num_ == allocate_num_) {
          cout << "run out of memory." << endl;
          return NO_PATH;
        }
        jump_node = path_node_pool_[use_node_num_++];
        jump_node->index      = land_id;
        jump_node->position   = landing;
        jump_node->f_score    = f_cand;
        jump_node->g_score    = g_cand;
        jump_node->parent     = cur_node;
        jump_node->node_state = IN_OPEN_SET;
        jump_node->motion_state    = next_motion_state;
        jump_node->penalty_g_score = penalty_g_score;

        if (dynamic) {
          jump_node->time     = cur_node->time + 1.0;
          jump_node->time_idx = timeToIndex(jump_node->time);
          expanded_nodes_.insert(land_id, jump_node->time, jump_node);
        } else {
          expanded_nodes_.insert(land_id, jump_node);
        }
        open_set_.push(jump_node);
      } else if (jump_node->node_state == IN_OPEN_SET) {
        if (g_cand < jump_node->g_score) {
          jump_node->position   = landing;
          jump_node->f_score    = f_cand;
          jump_node->g_score    = g_cand;
          jump_node->parent     = cur_node;
          jump_node->motion_state    = next_motion_state;
          jump_node->penalty_g_score = penalty_g_score;
          if (dynamic) jump_node->time = cur_node->time + 1.0;
        }
      }
    }
  }

  // 실패
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void ThetastarGJR::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> ThetastarGJR::getPath() {
  vector<Eigen::Vector3d> path;
  path.reserve(path_nodes_.size());
  for (int i = 0; i < (int)path_nodes_.size(); ++i) {
    path.push_back(path_nodes_[i]->position);
  }
  return path;
}

bool ThetastarGJR::checkMotionPrimitive(PolynomialTraj traj){
  bool result;
  double tau = traj.getTimeSum();
  for (double t = 0.0; t <= tau; t += 0.05){
    Eigen::Vector3d pos = traj.evaluate(t);
    if(edt_environment_->evaluateCoarseEDT(pos, -1.0) < 0){
      result = false;
      return result;
    }
  }
  result = true;
  return result;
}

double ThetastarGJR::scoreMotionPrimitive(PolynomialTraj traj, Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos){
  double nearest_dist = 10000;
  double score;
  bool dist_flag = false;
  double tau = traj.getTimeSum();
  for (double t = 0.0; t <= tau; t += 0.05){
    Eigen::Vector3d pos = traj.evaluate(t);
    if(edt_environment_->evaluateCoarseEDT(pos, -1.0) >= margin_ * 1.5 && edt_environment_->evaluateCoarseEDT(pos, -1.0) < 50){
      if(nearest_dist > edt_environment_->evaluateCoarseEDT(pos, -1.0))
        nearest_dist = edt_environment_->evaluateCoarseEDT(pos, -1.0);
      dist_flag = true;
    }
  }
  if(!dist_flag) nearest_dist = 0;
  score = weight_goal_ * (goal_pos - start_pos).norm() - nearest_dist;
  return score;
}

std::vector<Eigen::Vector3d> ThetastarGJR::sampleMotionPrimitive(PolynomialTraj traj, double td){
  std::vector<Eigen::Vector3d> wpts;
  double tau = traj.getTimeSum();
  for (double t = 0.0; t < tau; t += td){
    Eigen::Vector3d pos = traj.evaluate(t);
    wpts.push_back(pos);
  }
  return wpts;  
}

std::vector<NodePtr> ThetastarGJR::getVisitedNodes() {
  std::vector<NodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + std::max(0, use_node_num_ - 1));
  return visited;
}

}  // namespace fast_planner

