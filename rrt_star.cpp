/**
 * @file rrt_star.cpp
 * @author vss2sn
 * @brief Contains the RRTStar class 
 */

 // edited by Han Lee (updated for square goal region)

#include "path_planning/rrt_star.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <limits>
#include <queue>   // (not strictly required, but kept if you later want queues)
#include <cassert>

// constexpr double half_grid_unit = 0.5;
constexpr double precision_limit = 0.000001;

// Tunable constant for RRT* near radius in 2D: r(n)=min(gamma*sqrt(log n / n), eta)
static constexpr double kGammaRRT = 35.0;

// ------------------ small local helpers (no header changes) ------------------

static inline double Euclid(int x1, int y1, int x2, int y2) {
  const double dx = double(x1) - double(x2);
  const double dy = double(y1) - double(y2);
  return std::sqrt(dx*dx + dy*dy);
}

static inline double RadiusRRTStar(std::size_t n_nodes, double eta) {
  const double m = std::max<std::size_t>(n_nodes, 2);
  const double r = kGammaRRT * std::sqrt(std::log(m) / m);
  return std::min(r, eta);
}

// ------------------ RRTStar methods ------------------

std::tuple<bool, Node> RRTStar::FindNearestPoint(Node& new_node) {
  // In the original code, new_node is a random sample.
  // We will:
  // 1) find true nearest over point_list_
  // 2) steer toward sample by at most threshold_ (used as η)
  // 3) compute near set using RRT* radius
  // 4) pick parent that minimizes cost + dist (collision-free)
  //
  // Returns (found_parent, parent_node). Also mutates new_node to the steered pose
  // and fills its pid_/cost_ if a parent is found. We still touch near_nodes_
  // for compatibility, but we don't rely on it for rewiring anymore.

  if (point_list_.empty()) return {false, Node()};

  // ---- 1) True nearest over ALL nodes ----
  double best_d = std::numeric_limits<double>::infinity();
  Node nearest_node;
  for (const auto& node : point_list_) {
    const double d = Euclid(node.x_, node.y_, new_node.x_, new_node.y_);
    if (d < best_d) {
      best_d = d;
      nearest_node = node;
    }
  }

  // ---- 2) Steer: move from nearest toward sample by at most eta (=threshold_) ----
  // Keep threshold_ semantics: step size in grid cells.
  const double eta = static_cast<double>(threshold_);
  if (best_d <= precision_limit) {
    // Sample landed exactly on existing node -> reject (no new state)
    return {false, Node()};
  }
  const double step = std::min(eta, best_d);
  const double ux = (new_node.x_ - nearest_node.x_) / best_d;
  const double uy = (new_node.y_ - nearest_node.y_) / best_d;
  Node steered = new_node;
  steered.x_ = std::clamp<int>(static_cast<int>(std::round(nearest_node.x_ + ux * step)), 0, n_-1);
  steered.y_ = std::clamp<int>(static_cast<int>(std::round(nearest_node.y_ + uy * step)), 0, n_-1);
  steered.id_ = steered.x_ * n_ + steered.y_;

  // If already considered/occupied, skip (planner loop also guards this)
  if (grid_[steered.x_][steered.y_] != 0) {
    return {false, Node()};
  }

  // If edge to steered collides, skip
  if (IsAnyObstacleInPath(nearest_node, steered)) {
    return {false, Node()};
  }

  // ---- 3) Near set with shrinking radius ----
  const double r = RadiusRRTStar(point_list_.size(), eta);
  std::vector<Node> x_near;
  x_near.reserve(32);

  for (const auto& node : point_list_) {
    const double d = Euclid(node.x_, node.y_, steered.x_, steered.y_);
    if (d <= r + 1e-9) {
      x_near.push_back(node);
    }
  }

  // Maintain near_nodes_ adjacency in both directions for compatibility
  near_nodes_.insert({steered, {}});
  for (const auto& nn : x_near) {
    near_nodes_[steered].push_back(nn);
    near_nodes_[nn].push_back(steered);
  }

  // If no near neighbor can connect, fall back to the nearest (if it connects)
  bool found_parent = false;
  Node parent = nearest_node;
  double best_cost = nearest_node.cost_ + Euclid(nearest_node.x_, nearest_node.y_, steered.x_, steered.y_);

  // ---- 4) Choose best parent among X_near ----
  for (const auto& cand : x_near) {
    if (CompareCoordinates(cand, steered)) continue;
    if (IsAnyObstacleInPath(cand, steered)) continue;
    const double c = cand.cost_ + Euclid(cand.x_, cand.y_, steered.x_, steered.y_);
    if (c + 1e-12 < best_cost) {
      best_cost = c;
      parent = cand;
      found_parent = true;
    }
  }
  // If none in near set was better, but nearest connects, it's at least valid
  if (!found_parent) {
    // already collision-checked nearest -> steered above
    found_parent = true;
  }

  if (!found_parent) {
    near_nodes_.erase(steered);
    return {false, Node()};
  }

  // Return steered node by mutating new_node and setting its parent/cost
  new_node = steered;
  new_node.pid_ = parent.id_;
  new_node.cost_ = best_cost;
  return {true, parent};
}

bool RRTStar::IsAnyObstacleInPath(const Node& n_1, const Node& n_2) const {
  // Robust grid collision check with Bresenham line traversal over grid_
  auto in_bounds = [this](int x, int y) {
    return 0 <= x && x < n_ && 0 <= y && y < n_;
  };
  if (!in_bounds(n_1.x_, n_1.y_) || !in_bounds(n_2.x_, n_2.y_)) return true;

  int x0 = n_1.x_, y0 = n_1.y_;
  const int x1 = n_2.x_, y1 = n_2.y_;

  int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  while (true) {
    if (grid_[x0][y0] == 1) return true;  // obstacle
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
  return false;
}

Node RRTStar::GenerateRandomNode() const {
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n_ - 1);  // define the range
  const int x = distr(eng);
  const int y = distr(eng);
  return Node(x, y, 0, 0, n_ * x + y, 0);
}

void RRTStar::Rewire(const Node& new_node) {
  // Iterative rewiring using the SAME near radius used for parent selection.
  // We recompute neighbors on-the-fly (no recursion, no reliance on cached near_nodes_).
  const double eta = static_cast<double>(threshold_);
  const double r = RadiusRRTStar(point_list_.size(), eta);

  // Find neighbors within r of new_node in point_list_
  std::vector<Node> neighbors;
  neighbors.reserve(32);
  const Node new_node_in_point_list = *point_list_.find(new_node);

  for (const auto& cand : point_list_) {
    if (CompareCoordinates(cand, new_node_in_point_list)) continue;
    const double d = Euclid(cand.x_, cand.y_, new_node_in_point_list.x_, new_node_in_point_list.y_);
    if (d <= r + 1e-9) {
      neighbors.push_back(cand);
    }
  }

  for (auto neighbour : neighbors) {
    const double dist = Euclid(neighbour.x_, neighbour.y_, new_node_in_point_list.x_, new_node_in_point_list.y_);
    const double new_cost = new_node_in_point_list.cost_ + dist;
    if (new_cost + 1e-12 < neighbour.cost_ &&
        neighbour.id_ != new_node_in_point_list.id_ &&
        !IsAnyObstacleInPath(new_node_in_point_list, neighbour)) {

      // Update neighbour (erase+insert to refresh in set)
      neighbour.pid_ = new_node_in_point_list.id_;
      neighbour.cost_ = new_cost;
      point_list_.erase(neighbour);
      point_list_.insert(neighbour);
    }
  }
}

std::tuple<bool, std::vector<Node>> RRTStar::Plan(const Node& start,
                                                  const Node& goal) {
  start_ = start;
  goal_ = goal;
  grid_ = original_grid_;

  // Ensure start has correct id/pid/cost regardless of caller
  start_.id_  = start_.x_ * n_ + start_.y_;
  start_.pid_ = start_.id_;
  start_.cost_ = 0.0;
  // Ensure goal id as well (used for lookups)
  goal_.id_ = goal_.x_ * n_ + goal_.y_;

  int max_iterations = max_iter_x_factor_ * n_ * n_;
  CreateObstacleList();
  point_list_.insert(start_);
  grid_[start_.x_][start_.y_] = 3;
  int total_points = 1; // HAN: added to count total number of points at the end

  // HAN: ensure entire goal square is free space
  for (int dx = -goal_half_size_; dx <= goal_half_size_; ++dx) {
    for (int dy = -goal_half_size_; dy <= goal_half_size_; ++dy) {
      int gx = goal_.x_ + dx;
      int gy = goal_.y_ + dy;
      if (0 <= gx && gx < n_ && 0 <= gy && gy < n_) {
        grid_[gx][gy] = 0;
      }
    }
  }

  int iteration = 0;
  Node new_node = start_;
  bool found_goal = false;

  // Early success if start already in goal region
  if (IsInGoalRegion(new_node)) {
    found_goal = true;
  }

  while (iteration < max_iterations) {
    iteration++;
    // sample
    new_node = GenerateRandomNode();

    // (Pre) Skip if the sample is already not free (we'll steer anyway, but it helps)
    if (grid_[new_node.x_][new_node.y_] != 0) {
      continue;
    }

    // Nearest/Steer/ChooseParent via FindNearestPoint (now RRT* compliant)
    auto [found_parent, parent] = FindNearestPoint(new_node);
    if (!found_parent) {
      continue;
    }

    // Setting to 2 implies visited/considered
    grid_[new_node.x_][new_node.y_] = 2;
    point_list_.insert(new_node);
    total_points++;

    // Rewire neighbors
    Rewire(new_node);

    // HAN: success as soon as we land inside the goal square
    if (IsInGoalRegion(new_node)) {
      found_goal = true;
      break; // exit early on success
    }

    // (Optional) if you still want to allow connecting to the goal center when visible
    // keep this; it can improve cost if your goal region center is used in CreatePath().
    (void)CheckGoalVisible(new_node);
  }

  std::cout << "Total nodes generated: " << total_points << '\n';

  if (!found_goal) {
    // If we didn't directly land in the region, but connected goal center, path may still exist
    // CreatePath() already handles picking best node in region if present.
    const auto path = CreatePath();
    if (path.empty()) {
      return {false, {}};
    }
    return {true, path};
  }
  return {true, CreatePath()};
}

// HAN: custom function to support region as end goal 
bool RRTStar::IsInGoalRegion(const Node& n) const {
  return (std::abs(n.x_ - goal_.x_) <= goal_half_size_) &&
         (std::abs(n.y_ - goal_.y_) <= goal_half_size_);
}

bool RRTStar::CheckGoalVisible(const Node& new_node) {
  // Keep your existing semantics, but rely on Bresenham for collision checks.
  // Also, do not erase/insert new_node here; it's already in point_list_.

  // HAN: success as soon as we land inside the goal square 
  if (IsInGoalRegion(new_node)) {
    // ensure newest values are in point_list_
    auto it = point_list_.find(new_node);
    if (it != point_list_.end()) {
      point_list_.erase(it);
    }
    point_list_.insert(new_node);
    return true;
  }

  // all distance/cost computations remain relative to the center (goal_)
  const auto dist = std::sqrt(std::pow(goal_.x_ - new_node.x_, 2) +
                              std::pow(goal_.y_ - new_node.y_, 2));
  if (dist > static_cast<double>(threshold_)) {
    return false;
  }
  if (CompareCoordinates(goal_, new_node)) {
    point_list_.erase(new_node);
    point_list_.insert(new_node);
    return true;
  }
  if (!IsAnyObstacleInPath(new_node, goal_)) {
    if (auto it = point_list_.find(goal_);
        it != point_list_.end() && it->cost_ > dist + new_node.cost_) {
      auto goal_in_point_list = goal_;
      goal_in_point_list.cost_ = dist + new_node.cost_;
      goal_in_point_list.pid_ = new_node.id_;
      point_list_.erase(goal_in_point_list);
      point_list_.insert(goal_in_point_list);
    } else if (it == point_list_.end()) {
      auto goal_in_point_list = goal_;
      goal_in_point_list.cost_ = dist + new_node.cost_;
      goal_in_point_list.pid_ = new_node.id_;
      point_list_.insert(goal_in_point_list);
    }
    return true;
  }
  return false;
}

void RRTStar::CreateObstacleList() {
  obstacle_list_.clear();
  for (int i = 0; i < n_; i++) {
    for (int j = 0; j < n_; j++) {
      if (grid_[i][j] == 1) {
        Node obs(i, j, 0, 0, i * n_ + j, 0);
        obstacle_list_.push_back(obs);
      }
    }
  }
}

std::vector<Node> RRTStar::CreatePath() {
  std::vector<Node> path;

  //  HAN: choose the cheapest node inside the goal square if available
  bool have_goal_node = false;
  Node best_goal_node;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int dx = -goal_half_size_; dx <= goal_half_size_; ++dx) {
    for (int dy = -goal_half_size_; dy <= goal_half_size_; ++dy) {
      int gx = goal_.x_ + dx;
      int gy = goal_.y_ + dy;
      if (gx < 0 || gx >= n_ || gy < 0 || gy >= n_) continue;

      Node candidate(gx, gy, 0, 0, gx * n_ + gy, 0);
      auto it = point_list_.find(candidate);
      if (it != point_list_.end() && it->cost_ < best_cost) {
        best_cost = it->cost_;
        best_goal_node = *it;
        have_goal_node = true;
      }
    }
  }

  // Fallback to the center node if we never stepped into the region
  Node target;
  if (have_goal_node) {
    target = best_goal_node;
  } else {
    auto it = point_list_.find(goal_);
    if (it == point_list_.end()) {
      return path; // empty; shouldn't happen if Plan() said success
    }
    target = *it;
  }

  // Reconstruct path from target back to start_
  Node current = target;
  while (!CompareCoordinates(current, start_)) {
    path.push_back(current);
    current = *point_list_.find(
        Node(current.pid_ / n_, current.pid_ % n_, 0, 0, current.pid_));
  }
  path.push_back(current);
  return path;
}

void RRTStar::SetParams(const int threshold, const int max_iter_x_factor) {
  // Keep your original signature; interpret threshold as the step size eta.
  threshold_ = threshold;
  max_iter_x_factor_ = max_iter_x_factor;
}

// ------------------ Optional standalone main ------------------
#ifdef BUILD_INDIVIDUAL
#include <iostream>

int main() {
  std::cout << "USING BUILD_INDIVIDUAL MAIN HAN\n";

  constexpr int n = 128; // GRID SIZE
  std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
  MakeGrid(grid);

    /////////////////////////////////////////////////////////////////////

  // TEST 1
  // Place all obstacles in a tiny corner where they won't interfere - overlap in upper right corner
  // Node start(20, 10, 0, 0, 0, 0);
  // Node goal(95, 95, 0, 0, 0, 0);
  // int obs_left[]   = {126, 126, 126, 126, 126};
  // int obs_right[]  = {127, 127, 127, 127, 127};
  // int obs_top[]    = {0,   0,   0,   0,   0  };
  // int obs_bottom[] = {1,   1,   1,   1,   1  };
  // for (int i = 0; i < 5; ++i)
  //   for (int x = obs_left[i]; x < obs_right[i]; ++x)
  //     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
  //       grid[x][y] = 1;


  // // TEST 2
  // small obstacles, goal in upper right corner
  // Node start(24, 38, 0, 0, 0, 0);
  // Node goal(111, 15, 0, 0, 0, 0);
  // int obs_left[]   = { 5,   103, 74,  10,  95 };
  // int obs_right[]  = { 6,   104, 75,  11,  96 };
  // int obs_top[]    = { 15,  6,   62,  111, 23 };
  // int obs_bottom[] = { 16,  7,   63,  112, 24 };
  // for (int i = 0; i < 5; ++i)
  //   for (int x = obs_left[i]; x < obs_right[i]; ++x)
  //     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
  //       grid[x][y] = 1;

  // // TEST 3
//   Node start(24, 38, 0, 0, 0, 0);
//   Node goal(111, 15, 0, 0, 0, 0);
// int obs_left[]   = { 5, 100, 74, 10, 95 };
// int obs_right[]  = { 10, 105, 79, 15, 100 };
// int obs_top[]    = { 15, 6, 62, 111, 23 };
// int obs_bottom[] = { 20, 11, 67, 116, 28 };
// for (int i = 0; i < 5; ++i)
//   for (int x = obs_left[i]; x < obs_right[i]; ++x)
//     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
//       grid[x][y] = 1;

  //   // TEST 4
  // Node start(111, 15, 0, 0, 0, 0);
  // Node goal(111, 15, 0, 0, 0, 0);
  // int obs_left[]   = {126, 126, 126, 126, 126};
  // int obs_right[]  = {127, 127, 127, 127, 127};
  // int obs_top[]    = {0,   0,   0,   0,   0  };
  // int obs_bottom[] = {1,   1,   1,   1,   1  };
  // for (int i = 0; i < 5; ++i)
  //   for (int x = obs_left[i]; x < obs_right[i]; ++x)
  //     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
  //       grid[x][y] = 1;

  // // TEST 5
  // Node start(100, 38, 0, 0, 0, 0);
  // Node goal(10, 110, 0, 0, 0, 0);
  // int obs_left[]   = {115, 115, 60,  5,  65};
  // int obs_right[]  = {120, 120, 65, 10,  70};
  // int obs_top[]    = { 55, 100, 60, 50, 110};
  // int obs_bottom[] = { 60, 105, 65, 55, 115};
  // for (int i = 0; i < 5; ++i)
  //   for (int x = obs_left[i]; x < obs_right[i]; ++x)
  //     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
  //       grid[x][y] = 1;

  // // TEST 6
  // Node start(5, 10, 0, 0, 0, 0);
  // Node goal(60, 110, 0, 0, 0, 0);
  // int obs_left[]   = {115,  5, 60,  5, 115};
  // int obs_right[]  = {120, 10, 65, 10, 120};
  // int obs_top[]    = {115,115, 60, 50,  65};
  // int obs_bottom[] = {120,120, 65, 55,  70};
  // for (int i = 0; i < 5; ++i)
  //   for (int x = obs_left[i]; x < obs_right[i]; ++x)
  //     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
  //       grid[x][y] = 1;

  // // TEST 7
  // Node start(60, 2, 0, 0, 0, 0);
  // Node goal(60, 60, 0, 0, 0, 0);
  // int obs_left[]   = {  5,  5, 115,  60, 115 };
  // int obs_right[]  = { 10, 10, 120,  65, 120 };
  // int obs_top[]    = { 30, 60,  60,  30,  30 };
  // int obs_bottom[] = { 35, 65,  65,  35,  35 };
  // for (int i = 0; i < 5; ++i)
  //   for (int x = obs_left[i]; x < obs_right[i]; ++x)
  //     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
  //       grid[x][y] = 1;


  // // TEST 8
  // Node start(120, 120, 0, 0, 0, 0);
  // Node goal(65, 7, 0, 0, 0, 0);
  // int obs_left[]   = {115, 115,  90,  40,  70};
  // int obs_right[]  = {120, 120,  95,  45,  75};
  // int obs_top[]    = {  5,  60,  60,  40, 110};
  // int obs_bottom[] = { 10,  65,  65,  45, 115};
  // for (int i = 0; i < 5; ++i)
  //   for (int x = obs_left[i]; x < obs_right[i]; ++x)
  //     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
  //       grid[x][y] = 1;

  // // TEST 9
  // Node start(120, 60, 0, 0, 0, 0);
  // Node goal(10, 10, 0, 0, 0, 0);
  // int obs_left[]   = {100,  60,  80,  50,  20};
  // int obs_right[]  = {105,  65,  85,  55,  25};
  // int obs_top[]    = { 20,  80,  40,  10,  60};
  // int obs_bottom[] = { 25,  85,  45,  15,  65};
  // for (int i = 0; i < 5; ++i)
  //   for (int x = obs_left[i]; x < obs_right[i]; ++x)
  //     for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
  //       grid[x][y] = 1;

  //   // TEST 10
  Node start(10, 120, 0, 0, 0, 0);
  Node goal(10, 65, 0, 0, 0, 0);
  int obs_left[]   = {10, 10, 60, 40, 70};
  int obs_right[]  = {15, 15, 65, 45, 75};
  int obs_top[]    = {20, 90, 60, 40, 110};
  int obs_bottom[] = {25, 95, 65, 45, 115};
  for (int i = 0; i < 5; ++i)
    for (int x = obs_left[i]; x < obs_right[i]; ++x)
      for (int y = obs_top[i]; y < obs_bottom[i]; ++y)
        grid[x][y] = 1;

  /////////////////////////////////////////////////////////////////////

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.id_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = std::abs(start.x_ - goal.x_) + std::abs(start.y_ - goal.y_);

  // Make sure start and goal are not obstacles and their ids are correctly assigned.  
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  constexpr double threshold = 2.0;      
  constexpr int max_iter_x_factor = 2; // scales number of iterations based on grid area (max_iter_x_factor x n x n)

  RRTStar new_rrt_star(grid);
  new_rrt_star.SetParams(static_cast<int>(threshold), max_iter_x_factor);

  // make goal a 10x10 square 
  new_rrt_star.SetGoalRegionHalfSize(5);

  const auto [found_path, path_vector] = new_rrt_star.Plan(start, goal);

  if (!found_path) {
    std::cout << "No path found.\n";
    return 0;
  }

  std::cout << "Path found. Nodes in path: " << path_vector.size() << '\n';
  // std::cout << "Final cost: " << path_vector.back().cost_ << '\n';
  std::cout << "Final cost: " << path_vector.front().cost_ << '\n'; // goal cost
   
  // PrintPath(path_vector, start, goal, grid);
  // PrintGrid(grid);
  return 0;
}

#endif  // BUILD_INDIVIDUAL
