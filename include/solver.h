#ifndef _CORE_SOLVER_H__
#define _CORE_SOLVER_H__

#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <functional>
#include <numeric>
#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <string>


#include "common/basics/tic_toc.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/data_renderer.h"
#include "ssc_planner/map_adapter.h"
#include "ssc_planner/ssc_planner.h"
#include "phy_simulator/basics.h"
#include "phy_simulator/phy_simulator.h"


namespace planning{
class Solver{
public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using FrenetTrajectory = common::FrenetTrajectory;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;

  Solver();

  std::vector<std::vector<double>> solver(std::vector<std::vector<double>> lanes,
    std::vector<std::vector<int>> connections, std::vector<double> ego,
    std::vector<std::vector<double>> agents);

  // lanes: (num * 2)
  // connections (num * 2)
  // ego: ego_pose = ego[:3] # (1*3) x, y, heading
  //      ego_size = ego[3:] # (1*2) width, length
  // agents: agent_pose = (agents[:,0:3]) # (num * 3) x, y, heading
  //         agents_velocity = (agents[:,3 : 5]) # (num * 2) x, y
  //         agents_size = (agents[:,6:]) # (num * 2)

private:

  // path
  std::string agent_config_path;
  std::string vehicle_info_path;
  std::string map_path;
  std::string lane_net_path;
  std::string ssc_config_path;
  std::string bp_config_path;

  //sementic map
  semantic_map_manager::DataRenderer* p_data_renderer_;
  int ego_id = 0;

  // ssc
  decimal_t work_rate_ = 20.0;
  decimal_t plan_horizon = 1.0 / work_rate_;
  common::State desired_state;
  decimal_t t = 0.0;
  decimal_t global_init_stamp_{0.0};
  SscPlanner ssc_planner_;
  SscPlannerAdapter map_adapter_;
  vec_E<common::State> desired_state_hist_;
  std::unique_ptr<FrenetTrajectory> next_traj_;

  // eudm
  EudmManager bp_manager_;
  SemanticMapManager smm_;
  planning::eudm::Task task_;
  bool use_sim_state_ = true;

};
typedef std::shared_ptr<Solver> SolverPtr;
}// namespace planning
#endif


