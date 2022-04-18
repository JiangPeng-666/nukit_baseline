/**
 * @file solver.cc
 * @author  
 * @brief the solver of ego vehicle
 * @version 0.1
 * @date 2022-02
 * @copyright Copyright (c) 2022
 */

#include "solver.h"

DECLARE_BACKWARD;
using std::string;
using std::vector;
using std::cout;
using std::endl;

namespace planning{

  Solver::Solver() {};

  vector<vector<double>> Solver::solver(vector<int> &lanes_id, vector<double> &lanes_length,
  vector<vector<vector<double>>> &points, vector<vector<int>> &pre_connections, 
  vector<vector<int>> &nxt_connnections, vector<vector<int>> &left_connnection, 
  vector<vector<int>> &right_connnection, vector<double> &ego, 
  vector<vector<double>> &agents, vector<vector<double>> &obstacles) {

   // path to config
   // 1. for the convenience of working in VS Code, relative path is deprecated here
  // char pwd[255];
  // std::string s_tmp;
  // s_tmp = getcwd(pwd, 255);
  // cout << "Current path is: "<< s_tmp << endl;
  // int cur = s_tmp.size() - 1;
  // while(pwd[cur] != '/') {cur--;}
  // s_tmp = s_tmp.substr(0, cur + 1);
  // 2. and we take the absulte path here
  string s_tmp;
  s_tmp = "/home/lain/jiangpeng/nukit_baseline/";

  string pwd_cfg = s_tmp + "config/";
  string pwd_core = s_tmp + "core/";
  agent_config_path = pwd_cfg + "highway_v1.0/agent_config.json";
  agent_config_path = pwd_cfg + "highway_v1.0/agent_config.json";
  vehicle_info_path = pwd_cfg + "highway_v1.0/vehicle_set.json";
  map_path = pwd_cfg + "highway_v1.0/obstacles_norm.json";
  lane_net_path = pwd_cfg + "highway_v1.0/lane_net_norm.json";
  bp_config_path = pwd_core + "eudm_planner/config/eudm_config.pb.txt";
  ssc_config_path = pwd_core + "ssc_planner/config/ssc_config.pb.txt"; 

  // Initialization
  phy_simulator::PhySimulation phy_sim(vehicle_info_path, map_path, lane_net_path, 
        lanes_id, lanes_length, points, pre_connections, nxt_connnections, left_connnection,
        right_connnection, ego, agents, obstacles);
  semantic_map_manager::SemanticMapManager smm_(ego_id, agent_config_path);
  p_data_renderer_ = new semantic_map_manager::DataRenderer(&smm_);
  p_data_renderer_->Render(smm_.time_stamp(), phy_sim.lane_net(), phy_sim.vehicle_set(), 
                    phy_sim.obstacle_set());
  bp_manager_.Init(bp_config_path);
  ssc_planner_.Init(ssc_config_path);

  // behavior planning
  printf("Behavior planning begins.------------------------\n");
  auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);
  task_.user_perferred_behavior = 0;
  task_.user_desired_vel = 20;
  task_.is_under_ctrl = true;
  vector<double> terminate = bp_manager_.Run(smm_.time_stamp(), map_ptr, task_);
  cout << "terminate(x, y): " << terminate[0] << terminate[1] << endl;
  common::SemanticBehavior behavior;
  bp_manager_.ConstructBehavior(&behavior);
  smm_.set_ego_behavior(behavior);
  printf("Behavior planning finished.\n");

  // motion planning
  printf("Motion planning begins.--------------------------\n");
  map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);
  map_adapter_.set_map(map_ptr);
  ssc_planner_.set_map_interface(&map_adapter_);
  ssc_planner_.RunOnce();
  next_traj_ = std::move(ssc_planner_.trajectory());
  next_traj_->GetState(t, &desired_state);
  printf("Motion planning finished.\n");

  // result
  vector<vector<double>> ego_traj;
  int cnt = 0;
  while (std::pow((terminate[0] - desired_state.vec_position[0]), 2) + 
      std::pow((terminate[1] - desired_state.vec_position[1]), 2) > 0.001) {
      t = global_init_stamp_ + plan_horizon * (cnt++);
      next_traj_->GetState(t, &desired_state);
      vector<double> res;
      res.push_back(desired_state.vec_position[0]);//x
      res.push_back(desired_state.vec_position[1]);//y
      res.push_back(desired_state.velocity);//v
      res.push_back(desired_state.acceleration);//a
      res.push_back(desired_state.angle);//theta
      
      ego_traj.push_back(res);
    }
  cout << "The size of ego_traj: " << ego_traj.size() << endl;

  return ego_traj;
  }
}

