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

namespace planning{
  std::vector<std::vector<double>> Solver::solver(int ego_id = 0, 
    std::string agent_config_path = "/home/jp/ego_solver/config/highway_v1.0/agent_config.json",
    std::string bp_config_path = "/home/jp/ego_solver/core/eudm_planner/config/eudm_config.pb.txt", 
    std::string ssc_config_path = "/home/jp/ego_solver/core/ssc_planner/config/ssc_config.pb.txt", 
    std::string vehicle_info_path = "/home/jp/ego_solver/config/highway_v1.0/vehicle_set.json",
    std::string map_path = "/home/jp/ego_solver/config/highway_v1.0/obstacles_norm.json",
    std::string lane_net_path = "/home/jp/ego_solver/config/highway_v1.0/lane_net_norm.json",
    double desired_vel = 20.0) {

  //TODO: replace the hard coding above
  // char pwd[255];
  // getcwd(pwd, 255);
  // std::cout << "dir is " << pwd << std::endl;
 
  // Initialization
  phy_simulator::PhySimulation phy_sim(vehicle_info_path, map_path, lane_net_path);
  semantic_map_manager::SemanticMapManager smm_(ego_id, agent_config_path);
  p_data_renderer_ = new semantic_map_manager::DataRenderer(&smm_);
  p_data_renderer_->Render(smm_.time_stamp(), phy_sim.lane_net(), phy_sim.vehicle_set(), 
                    phy_sim.obstacle_set());
  bp_manager_.Init(bp_config_path);
  ssc_planner_.Init(ssc_config_path);
  auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);
  map_adapter_.set_map(map_ptr);
  ssc_planner_.set_map_interface(&map_adapter_);

  // behavior planning
  printf("Behavior planning begins.------------------------\n");
  task_.user_perferred_behavior = 0;
  task_.user_desired_vel = desired_vel;
  task_.is_under_ctrl = true;
  bp_manager_.Run(smm_.time_stamp(), map_ptr, task_);
  common::SemanticBehavior behavior;
  bp_manager_.ConstructBehavior(&behavior);
  smm_.set_ego_behavior(behavior);
  printf("Behavior planning finished.\n");

  // motion planning
  printf("Motion planning begins.--------------------------\n");
  map_adapter_.set_map(map_ptr);
  ssc_planner_.set_map_interface(&map_adapter_);
  ssc_planner_.RunOnce();
  next_traj_ = std::move(ssc_planner_.trajectory());
  t = smm_.time_stamp();
  next_traj_->GetState(t, &desired_state);
  printf(
      "[SscPlannerServer] desired state (x,y,v,a,theta):(%lf, %lf, %lf, "
      "%lf, %lf).\n",
      desired_state.vec_position[0], desired_state.vec_position[1],
      desired_state.velocity, desired_state.acceleration, desired_state.angle);
  printf("Motion planning finished.\n");

  // result
  std::vector<double> res;
  res.push_back(desired_state.vec_position[0]);//x
  res.push_back(desired_state.vec_position[1]);//y
  res.push_back(desired_state.velocity);//v
  res.push_back(desired_state.acceleration);//a
  res.push_back(desired_state.angle);//theta
  std::vector<std::vector<double>> ego_traj;
  ego_traj.push_back(res);
  return ego_traj;

  }
}
int main(int argc, char **argv){
    planning::Solver solver;
    std::vector<std::vector<double>> ego_traj = solver.solver();
    // for (int i = 0; i < ego_traj.size(); i++) {
    //   std::cout << ego_traj[i] << endl;
    // }
    return 0;
  }


