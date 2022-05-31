#include "phy_simulator/phy_simulator.h"
using std::vector;
using std::string;

namespace phy_simulator {

PhySimulation::PhySimulation() {
  p_arena_loader_ = new ArenaLoader();
  // if (!GetDataFromArenaLoader()) assert(false);
}

PhySimulation::PhySimulation(const string &vehicle_set_path,
                             const string &map_path,
                             const string &lane_net_path,
                             const vector<int> &lanes_id,
                             const vector<double> &lanes_length,
                             const vector<vector<vector<double>>> &points, 
                             const vector<vector<int>> &pre_connections,
                             const vector<vector<int>> &nxt_connections,
                             const vector<vector<int>> &left_connection,
                             const vector<vector<int>> &right_connection,
                             const vector<double> &ego, 
                             const vector<vector<double>> &agents,
                             const vector<vector<double>> &obstacles) {
  std::cout << "[PhySimulation] Constructing..." << std::endl;
  p_arena_loader_ = new ArenaLoader();

  p_arena_loader_->set_vehicle_set_path(vehicle_set_path);
  p_arena_loader_->set_map_path(map_path);
  p_arena_loader_->set_lane_net_path(lane_net_path);

  GetDataFromArenaLoader(lanes_id, lanes_length, points, pre_connections,
    nxt_connections, left_connection, right_connection, ego, agents, obstacles);
  SetupVehicleModelForVehicleSet();
}

bool PhySimulation::GetDataFromArenaLoader(const vector<int> &lanes_id,
                             const vector<double> &lanes_length,
                             const vector<vector<vector<double>>> &points, 
                             const vector<vector<int>> &pre_connections,
                             const vector<vector<int>> &nxt_connections,
                             const vector<vector<int>> &left_connection,
                             const vector<vector<int>> &right_connection,
                             const vector<double> &ego, 
                             const vector<vector<double>> &agents,
                             const vector<vector<double>> &obstacles) {
  std::cout << "[PhySimulation] Parsing simulation info..." << std::endl;
  p_arena_loader_->ParseVehicleSet(&vehicle_set_, ego, agents);
  p_arena_loader_->ParseMapInfo(&obstacle_set_, obstacles);
  p_arena_loader_->ParseLaneNetInfo(&lane_net_, lanes_id, lanes_length, points, 
    pre_connections, nxt_connections, left_connection, right_connection);
  return true;
}

bool PhySimulation::SetupVehicleModelForVehicleSet() {
  for (const auto &p : vehicle_set_.vehicles) {
    simulator::VehicleModel vehicle_model(
        p.second.param().wheel_base(), p.second.param().max_steering_angle());
    vehicle_model.set_state(p.second.state());
    vehicle_model_set_.insert(
        std::pair<int, simulator::VehicleModel>(p.first, vehicle_model));

    vehicle_ids_.push_back(p.first);
  }
  return true;
}

bool PhySimulation::UpdateSimulatorUsingSignalSet(
    const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
  TicToc updata_vehicle_time;
  UpdateVehicleStates(signal_set, dt);
  return true;
}

bool PhySimulation::UpdateVehicleStates(
    const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
  if (signal_set.signal_set.size() != vehicle_set_.vehicles.size()) {
    std::cerr << "[PhySimulation] ERROR - Signal number error." << std::endl;
    std::cerr << "[PhySimulation] signal_set num: "
              << signal_set.signal_set.size()
              << ", vehicle_set num: " << vehicle_set_.vehicles.size()
              << std::endl;
    assert(false);
  }
  for (auto iter = vehicle_set_.vehicles.begin();
       iter != vehicle_set_.vehicles.end(); ++iter) {
    int id = iter->first;
    common::VehicleControlSignal signal = signal_set.signal_set.at(id);
    decimal_t steer_rate = signal.steer_rate;
    decimal_t acc = signal.acc;

    auto model_iter = vehicle_model_set_.find(id);

    if (!signal.is_openloop) {
      model_iter->second.set_control(
          simulator::VehicleModel::Control(steer_rate, acc));
      model_iter->second.Step(dt);
    } else {
      model_iter->second.set_state(signal.state);
    }
    iter->second.set_state(model_iter->second.state());
  }
  return true;
}

}  // namespace phy_simulator