#include "phy_simulator/arena_loader.h"

using std::vector;
using std::string;

namespace phy_simulator {

using Json = nlohmann::json;

ArenaLoader::ArenaLoader() {}

ArenaLoader::ArenaLoader(const std::string &vehicle_set_path,
                         const std::string &map_path,
                         const std::string &lane_net_path)
    : vehicle_set_path_(vehicle_set_path),
      map_path_(map_path),
      lane_net_path_(lane_net_path) {}

bool ArenaLoader::ParseVehicleSet(common::VehicleSet *p_vehicle_set, 
                const vector<double> &ego, 
                const vector<vector<double>> &agents) {
  printf("\n[ArenaLoader] Loading vehicle set\n");

  std::fstream fs(vehicle_set_path_);
  Json root;
  fs >> root;

  Json vehicles_json = root["vehicles"];
  Json info_json = vehicles_json["info"];
  // loading
  for (int j = 0; j <= static_cast<int>(agents.size()); ++j) {
    int i;
    if (j < static_cast<int>(agents.size())) {i = 1;}
    else {i = 0;}
    common::Vehicle vehicle;
    vehicle.set_id(j);
    vehicle.set_subclass(info_json[i]["subclass"].get<std::string>());
    vehicle.set_type(info_json[i]["type"].get<std::string>());

    Json state_json = info_json[i]["init_state"];
    common::State state;
    if (i == 0) {
      state.vec_position(0) = ego[0];
      state.vec_position(1) = ego[1];
      state.angle = ego[2];
      state.velocity = 0;
    } else {
      state.vec_position(0) = agents[j][0];
      state.vec_position(1) = agents[j][1];
      state.angle = agents[j][2];
      state.velocity = sqrt(pow(agents[j][3], 2) + pow(agents[j][4], 2));
    }
    state.curvature = state_json["curvature"].get<double>();
    state.acceleration = state_json["acceleration"].get<double>();
    state.steer = state_json["steer"].get<double>();
    vehicle.set_state(state);

    Json params_json = info_json[i]["params"];
    common::VehicleParam param;
    if (i == 0) {
      param.set_width(ego[3]);
      param.set_length(ego[4]);
    } else {
      param.set_width(agents[j][6]);
      param.set_length(agents[j][7]);
    }
    param.set_wheel_base(params_json["wheel_base"].get<double>());
    param.set_front_suspension(params_json["front_suspension"].get<double>());
    param.set_rear_suspension(params_json["rear_suspension"].get<double>());
    auto max_steering_angle = params_json["max_steering_angle"].get<double>();
    param.set_max_steering_angle(max_steering_angle * kPi / 180.0);
    param.set_max_longitudinal_acc(
        params_json["max_longitudinal_acc"].get<double>());
    param.set_max_lateral_acc(params_json["max_lateral_acc"].get<double>());

    param.set_d_cr(param.length() / 2 - param.rear_suspension());
    vehicle.set_param(param);

    p_vehicle_set->vehicles.insert(
        std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
  }
  // p_vehicle_set->print();

  fs.close();
  return true;
}

ErrorType ArenaLoader::ParseMapInfo(common::ObstacleSet *p_obstacle_set,
      const vector<vector<double>> &obstacles) {
  printf("\n[ArenaLoader] Loading map info\n");

  std::fstream fs(map_path_);
  Json root;
  fs >> root;

  Json obstacles_json = root["features"];
  for (int i = 0; i < static_cast<int>(obstacles.size()); ++i) {
    int j;
    if (i >= obstacles_json.size()) j = obstacles_json.size() - 1;
    else j = i;
    Json obs = obstacles_json[j];
    // printf("Obstacle id %d.\n", obs["properties"]["id"].get<int>());

    common::PolygonObstacle poly;
    poly.id = i;
    if (i < obstacles.size() - 1) poly.type = 0;
    else poly.type = 1;
    int num_pts = static_cast<int>(obstacles[i].size());
    for (int k = 0; k < num_pts; ++k) {
      common::Point point(obstacles[i][0], obstacles[i][1]);
      poly.polygon.points.push_back(point);
    }
    p_obstacle_set->obs_polygon.insert(
        std::pair<int, common::PolygonObstacle>(poly.id, poly));
  }
  // p_obstacle_set->print();

  fs.close();
  return kSuccess;
}

ErrorType ArenaLoader::ParseLaneNetInfo(common::LaneNet *p_lane_net,
                const vector<int> &lanes_id, 
                const vector<double> &lanes_length, 
                const vector<vector<vector<double>>> &points,
                const vector<vector<int>> &pre_connections,
                const vector<vector<int>> &nxt_connections,
                const vector<vector<int>> &left_connection,
                const vector<vector<int>> &right_connection) {
  printf("\n[ArenaLoader] Loading lane net info\n");

  std::fstream fs(lane_net_path_);
  Json root;
  fs >> root;

  Json lane_net_json = root["features"];

  for (int i = 0; i < static_cast<int>(lanes_id.size()); ++i) {
    common::LaneRaw lane_raw;
    int j;
    if (i >= lane_net_json.size()) j = lane_net_json.size() - 1;
    else j = i;
    Json lane_json = lane_net_json[j];
    Json lane_meta = lane_json["properties"];

    lane_raw.id = i;
    lane_raw.length = lanes_length[i];
    lane_raw.dir = 1;

    // printf("-Lane id %d.\n", lane_raw.id);
    if (i < lanes_id.size() - 1) {
      for (const auto lane_id : nxt_connections[i]) {
        if (lane_id != 0) lane_raw.child_id.push_back(lane_id);
      }
    }

    if (i > 0) {
      for (const auto &lane_id : pre_connections[i]) {
        if (lane_id != 0) lane_raw.father_id.push_back(lane_id);
      }
    }

    int lchg_vld = 0;
    int rchg_vld = 0;
    for (const auto &lane_id : left_connection[i]) {
      lane_raw.l_lane_id = lane_id;
      lchg_vld = 1;
    }
    for (const auto &lane_id : right_connection[i]) {
      lane_raw.r_lane_id = lane_id;
      rchg_vld = 1;
    }

    lane_raw.l_change_avbl = static_cast<bool>(lchg_vld);
    lane_raw.r_change_avbl = static_cast<bool>(rchg_vld);
    lane_raw.behavior = "s";

    for (int k = 0; k < points[i].size(); ++k) {
      Vec2f pt(points[i][k][0], points[i][k][1]);
      lane_raw.lane_points.emplace_back(pt);
    }
    lane_raw.start_point = *(lane_raw.lane_points.begin());
    lane_raw.final_point = *(lane_raw.lane_points.rbegin());
    p_lane_net->lane_set.insert(
        std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
  }
  // p_lane_net->print();

  fs.close();
  return kSuccess;
}

}  // namespace phy_simulator
