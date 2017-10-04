#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class PathPlanner {
private:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  int count;
  
  vector<vector<double>> sensor_fusion;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
  
  int lane;
  double ref_vel = 0.0;
  bool CheckActualLane(vector<vector<double>> sensor_fusion, int prev_size);
  
  vector<bool> CheckAllLanes(vector<bool> lanes_change, double car_s, vector<vector<double>> sensor_fusion, int prev_size);
  
  bool ChooseLaneToChange(vector<bool> lanes_change, double car_d);
  
public:
  PathPlanner(
     vector<double> map_waypoints_x,
     vector<double> map_waypoints_y,
     vector<double> map_waypoints_s,
     vector<double> map_waypoints_dx,
     vector<double> map_waypoints_dy);
  
  virtual ~PathPlanner();
  
  // TODO
  vector<double> SolvePath(vector<double> car_data,
                       vector<vector<double>> sensor_fusion,
                       vector<double> previous_path_x,
                       vector<double> previous_path_y,
                       double end_path_s,
                       double end_path_d);
  
  // Generates the next path
  vector<double> GenerateNextPath(int prev_size);
};

#endif /* PATHPLANNER_H */
