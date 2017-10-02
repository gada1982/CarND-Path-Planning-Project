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
  
  vector<vector<double>> sensor_fusion;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
  
  int car_lane;
  double car_ref_vel = 0.0;
  
public:
  PathPlanner(
     vector<double> map_waypoints_x,
     vector<double> map_waypoints_y,
     vector<double> map_waypoints_s,
     vector<double> map_waypoints_dx,
     vector<double> map_waypoints_dy);
  
  virtual ~PathPlanner();
  
  // Solve for the path plan, given a bunch of information about the car and enviroment
  // Returns the path
  vector<double> SolvePath(vector<double> car_data,
                       vector<vector<double>> sensor_fusion,
                       vector<double> previous_path_x,
                       vector<double> previous_path_y,
                       double end_path_s,
                       double end_path_d);
};

#endif /* PATHPLANNER_H */
