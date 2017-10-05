/*
 * pathplanner.h
 *
 * Created on: October 05, 2017
 * Author: Daniel Gattringer
 * Mail: daniel@gattringer.biz
 */

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <cmath>
#include <vector>

using namespace std;

class PathPlanner {
private:
  // Waypoints which are used for pathplanning
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  // Position, orientation and speed of the own car
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  
  // Actual lane of the own car
  int lane;
  
  // Reference velocity
  double ref_vel = 0.0;
  
  // Counter to manage DEBUG output
  int count;
  
  // Data from the sensor fusion modul (other cars on the track)
  vector<vector<double>> sensor_fusion;
  
  // Path data
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
  
  // Check if there is a car in front (on the same lane)
  bool CheckActualLane(vector<vector<double>> sensor_fusion, int prev_size);
  
  // Check if there is traffic on all lanes at the relevant position of the track
  // Before and behind the own car
  vector<bool> CheckAllLanes(vector<bool> lanes_change, double car_s, vector<vector<double>> sensor_fusion, int prev_size);
  
  // Decide which lane should be taken (depands on the actual lane and the traffic on the other lanes)
  void ChooseLaneToChange(vector<bool> lanes_change, double car_d);
  
  // Generate the next trajectory
  vector<double> GenerateNextPath(int prev_size);
  
public:
  PathPlanner(
     vector<double> map_waypoints_x,
     vector<double> map_waypoints_y,
     vector<double> map_waypoints_s,
     vector<double> map_waypoints_dx,
     vector<double> map_waypoints_dy);
  
  virtual ~PathPlanner();
  
  // Solves the pathplanning problem
  // Usage of data from the own car, other traffic
  // Delivers the future path for the car
  vector<double> SolvePath(vector<double> car_data,
                       vector<vector<double>> sensor_fusion,
                       vector<double> previous_path_x,
                       vector<double> previous_path_y,
                       double end_path_s,
                       double end_path_d);
};

#endif /* PATHPLANNER_H */
