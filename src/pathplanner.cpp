#include <cmath>
#include <vector>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "helper.h"
#include "pathplanner.h"


// Path Planner class definition
PathPlanner::PathPlanner(vector<double> map_waypoints_x, vector<double> map_waypoints_y,
       vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy)
{
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_dx = map_waypoints_dx;
  this->map_waypoints_dy = map_waypoints_dy;
  
  // Reference velocity (mph)
  ref_vel = 0.0;
  
  // Start in lane 1
  lane = 1;
}

PathPlanner::~PathPlanner() {}

vector<double> PathPlanner::SolvePath(vector<double> car_data, vector<vector<double>> sensor_fusion,
                         vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d)
{
  car_x = car_data[0];
  car_y = car_data[1];
  car_s = car_data[2];
  car_d = car_data[3];
  car_yaw = car_data[4];
  car_speed = car_data[5];
  
  this->sensor_fusion = sensor_fusion;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->end_path_s = end_path_s;
  this->end_path_d = end_path_d;
  
  //impacts default behavior for most states
  float SPEED_LIMIT = 49.5;
  float MAX_ACCEL = 0.224;
  
  vector<double> new_path;
  
  // Size of the previous path
  int prev_size = previous_path_x.size();
  
  // Start Sensor Fusion Part
  if(prev_size > 0)
  {
    this->car_s = this->end_path_s;
  }
  
  bool too_close = false;
  
  // Find rev_v to use
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    // Is a car in my lane?
    float d = sensor_fusion[i][6];
    // Not off the road
    if(d < (2+4*lane+2) && d > (2+4*lane-2)) // TODO - Is a car in my lane
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(pow(vx,2) + pow(vy,2));
      double check_car_s = sensor_fusion[i][5];
      
      check_car_s += (double)prev_size*0.02*check_speed;
      // Check if s values are greater than mine and s gap
      if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
      {
        // TODO do something more sophisticated
        too_close = true;
        
        if(lane > 0)
        {
          lane = 0;
        }
      }
    }
  }
  
  // End Sensor Fusion Part
    
  // TODO optimise -> Video bei ca. 51:00
  if(too_close)
  {
    ref_vel -= MAX_ACCEL; // TODO
  }
  else if(ref_vel < SPEED_LIMIT)
  {
    //ref_vel += MAX_ACCEL;
    ref_vel += MAX_ACCEL;
  }
  
  
    
  vector<double> ptsx;
  vector<double> ptsy;
  
  // Reference state (x, y, yaw)
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  
  // For small pref_size use car as starting point
  if(prev_size < 2)
  {
    // Usage of two points to get a tangent for the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  // Previous path exists -> use it
  else
  {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    // Usage of two points to get a tangent for the car
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  // Add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  for(int i = 0; i < ptsx.size(); i++)
  {
    // Shift car reference angle to 0 degree
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    
    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }
  
  // Create a spline to follow
  tk::spline s;
  
  // Set (x,y) points to the spline
  s.set_points(ptsx, ptsy);
  
  // Start with all the prev path points
  for(int i = 0; i < previous_path_x.size(); i++)
  {
    new_path.push_back(previous_path_x[i]);
    new_path.push_back(previous_path_y[i]);
  }
  
  // Calculate spline
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
  
  double x_add_on = 0;
  
  // Fill up the rest
  for(int i = 1; i <= 50 - previous_path_x.size(); i++)
  {
    double N = target_dist / (0.02 * ref_vel/2.24); // mph->m/s
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);
    
    x_add_on  = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // Rotate back
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
    
    x_point += ref_x;
    y_point += ref_y;
    
    new_path.push_back(x_point);
    new_path.push_back(y_point);
  }
  
  return new_path;
}
