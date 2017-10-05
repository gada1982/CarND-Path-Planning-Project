/*
 * pathplanner.cpp
 *
 * Created on: October 05, 2017
 * Author: Daniel Gattringer
 * Mail: daniel@gattringer.biz
 */

#include <cmath>
#include <vector>
#include <iostream>
#include "spline.h"
#include "helper.h"
#include "pathplanner.h"

using namespace std;

// Pathplanner class definition
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
  
  // Init counter for DEBUG-Issues
  count = 0;
}

PathPlanner::~PathPlanner() {}

// Check if there is a car in front (on the same lane)
bool PathPlanner::CheckActualLane(vector<vector<double>> sensor_fusion, int prev_size)
{
  bool too_close = false;
  bool debug = true;
  
  // Don't get closer to the car in front then this amount
  // Brake or change lane if violeted
  int safety_distance = 20;
  
  // Use sensor fusion data to check if there is a slower car in front (on the same lane)
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    float d = sensor_fusion[i][6];
    
    // Check only the own lane
    if(d <= (2+4*lane+2) && d >= (2+4*lane-2))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(pow(vx,2) + pow(vy,2));
      double check_car_s = sensor_fusion[i][5];
      
      // Be aware of latency (path is already old)
      check_car_s += (double)prev_size*0.02*check_speed;
      
      // Check if there is a car in front of me within the given Distance
      if((check_car_s > car_s) && ((check_car_s - car_s) < safety_distance))
      {
        // There is a car to close in front
        too_close = true;
        
        // Print some screen information
        if(debug && count > 20)
        {
          cout << "\nToo close to vehicle in front!";
        }
      }
    }
  }
  return too_close;
}

// Check if there is traffic on all lanes at the relevant position of the track
// Before and behind the own car
vector<bool> PathPlanner::CheckAllLanes(vector<bool> lanes_change, double car_s, vector<vector<double>> sensor_fusion, int prev_size)
{
  float d;
  bool debug = true;
  
  // Changing lane is only safe if there is enough free space
  // In front of the car and behind the car
  int safety_distance = 20;
  
  // Find rev_v to use
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    d = sensor_fusion[i][6];
    
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(pow(vx,2) + pow(vy,2));
    double check_car_s = sensor_fusion[i][5];
    
    // Be aware of latency (path is already old)
    check_car_s += (double)prev_size*0.02*check_speed;
    
    // Check if there is a car within the safety distance
    // In front of the car or behind the car
    if(abs(car_s - check_car_s) < safety_distance)
    {
      // Check if lane 0 (most left lane) is free
      if(d >= 0 && d < 4)
      {
        // Lane 0 is not free
        lanes_change[0] = false;
        
        // Print some screen information
        if(debug && count > 20)
        {
          cout << "\nLane 0 is not free!";
        }
      }
      // Check if lane 1 (middle lane) is free
      else if(d >= 4 && d <= 8)
      {
        // Lane 1 is not free
        lanes_change[1] = false;
        
        // Print some screen information
        if(debug && count > 20)
        {
          cout << "\nLane 1 is not free!";
        }
      }
      // Check if lane 2 (most right lane) is free
      else
      {
        // Lane 2 is not free
        lanes_change[2] = false;
        
        // Print some screen information
        if(debug && count > 20)
        {
          cout << "\nLane 2 is not free!";
        }
      }
    }
  }
  
  return lanes_change;
}

// Decide which lane should be taken (depands on the actual lane and the traffic on the other lanes)
void PathPlanner::ChooseLaneToChange(vector<bool> lanes_change, double car_d)
{
  bool debug = false;
  
  // Check if the own car is in the lane it should be -> Lane change is finished
  // Don't change lane again before the prior lane change is finished
  if(car_d <= (2+4*lane+2) && car_d >= (2+4*lane-2))
  {
    // The own car is at lane 0 (most left) or lane 2 (most right)
    // Check if lane 1 (middle lane) is free
    // If yes change to lane 1 (middle lane)
    if((lane == 0 && lanes_change[1]) || (lane == 2 && lanes_change[1]))
    {
      lane = 1;
      
      // Print some screen information
      if(debug && count > 20)
      {
        cout << "\nChange to lane 1!";
      }
    }
    // The own car is at lane 1 (middle lane)
    else if(lane == 1)
    {
      // Check if lane 0 (most left lane) is free
      // If yes change to lane 0 (most left  lane)
      if(lanes_change[0])
      {
        lane = 0;
        
        // Print some screen information
        if(debug && count > 20)
        {
          cout << "\nChange to lane 0!";
        }
      }
      // If lane 0 (most left lane) is not free, check if lane 2 (most right lane) is free
      // If yes change to lane 2 (most right  lane)
      else if(lanes_change[2])
      {
        lane = 2;
        
        // Print some screen information
        if(debug && count > 20)
        {
          cout << "\nChange to lane 2!";
        }
      }
    }
  }
}

// Solves the pathplanning problem
// Usage of data from the own car, other traffic
// Delivers the future path for the car
vector<double> PathPlanner::SolvePath(vector<double> car_data, vector<vector<double>> sensor_fusion,
                         vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d)
{
  bool debug = true;
  
  // Position, orientation and speed of the own car
  car_x = car_data[0];
  car_y = car_data[1];
  car_s = car_data[2];
  car_d = car_data[3];
  car_yaw = car_data[4];
  car_speed = car_data[5];
  
  // Data from the sensor fusion modul (other cars on the track)
  this->sensor_fusion = sensor_fusion;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->end_path_s = end_path_s;
  this->end_path_d = end_path_d;
  
  // Speed limit and change rate of the velocity
  float speed_limit = 49.5;
  float change_rate_speed = 0.224;
  
  vector<double> new_path;
  
  // Size of the previous path
  int prev_size = previous_path_x.size();
  
  count += 1;
  if(debug && count > 50)
  {
    count = 0;
    cout << "\nOwn car s: " << car_s;
  }
  
  // Start Sensor Fusion Part
  if(prev_size > 0)
  {
    car_s = end_path_s;
  }
  
  // Check if actual lane is free or if it is occupied by a slower car
  bool too_close = false;
  too_close = CheckActualLane(sensor_fusion, prev_size);
  
  // Check if the other lanes are free
  // This is only done when the own lane is occupied by a slower car
  vector<bool> lanes_change = {true, true, true};
  if(too_close)
  {
    lanes_change = CheckAllLanes(lanes_change, car_s, sensor_fusion, prev_size);
  }
  
  // Decide if a lane change is possible and if yes to which lane
  // This is only done when the own lane is occupied by a slower car
  if(too_close)
  {
    ChooseLaneToChange(lanes_change, car_d);
  }
    
  // Adjust velocity
  // If the own lane is occupied by a slower car -> brake
  if(too_close)
  {
    ref_vel -= change_rate_speed;
  }
  // If the own lane is NOT occupied by a slower car and the own car has not reached the speed limit -> accelerate
  else if(ref_vel < speed_limit)
  {
    ref_vel += change_rate_speed;
  }
  
  // Get the new trajectory, which the car should follow in the next cycle
  new_path = GenerateNextPath(prev_size);
  
  return new_path;
  
}
  
// Generate the next trajectory
vector<double> PathPlanner::GenerateNextPath(int prev_size)
{
  vector<double> new_path;
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
  // Previous path exists -> use it to get a smoother transition
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

  // Add evenly 30m spaced points (30m ,60m, 90m) ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Do some transformation of the coordinates to make calculation easier
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

  // Fill up the rest of the spline to get a smooth movement
  for(int i = 1; i <= 50 - previous_path_x.size(); i++)
  {
    double N = target_dist / (0.02 * ref_vel/2.24); // mph->m/s
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);
    
    x_add_on  = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // Rotate back to original coordinate system
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
    
    x_point += ref_x;
    y_point += ref_y;
    
    new_path.push_back(x_point);
    new_path.push_back(y_point);
  }

  return new_path;
}
