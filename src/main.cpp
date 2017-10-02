#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;
  
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  //impacts default behavior for most states
  float SPEED_LIMIT = 49.5;
  float MAX_ACCEL = 0.224;
  
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  
  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  // Start in lane 1
  int lane = 1;
  
  // Reference velocity (mph)
  double ref_vel = 0.0;
  
 h.onMessage([&MAX_ACCEL, &SPEED_LIMIT,&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                        uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      
      auto s = hasData(data);
      
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          // Size of the previous path
          int prev_size = previous_path_x.size();
          
          json msgJson;
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          
          bool followLine1 = false;
          if(followLine1)
          {
            double dist_inc = 0.5;
            for(int i = 0; i < 50; i++)
            {
              double next_s = car_s + (i+1)*dist_inc;
              double next_d = 6;
              
              vector <double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
            }
          }
          
          bool followSpline1 = true;
          if(followSpline1)
          {
            
            
            // Start Sensor Fusion Part
            if(prev_size > 0)
            {
              car_s = end_path_s;
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
            
            // TODO optimise -> Video bei ca. 51:00
            if(too_close)
            {
              ref_vel -= MAX_ACCEL; // TODO
            }
            else if(ref_vel < SPEED_LIMIT)
            {
              ref_vel += MAX_ACCEL;
            }
            
            
            
            
            
            // End Sensor Fusion Part
            
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
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
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
              
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
            
          }
          
          bool simpleGoCircle = false;
          if(followLine1 == false && followSpline1 == false && simpleGoCircle == true)
          {
            double pos_x;
            double pos_y;
            double angle;
            int path_size = previous_path_x.size();
            
            for(int i = 0; i < path_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            
            if(path_size == 0)
            {
              pos_x = car_x;
              pos_y = car_y;
              angle = deg2rad(car_yaw);
            }
            else
            {
              pos_x = previous_path_x[path_size-1];
              pos_y = previous_path_y[path_size-1];
              
              double pos_x2 = previous_path_x[path_size-2];
              double pos_y2 = previous_path_y[path_size-2];
              angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            }
            
            double dist_inc = 0.5;
            for(int i = 0; i < 50-path_size; i++)
            {
              next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
              next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
              pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
              pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
            }
          }
          
          
          // End of TODO
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          
          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
  
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
  
  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
