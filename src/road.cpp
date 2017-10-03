#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Road
 */
Road::Road(int speed_limit, double traffic_density, vector<int> lane_speeds) {
  
  this->num_lanes = lane_speeds.size();
  this->lane_speeds = lane_speeds;
  this->speed_limit = speed_limit;
  this->density = traffic_density;
  this->camera_center = this->update_width/2;
  
}

Road::~Road() {}

Vehicle Road::get_ego() {
  
  return this->vehicles.find(this->ego_key)->second;
}

void Road::populate_traffic() {
  
  int start_s = max(this->camera_center - (this->update_width/2), 0);
  for (int l = 0; l < this->num_lanes; l++)
  {
    int lane_speed = this->lane_speeds[l];
    bool vehicle_just_added = false;
    for(int s = start_s; s < start_s+this->update_width; s++)
    {
      
      if(vehicle_just_added)
      {
        vehicle_just_added = false;
      }
      if(((double) rand() / (RAND_MAX)) < this->density)
      {
        
        Vehicle vehicle = Vehicle(l,s,lane_speed,0);
        vehicle.state = "CS";
        this->vehicles_added += 1;
        this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
        vehicle_just_added = true;
      }
    }
  }
  
}

void Road::advance() {
  
  map<int ,vector<vector<int> > > predictions;
  
  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
    vector<vector<int> > preds = it->second.generate_predictions(10);
    predictions[v_id] = preds;
    it++;
  }
  it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
    if(v_id == ego_key)
    {
      it->second.update_state(predictions);
      it->second.realize_state(predictions);
    }
    it->second.increment(1);
    
    it++;
  }
  
}

void Road::add_ego(int lane_num, int s, vector<int> config_data) {
  
  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
    Vehicle v = it->second;
    if(v.lane == lane_num && v.s == s)
    {
      this->vehicles.erase(v_id);
    }
    it++;
  }
  Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0);
  ego.configure(config_data);
  ego.state = "KL";
  this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));
  
}

void Road::cull() {
  
  
  Vehicle ego = this->vehicles.find(this->ego_key)->second;
  int center_s = ego.s;
  set<vector<int>> claimed;
  
  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
    Vehicle v = it->second;
    vector<int> claim_pair = {v.lane,v.s};
    claimed.insert(claim_pair);
    it++;
  }
  it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
    Vehicle v = it->second;
    if( (v.s > (center_s + this->update_width / 2) ) || (v.s < (center_s - this->update_width / 2) ) )
    {
      try {
        claimed.erase({v.lane,v.s});
      }
      catch (const exception& e) {
        continue;
      }
      this->vehicles.erase(v_id);
      
      bool placed = false;
      while(!placed) {
        int lane_num = rand() % this->num_lanes;
        int ds = rand() % 14 + (this->update_width/2-15);
        if(lane_num > this->num_lanes/2)
        {
          ds*=-1;
        }
        int s = center_s + ds;
        if(claimed.find({lane_num,s}) != claimed.end())
        {
          placed = true;
          int speed = lane_speeds[lane_num];
          Vehicle vehicle = Vehicle(lane_num, s, speed, 0);
          this->vehicles_added++;
          this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
          cout << "adding vehicle "<< this->vehicles_added << " at lane " << lane_num << " with s=" << s << endl;
        }
        
      }
    }
    it++;
  }
  
  
}

