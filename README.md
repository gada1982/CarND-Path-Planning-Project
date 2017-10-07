# Introduction
This project is done as a part of the Nanodegree *Self-Driving Car Engineer* provided by Udacity. The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Therefor a path planner has to be implemented which generates smooth, safe but efficient trajectories for the car to follow.

![image_screen](https://github.com/gada1982/CarND-Path-Planning-Project/blob/master/info/Screenshot1.png)

# Outline
1. Requirements to Build and Run
2. Files
3. Project Description
4. Implementation Details
5. Conclusion
  
# Requirements to Build and Run

### Development Environment
- cmake >= 3.5
   - All OSes: [click here for installation instructions](https://cmake.org/install/)
- make >= 4.1
  - Linux: make is installed by default on most Linux distros
  - Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  - Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
- gcc/g++ >= 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros
  - Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  - Windows: recommend using [MinGW](http://www.mingw.org/)
- [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  - Run either `install-mac.sh` or `install-ubuntu.sh`.
  - If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
### Build Instructions
1. Clone this repo
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`

### Simulator
- Udacity Term 3 [simulator](https://github.com/udacity/self-driving-car-sim/releases) which contains the Path Planning Project.

# Files

### Source Files
- *main.cpp* - Main file to communicate with the simulator and trigger the pathplanner
- *pathplanner.h/.cpp* - Pathplanner to create safe, smooth and efficient trajectories for the car
- *helper.h/.cpp* - Helper functions
- *spline.h* - Library used for creating splines (used for the trajectories)
- *json.hpp* - Library used for the communication with the simulator

### Data File
- *highway_map.csv* - File with waypoints around the track (yellow middle lane)

# Project Description
The path planner, which has to be developed, gets map data (highway), the car's localization data and sensor fusion data, which represent the other cars on the highway. The car should try to go as close as possible to the 50 mph speed limit, which means passing slower traffic when possible, while other cars will try to change lanes too. The car should avoid hitting other cars at all cost. The car should drive inside of the marked road lanes at all times, unless going from one lane to another. The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

### Map Data
Each waypoint is represented by [x,y,s,dx,dy] values. X and y are the waypoint's map coordinate position. The s value is the distance along the road in Frenet coordinate system and dx and dy define the distance vector (in Frenet system) from the center of the road (yellow) line to the border of the road. One lane is 4 meters wide. The highway is a closed loop with s value going form 0 to 6945.554.

### Localization Data

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

Previous path data given to the Planner:

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

Previous path's end s and d values :

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data
["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Definition of valid trajectories:

Valid trajectories have the following attributes:

- The car is able to drive at least 4.32 miles without incident.
- The car drives according to the speed limit.
- Max Acceleration and Jerk are not Exceeded.
- Car does not have collisions.
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lanes.

# Implementation Details

# Conclusion

The following [video](https://youtu.be/_x-chRCk67s) shows how the car, which is controlled by the implemented path planner, manages driving on the virtual highway.
