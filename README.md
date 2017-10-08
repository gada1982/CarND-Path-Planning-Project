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
The localization module delivers data about the own car. Each set of data includes [x,y,s,d,yaw,speed]. In addition to this basic information, data about the previous path [previous_path_x, previous_path_y, end_path_s, end_path_d] is given back to use it at the actual cycle to generate smoother trajectories.

### Sensor Fusion Data
The sensor fusion module delivers data about all other cars which are driving next to the own car. The data format for each car includes [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and vx, vy are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for each car.

### Definition of valid trajectories:

Valid trajectories have the following attributes:

- The car is able to drive at least 4.32 miles without incident.
- The car drives according to the speed limit.
- Max Acceleration and Jerk are not exceeded.
- Car does not have collisions.
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lanes.

# Implementation Details

### Driving Behaviour
The car drives as fast as possible but doesn't go faster than the speed limit (50mph). It accelerates and brakes smoothly except there is the need for an emergency brake, because of an other car which changes the lane to close to the front of the own car.

The following dummy code from the implementation in *pathplanner.cpp* shows how acceleration and braking is implemented and how the maximal allowed acceleration and braking (10m/s^2) is handled:

Acceleration until the speed limit is reached:
```
if(ref_vel < 50mph)
{
  // Accelerate with 5m/s^2
  ref_vel += change_rate_speed;
}
```

Braking:
```
if(DISTANCE_TO_CAR_IN_FRONT < 12.5m)
{
  // Emergency Brake
  // Brake with 9m/s^2
  ref_vel -= 1.8*change_rate_speed;
}
else if(DISTANCE_TO_CAR_IN_FRONT < 25m)
{
  // Brake with 5m/s^2
  ref_vel -= change_rate_speed;
}
```

### Collission Avoidance
Collission avoidance is implemented detacting if a car is within the safety distance (25m) in front of the own car. If yes the path planner has to decide if it possible to change lane safely or if the car has to brake to stay behind the slower car. `CheckActualLane()` in *pathplanner.cpp* checks if there is a car in front of the own car. If yes all other lanes are checked by `CheckAllLanes()`. This functions returns the information if the other lanes are occupied by other cars. A safe lane change is only possible if there is no other car in front (25m) or behind (25m) on the next lane on the right or on the left side.

### Changing Lanes
Changing lane can only be triggerd if the own car has finished the last lane change manover (???) an it has completely reached the lane where it should be. This prevents from swerving around between lanes if the lane where to car should change to is occupied too. The car is only allowed to start a lane change when it is going faster than 35mph to avoid getting hit by a faster car on the next lane which could not be detected because of the limited sensor range. All of this is checked in function `ChooseLaneToChange()` within *pathplanner.cpp*. Finally the function `ChooseLaneToChange()` returns if a lane change should be done and if yes it sets the target lane number.

### Creating a Path
The trajectory for the next cylce (approximately every 50ms) is generated by the function `GenerateNextPath()` within *pathplanner.cpp*. This part of the implemententation is strongly influenced by the *Q&A - Video* which is provided by Udactiy for this project. The implementation uses the previous car trajectory, which is deliverd by the simulator, to make sure that the car doesn't turn or accelerate/brake to fast and violate the acceleration or jerk limit. Afterwards the pipeline adds these previous points and new target points 30, 60, 90m in front of the car within it's actual lane or at the lane which a lane change should be made to to a spline. This spline is the base for the next trajectory. This spline is filled up with 50 new target points. The distance between the single points depands how fast the car should go within the next cycle (50ms). The faster the car should drive, the bigger is the distance between two of these points. This generates smooth tracjectories which never violate the maximal turn rates, acceleration and jerk limits.

# Conclusion

The following [video](https://youtu.be/_x-chRCk67s) shows how the car, which is controlled by the implemented path planner, manages driving on the virtual highway.

Going faster and slower
Use more complex solutions to generate trajectories (prediction)
