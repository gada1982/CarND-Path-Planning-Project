# Introduction
This project is done as a part of the Nanodegree *Self-Driving Car Engineer* provided by Udacity. The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Therefore a path planner has to be implemented which generates smooth, safe but efficient trajectories for the car to follow.

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
- *main.cpp* - Main file to communicate with the simulator and trigger the path planner
- *pathplanner.h/.cpp* - Pathplanner to create safe, smooth and efficient trajectories for the car
- *helper.h/.cpp* - Helper functions
- *spline.h* - Library used for creating splines (used for the trajectories)
- *json.hpp* - Library used for the communication with the simulator

### Data File
- *highway_map.csv* - File with waypoints around the track (yellow middle lane)

# Project Description
The path planner, which has to be developed, gets map data (highway), the car's localization data and sensor fusion data, which represent the other cars on the highway. The car should try to go as close as possible to the 50 mph speed limit, which means passing slower traffic when possible, while other cars will try to change lanes too. The car should avoid hitting other cars at all cost. The car should drive inside of the marked road lanes at all times unless going from one lane to another. The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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
The following image shows a simplified cycle of the path planning algorithm.

![image_screen](https://github.com/gada1982/CarND-Path-Planning-Project/blob/master/info/cycle.png)

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

Jerk is minimized by using splines with 50 points for the trajectory, which includes points from the last trajectory to get a smooth transition.

### Collission Avoidance
To avoid collisions the most important part is to detect if a car is within the safety distance (25m) in front of the own car. If yes the path planner has to decide if it is possible to change the lane safely or if the car has to brake to stay behind the slower car. `CheckActualLane()` in *pathplanner.cpp* checks if there is a car in front of the own car. If yes the occupation status of all other lanes is checked by `CheckAllLanes()`. This functions returns the information if the other lanes are occupied by other cars. A safe lane change to the left or to the right is only possible if there is no other car in front (25m) or behind (25m) the own car on the next lane on the right or on the left side. This function of course uses the information about the actual lane to decide if the own car is already driving on the outer left or outer right lane.

### Changing Lanes
Changing lane can only be triggerd if the own car has finished the last lane change manover and it has completely reached the lane where it should be. This prevents the car from swerving around between lanes if the lane, where to car should change to, is occupied too. At the moment the car is only allowed to start a lane change when it is going faster than 35mph to avoid getting hit by a faster car on the next lane which could not be detected because of the limited sensor range. All these points are checked in function `ChooseLaneToChange()` within *pathplanner.cpp*. Finally the function `ChooseLaneToChange()` returns if a lane change should/can be done and if yes it sets the new target lane number.

### Creating a Path
The trajectory for the next cycle (approximately every 20ms) is generated by the function `GenerateNextPath()` within *pathplanner.cpp*. This part of the implemententation is strongly influenced by the *Q&A - Video* which is provided by Udactiy for this project. The implementation uses the previous car trajectory, which is delivered by the simulator, to make sure that the car doesn't turn or accelerate/brake to fast and violates the acceleration or jerk limits. The pipeline adds some of these previous points and new target points 30 and 60 in front of the car within it's actual lane or within the lane which the car should change to. This anker points build up a spline. This spline is the base for the next trajectory. A new trajectory with 50 target points is build out of the old reused target points and new ones. The distance between the single points depand how fast the car should go (*ref_vel*) within the next cycle (20ms). The faster the car should drive, the bigger the distance between two of these points has to be. All of these is necessary to generate smooth tracjectories which never violate the maximal turn rates, acceleration and jerk limits.

### Example trajectories
The following image shows two examples of how trajectories could look like:

![image_screen](https://github.com/gada1982/CarND-Path-Planning-Project/blob/master/info/situation.jpg)

In situation 1 the own car (blue) is driving on the middle lane and has a car in front but can't change lanes because the left and the right lane are occupied by other cars within the safety distance (25 before and 25m behind the car). The own car (blue) has to brake and follow the car while staying at the middle lane. The green points show where the car should be in the future (30 and 60m ahead). The green line shows the new trajectory for this situation. The car is slowing down -> the 50 new target points are closer together and the green line is short.

In situation 2 the own car (blue) is driving on the middle lane and has a car in front but it can change lane because right lane is not occupied by an other car within the safety distance (25 before and 25m behind the car). The car has to change to the right lane because the left lane is occupied by a car within the safety distance (25 before and 25m behind the car). The own car (blue) has to brake and follow the car while staying at the middle lane. The green points show where the car should be in the future (30 and 60m ahead). The green line shows the new trajectory for this situation. The car is slowing down -> the 50 new target points are closer together and the green line is short.

# Conclusion
Behavior Planning and path planning are really complex areas within the develepment of autonomous cars. The driving situation which is part of the project (highway driving) is one of the easier ones a real self-driving car has to handle. Especially in the setting with not too much traffic and rare lane changes of the other cars.

The solution was kept as simple as possible while meeting all of the project requirements. 

The following [video](https://youtu.be/_x-chRCk67s) shows how the car, which is controlled by the implemented path planner, manages driving on the virtual highway.

Based on the acutal implementation a more complex solution will be implemented because there are many points for further improvements:
- Refactor the code
- Include a "real" finite state machine and more complex cost funtions
- Include the speed of the car to follow, and not only the distance how far it is away, to get a smoother ride
- Predict how the other cars will behave in the future (next few seconds) and not only check the actual situation
- Implementent an optimicing lane change algorithmus, which better checks on which lane the car will be able to go at the highest speed in the near future
- Use the actual speed to adjust the safety distance necessary in front or behind the car
- Implemented a solution to activly search for free spaces for a lane change. Brake, drive as fast as the cars on the next lane and change.
- ...
