# **Project 7: Path planning**

This document presents the work developed for the 7th project of the SDC Nanodegree.

Table of Contents:
1. Project Introduction
2. Build Instructions
3. Reflection on how to generate paths
4. Results

## 1. Project Introduction
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The map of the highway is in data/highway_map.txt, and each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Using the available information, the objective of this project is to generate a series of points (x, y) for the car to follow at all times. These combined points generate a trajectory that the vehicle must follow, fulfilling all the requirements described above.

### 1.1. Input Data
Here is the data provided from the Simulator to the C++ Program

### 1.1.1 Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### 1.1.2 Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### 1.1.3 Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### 1.1.4 Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

### 1.2. Output Data

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## 2. Build instructions
### 2.1. Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

### 2.2. Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## 3. Reflection on how to generate paths
Some of the steps that were needed to complete this project are:

* **Get the current location and speed of the vehicle**: The first thing to do is to establish the actual state of the ego vehicle. This information comes from the localization module, and is already given in this project. It is important to note that, after some path has been generated for the car, at the next time step it is better to place the vehicle over the same path, to mantain continuity in the trajectories and make them smooth and comfortable.
* **Get the location of the cars (and other objects) around the ego vehicle**: Since this is a highway with other cars (and in real life, other agents), it is very important to know where these other agents are. The sensor fusion module provides this information, and in this project it is included in the input data. For trajectory generation, it is important to know where the other cars are and where they will be in the next time steps, in order to properly avoid possible collisions with them. In the code, this is done by:
    *  Looping through all the cars detected by the sensor fusion module.
    *  Detecting where each of these vehicles are with respect to the ego car (left, right or same lane, in front or behind) and how far or close they are.
    *  A proper location of the other vehicles with respect to the ego vehicle allows to take more informed decisions about what to do next (*i.e.*, plan the next trajectory).
* **Decide what to do next**: Now we know where our car is and where are the other cars around us. This information must then be used to establish the strategy for the next points of the path. It is very useful to establish a finite set of possible actions (for example, a finite state machine), in order to simplify the decision making process and ensuring feasibility of the future trajectory. In this project, 4 states were defined:
    * Keep lane: Keep going in the same lane. In this state the objective is to take the car's speed as close to the speed limit as possible, without going above it. Depending on the presence/absence of cars in front of the ego car, our car can accelerate or brake to avoid collisions. One special case at this state is when it comes extremely close to another vehicle (for example, because another car just changed lanes and came very close to us). In this case, the car will dramatically slow down its speed to avoid collisions, while respecting the acceleration constraint.
    * Prepare for lane change: If the car sees a car in front of it and is obligated to slow down, it will enter this state in which the ego car starts looking at the side lanes (left and/or right, depending on where the car is at the moment) to see if a lane change is possible without crashing with other cars.
    * Change lane to the left: If the car finds it is possible to change to the left lane, then it will enter this state, in which the car basically changes the target lane where it wants to be in a few time steps in the future. 
    * Change lane to the right: Similar to the previous state, but changing to the opposite lane. In this project, if the car sees that it can change to both its right and left lanes, the car will always try to change to the left lane first, under the assumption that the most-left lane is the fast lane of the highway.
* **Generate a smooth trajectory**: After defining the states of the vehicle and the transition functions between them, we are now able to tell the car what to do at each time step. However, 1 very important step is missing, and that it generating a feasable trajectory over time. Whether it is staying in the same lane and reducing/mantaining/increasing the car's speed, or changing to another lane, we must be able to generate trajectories that are feasible (the car is able to follow it), safe (no collisions), legal (not above the speed limit) and smooth/comfortable (respecting the acceleration and jerk constraints). To do this, we take our target velocity and lane for the car at that moment and project some points forward with this information. Then, we mathematically find a trajectory that passes through those points. In this project I used a spline, generated with the code available at https://kluge.in-chemnitz.de/opensource/spline/.

## 4. Results
This project has two main criteria to determine if the implementation of the particle filter is successful:

1. : Drive for a minimum distance (4.32 miles) without having incidents of any kind. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. 
2. Performance: The speed of the car must not be much slower than the speed limit, unless obstructed by traffic.

Below, I show a video of the car at the moment that it reaches 4.32 miles of driving without incidents. This is the minimum distance that the vehicle is required to achieve in order to pass this project.

<img src="media/1.gif" width="600">
    