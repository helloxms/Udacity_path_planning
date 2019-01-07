# Udacity CarND Path Planning Project
Self-Driving Car Engineer Nanodegree Program

##Introduction
In this project, my goal is to design a path planner that is able to create smooth, safe paths for the car to follow
 along a 3 lane highway with traffic.This path planner will be able to keep inside its lane, avoid hitting other cars, and passw slower moving traffic all by using localizaion,sensor fusion, and map data.

- The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
- The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
- The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


#### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

#### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


#### Dependencies
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)


##My WorkFlow
thanks for Aaron Brown AND  David Silver's explanation course.

####Base Step
- install working condition and build project
(uWebSockets,ssh lib, build local server etc.)
- run the local server and the simulator
(we make the vehicle continue running in the lane)
- understanding all the parameters 
(understanding Main car's localization data, previous path
 x/y,end path x/y,sensor fusion data )
- use Main car's global localization ,use spline interpolation curve, use sensor fusion data, create a path planning controller

####Code Detail

- create base Vehicle structor , PathPlanning structor and  
VehicleStatMachine structor;

	1. **Vehicle structor** save information about id,x,y,vx,vy,s,d,speed. These information get from sensor fusion data ,and used for PathPlanning controller.

	2. **PathPlanning structor** save information about Main card's surrounding traffic conditions,we use these information to
calculat every lane's costs,and select the minimal costvalue lane to drive.

	3. **VehicleStatMachine** save information about Main Card's
stat,used for change lane and keep lane.

- flow chart

- some core function and core logic


	1. **getFrenet** Transform from Cartesian x,y coordinates to Frenet s,d coordinates

	2. **getXY** Transform from Frenet s,d coordinates to Cartesian x,y 

	3. **NearbyVehicles** get information of nearby vehicle,
check every lane,get vehicles whitch are in main car's range.

	4. **Nearest vehicles** get main car's nearest vehicles in each lane, include the front car and the back car.

	5. **too_close** if there is a vehicle in front of the main car and keep in a low speed,set the "too_close" stat to true

	6. **minimal_cost**  we define a cost value function,in main.cpp's code at line 551. (costvalue = kd*dValue + ks*sValue)	We will select the target lane whose cost value is the minimal.

	7. **inChangeLane** if the target lane is different with the current driving lane ,set the "inChangeLane" stat to true.

	8. **Spline** use the spline interpolation curve to set the next waypoints

	9. **update waypoints** update waypoints to simulator in every 0.02 seconds.

- result display


 


















