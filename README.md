# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program
   
The goal of the project is to implement a Path Planner for creating a path for
a vehicle to be able to drive on a highway.  In order to satisfy the project
rubric requirements, an implementation is created where a smooth path is
generated, a speed limit, acceleration and jerk are taken into account.

This project should be run in the Simulator which can be downloaded
[here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

Cmake is used as a build system for the project. In order to compile the code:
1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./path_planning`

## Important Dependencies

The project has been tested / run on Ubuntu.

* cmake >= 3.5
* make >= 4.1 (Linux, Mac),
* gcc/g++ >= 5.4

### Goals

The goal of the project is to safely navigate around a virtual highway with
other traffic that is driving `+-10 MPH` of the `50 MPH` speed limit.  Using
the car's localization and sensor fusion data (position and velocity of other
vehicles in Euclidean and Frenet coordinates).  The car must respect the speed
limit of `50 MPH`, which means passing slower traffic when possible, note that
other cars will try to change lanes too.  In addition, the total acceleration
should be restricted to `10 m/s^2` and jerk is limited to the `10 m/s^3`.

### Architecture bird's-eye-view

The main program opens a socket connection and listens to a port `4567`, the
simulator publishes a json message on this socket with the relevant information
regarding the current simulation scene (car previous path, car coordinates
(Euclidean and Frenet), coordinates and velocities of other cars on the road).

Given this information, we (i) calculate a path and then fill in `next_x` and
`next_y` fields in the `json` message, (ii) send the json message with a new
path to the simulator, wait for the simulator to perform the scene update and
upon receiving a next message recalculate the path and repeat.

### Path generation

In order to generate a path, we first re-use previous points computed in the
previous step.  The main purpose of this is to have a smooth trajectory and
avoid sharp changes in acceleration and jerk.  In order to generate a smooth
path, we use a spline library, and take anchor points 30, 60, and 90 meters
away from the vehicle.  Before computing the spline, we perform coordinate
transformation from global coordinates to the car frame, use spline, and before
filling in the `next x,y` values perform the inverse transformation.


On the higher level behavour side, we use the Frenet coordinates 
to identify vehicles ahead of us. If there is such a vehicle, and 
we are close to this vehicle, we try to change lanes.
Lane change is only possible when there is no vehicle in the interval
`[-30;30]` meters in the next line. This allows us to change lanes.

Using the spline library allows to keep the path smooth. 

We also tend to stay in the center of the road, i.e. if there are 
no cars around, we stay in the center line.

### Limitation / possible improvements

There is a list of limitations:
* currently when changing lanes, the car just sees that no cars are
in window `[-30; 30]` meters. It does not use any prediction function,
to predict the movements of other cars;

* We do not use cost function. The current implementation just slows
down the car if there is a car in front of us, and it is not possible
to change the lane. 


## Simulator details

1. The car uses a perfect controller and will visit every (x,y) point it
   recieves in the list every .02 seconds. The units for the (x,y) points are
   in meters and the spacing of the points determines the speed of the car. The
   vector going from a point to the next point in the list dictates the angle
   of the car. Acceleration both in the tangential and normal directions is
   measured along with the jerk, the rate of change of total Acceleration. The
   (x,y) point paths that the planner recieves should not have a total
   acceleration that goes over 10 m/s^2, also the jerk should not go over 50
   m/s^3. (NOTE: As this is BETA, these requirements might change. Also
   currently jerk is over a .02 second interval, it would probably be better to
   average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path
   planner returning a path, with optimized code usually its not very long
   maybe just 1-3 time steps. During this delay the simulator will continue
   using points that it was last given, because of this its a good idea to
   store the last points you have used so you can have a smooth transition.
   previous_path_x, and previous_path_y can be helpful for this transition
   since they show the last points given to the simulator controller with the
   processed points already removed. You would either return a path that
   extends this previous path or make sure to create a new path that has a
   smooth transition with this last path.

