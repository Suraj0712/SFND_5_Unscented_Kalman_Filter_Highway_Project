[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)

# Udacity Nanodegree: Sensor Fusion

## Project 03: Unscented Kalman Filter

<img src="./Project/media/ukf_highway_tracked.gif" width="800" height="500" />

This project aims to develop a software stack that will enable us to achieve the following objectives.

```
1. We need to develop a constant turn rate and velocity magnitude model (CTRV) based motion model
2. Then we need to initialise the variables based on the first measurement
3. Based on the measurement we need to predict the state of the system after some time ```delta_t```
4. Simulaneousely we need to take the measurement and need to find the ```best estimation``` of state using predicted state and measured state.
```
We are going to implement **Unscented Kalman Filter** to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. The project pipeline can be broadly summerised by the following image.

<img src="./Project/media/project_map.png" width="800" height="500" />

To acheive our goal we need to complete following four major tasks:
1. We are going to create UKF object for every car in the lane
2. The object will be initialise state vector and covariance matrix
3. We will also update other variables like agumented state vector, sigma point matrics, and weight matrics
4. Once we get the measurement we will call predict fucntion with ```elapsed time & last state space vector```
5. In prediction function we will calculate the sigma points, and then based on the sigma points derive the mean and covariance matrix
6. Then based on the type of measurement we receive we will call the appropriate sensor function for update
7. The steps to update are identical for both the sensor except determining the states from the data.

<img src="media/ukf_highway.png" width="500" height="500" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---
## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux)
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
* PCL 1.2

### Basic Build Instructions

#### 1. First of all, clone this repo:
```
$ git clone git@github.com:Suraj0712/SFND_5_Unscented_Kalman_Filter_Highway_Project.git
```

#### 2. Run Quiz
```
$ cd <Path_to_quiz_directory>
$ makdir build && cd build
$ cmake ..
$ make
$ ./<executable_name>
```
#### 3. Run Project
```
$ cd <directory_where_you_have_cloned_the_repo>/SFND_5_Unscented_Kalman_Filter_Highway_Project/Project/
$ makdir build && cd build
$ cmake ..
$ make
$ ./<executable_name>
```



In this project you will implement an  Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, and src/ukf.h

The program main.cpp has already been filled out, but feel free to modify it.

---
### Project Rubic
#### FP.1 Constant turn rate and velocity magnitude model (CTRV)
The motion of the vehicle can be tracked by many different models. In the implementation of  ```Kalman filter and Extended Kalman Filter``` we assumed the constant velocity model however this model will give good estimate when the vehicle is following the straight line and suffer when the vehicle goes into a turn. To better simulate this scenario we use the Constant turn rate and velocity magnitude model (CTRV) as our motion model. The state vector for the model is as follows

```
State vector = [ Position_in_x , Position_in_y , magnitude_of_velocity , yaw_angle , yaw_angle_change_rate ]

```

