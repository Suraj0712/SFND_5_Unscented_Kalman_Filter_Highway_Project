Kalman filter consist of two cycles sensor update and state prediction. In kalman filter we assume sensor reading and robots motion control follows the gaussian distribution.

## Steps in Kalman filters
1. Initialization
-> First sensor take some measurement and based on that measurement we initialise our state

Now robot movex in environment

2. Measurement step
-> robot takes the measurement of distance with onboard proximity sensor
-> so we have estimation of state based on measurement

3. Prediction step
-> based on the change in the odometry values of the sensor we can predict the current state based on the previous state and odometry readings
-> so we have a estimate of state based on the prediction

new_mean = mean_1 + mean_measurement
new_var = var_1 + var_measurement	

4. now we combine this two distribution to get better estimate

new_mean = (mean_1 * var_2 + mean_1 * var_2)/ var_2 + var_1
new_var = 1 / (1/var_1 + 1/var_2)

5. Now we have a better estimate of our pose. now again robot will go to move step and cycle continues.

## Kalman filter with more than one state
1. Here the steps remains same however the we need to use linear algebra for the estimation.
### Observed vs infered states
1. So lets say out state space contains 3 states [ Position, Velocity, Accelaration]. We know all the states are interrelaed. If we take measurement of position in consecutive frames we will get velocity and accelaration without measuring the position.
2. Lets consider two variables [ position , velocity ]. so if we know one variable we can infer about the second one. lets see we plat this variables on the graph where x-coordinate is position and y-coordinate is velocity. 
3. As we know this variables are normaly distributed so we will get gaussian with mean at [position, velocity] point.
4. BUt every state will have some uncertianty, and if the states are independent then we will get two gaussian which are perpendicular to each other, else we will get sloped gaussianwhere the projection on axis represent the variance associated with that particular state.

## Notations

### State Prediction
x = state vector [position , velocity]
p = covariance matrix [sigma_position sigma_velocity]' * [sigma_position sigma_velocity]

F = state prediction matrix 
  > Position = Pose_prev + time_elapsed * velocity
  > velocity = velocity  
  > F = [1 time_elapsed; 0 1]

so as every point will transform by F the covariance will also transform by F
P = F * P * F'

so at the end of ```prediction``` cycle we get 
x = x * F
P = F * P * F

we have not considered the effect of the external influence which will be
x = x * F + B * U

B = control matrix
u = control vector

we address the uncertainty in the sensor measurement by adding some noise in std deviation
P = F * P * F + Q

Q = Noise

```
So at the end of prediction step we get 
x = x * F + B * U
P = F * P * F + Q
```

### Sensor update
The sensors will take some measurements and we can convert them into our state space by multiplying it by matrix H

x = H * [sensor reading vector]
P = H * P * H'

### Combining both the results and gettting better estimate
Now we have a two estimate for position from prediction and sensor measurement. we will multiply them to get the best estimate.

[ref_1](http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies)
[ref_2](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
