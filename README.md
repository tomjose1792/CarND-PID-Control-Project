# CarND-PID-Control-Project



[//]: # (Image References)

[image1]: ./simulation/simulation.png "Simulator screen during the run"

### Project Goal and Implementation:

The project is to implement a PID controller to have a car drive around a track with sharp turns. 
A simulated environment of the track is provided by the Udacity's Self Driving Car Simulator. The cross track error(cte) i.e. the distance between central line of the track and the instantaneous lateral postion of the simulated car is used to calculate the P(proportional), I(Integral) & D(Derivative) terms of the PID controller to govern the steering angle and the throttle of the car for autonomous drive. 

 
Note:

For comprehensive instructions on how to install and run project, please, refer to the following repo, which was used as a skeleton for this project: https://github.com/udacity/CarND-PID-Control-Project
 

#### Basic Build instructions
    $ mkdir build && cd build
    $ cmake .. && make
    $ ./pid

Or use the `$ bash build_and_run.sh` provided to execute the above commands.

### Project Reflections:
--

#### The effect of the P,I,D components on the PID controller implementation.

1. The Proportional (P) component of the controller produces an output value which is proprtional to the cte. The output response is adjusted by the proportional gain constant, Kp. Therefore, when the car goes away by a great margin from the central line along the track it tends to overshoot when it tries to get back to central line.

2. The derivative (D) component of the controller produces an output response which depends on the rate of  change of cte.The output response is adjusted by the derivative gain constant, Kd. Therefore derivative component tends to counteract the overshoot of the car caused by the proportional component.
3. The Integral (I) component of the controller produces an output response which is the summation of the current cte and previous values of cte(s) since the start to the current time step. The output response is adjusted by the iintegral gain constant, Ki. Therefore integral component tends to make the car go in circles because of accumulation of cte over time. 


#### Hyperparameters choice process:
The hyperparameters of Kp,Ki & Kd for the PID control of steering value and throttle were chosen by trial and error method and also with the understanding of how each parameter effects the control.

Initially the parameters for steeering value were tried one by one starting with Kp then Kd and then Ki. As the value for Ki was added, I found that, it tended to push the car out of the track. Therefore a very minimal value was added as Ki.

In order to have better control of the run of the car around the car, a PID controller for throttle was also added. The parameters were for throttle were also tried the same way as for steering value. Here, any value for Ki, tended to take the car to go in reverse at sharp turns. Therefore the Ki was set to zero.

##### The final gain values are:

For steering value control

(Kp,Ki,Kd) = (0.15,0.001,3.0)

For throttle value control

(Kp,Ki,Kd) = (0.2,0.0,1.5)

### Simulation:

The simulator screen showing the autonomous drive through a PID controller.

![alt text][image1]

#### Video
A short video of the run of the car aroudn the car can be found here. 




