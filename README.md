# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

In this project, we will navigate through the track in [udacity simulator](https://github.com/udacity/self-driving-car-sim/releases) using a model-predictive-control algorithm. Given the set of points which the car ideally follows, a cubic polynomial function is fitted to with a least squares method. The algorithm finds an optimal path and actuation values with respect to the current states of the car by minimizing a cost function including cross-track error and orientation error within the next N iterative points in a prespecified timestep, dt. The algorithm also takes an actuation delay of 100 ms, which is typical in many robotic applications. 

## Rubrics 

### The Kinematic Model

The kinematic model is consists of the states and the errors of the vehicle at a specific time. The states involve the horizontal and vertical position of the car, x and y values, orientation angle <img src="https://latex.codecogs.com/svg.latex?\Large&space;\psi"/> and the speed of vehicle, v. The errors are the cross track error, cte, and the orientation error, <img src="https://latex.codecogs.com/svg.latex?\Large&space;e\psi"/>. The ordinary differential equations (ODEs) which describes change of states and errors are as follows:

<img src="https://latex.codecogs.com/svg.latex?\Large&space;x_{t+1} = x_t + v_t cos(\psi_t) dt "/> 

<img src="https://latex.codecogs.com/svg.latex?\Large&space;y_{t+1} = y_t + v_t sin(\psi_t) dt "/>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;\psi_{t+1} = \psi_t + \frac{v_t}{L_f} \delta_t dt "/>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;v_{t+1} = v_t + a_t dt "/>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;cte_{t+1} = cte_t + v_t sin(e\psi_t) dt "/>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;e\psi_{t+1} = e\psi_t + \frac{v_t}{L_f} \delta_t dt "/>

where <img src="https://latex.codecogs.com/svg.latex?\Large&space;L_f"/> is the distance between the center of gravity of the vehicle and the front axle. 

__Note :__ The update equation for the orientation angle is changed to <img src="https://latex.codecogs.com/svg.latex?\Large&space;\psi_{t+1} = \psi_t - \frac{v_t}{L_f} \delta_t dt "/> as the vehicle driven in a different orientation throughout the track. 

### Timestep Length and Elapsed Duration (N & dt)

The time step length and elapsed duration are directly taken from the previoud Udacity example as 25 and 0.05, respectively. These values are pretty conservative as the timestep is small and the total prediction horizon, N*dt = 1.25s, is not long. To make a more efficient calculation, I have used 12-0.1s and 6-0.2s. But to stabilize with coarser configuration, it is necessary to use either a better fit or better cost function.

### Polynomial Fitting and MPC Preprocessing   

As mentioned in the introduction, the waypoints are fitted by a least-squares regression to a 3rd order polynomial using this [formulation](https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716). Now, we can use this polynomial as a reference in error calculations instead of a set of points. 

The mapping from the global coordinates to the vehicle coordinates are made by inverse transformation equation in [here] (https://en.wikipedia.org/wiki/Rotation_of_axes). The update equations and waypoints are also mapped into the vehicle coordinates, so that the calculation is simplified as:

<img src="https://latex.codecogs.com/svg.latex?\Large&space;x_{t+1} = v_t cos(\psi_t) dt "/> 

<img src="https://latex.codecogs.com/svg.latex?\Large&space;y_{t+1} = 0 "/>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;\psi_{t+1} = \frac{v_t}{L_f} \delta_t dt "/>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;v_{t+1} = v_t + a_t dt "/>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;cte_{t+1} = cte_t + v_t sin(e\psi_t) dt "/>

<img src="https://latex.codecogs.com/svg.latex?\Large&space;e\psi_{t+1} = e\psi_t + \frac{v_t}{L_f} \delta_t dt "/>

as x, y and <img src="https://latex.codecogs.com/svg.latex?\Large&space;\psi"/> values of the vehicle are zero in vehicle coordinates. 

### Control Algorithm

A model predictive control(MPC) algorithm is used to keep the  navigating vehicle in track. Before doing so the latency is taken into account for the inputs/ states of the vehicle, therefore, the update equation in vehicle coordinates above is used where dt is regarded as the actuation delay, 0.1 s. Then, the updated values are fed to the MPC routine. 

The cost function and the optimization constraints are determined first. The cost function is consists of the L2-errors of cross-track error, orientation error and velocity error with respect to reference velocity.
```
double cost = 0;
for (int t = 0; t < N; t++) {
    cost += pow(cte[t], 2);
    cost += pow(epsi[t], 2);
    cost += pow(v[t] - v_ref, 2);
}
```

After that, a penalty is applied to the change rates of the actuations.

```
// Minimize change-rate.
for (int t = 0; t < N - 1; t++) {
  cost += pow(vars[delta_start + t], 2);
  cost += pow(vars[a_start + t], 2);
}
```

The gap between actuations at adjacent time step is also penalized. 
```
// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
  cost += pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); 
  cost += pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

And then the constraints are introduced with updated state values and physical actuation constraints, e.g. <img src="https://latex.codecogs.com/svg.latex?\Large&space;\psi \in [-25,25]"/> degree.

Here we used CppAD to compute the derivates and IPOPT to optimized actuation variables in N timesteps within the given constraints.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
