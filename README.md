# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Project report by Michael Berner, 2nd of October 2018

---
## Project report / write-up

### Project scope
During this project, we had to implement a model-predictive-controller to keep a simulated vehicle on a race track.

Term 2 simulator version 1.45 was used.

![Intro image](/imgs/Intro.png)


### Polynomial fitting and MPC preprocessing
The current vehicle state is obtained from the simulator in the `main.cpp` routine. A pre-processing is carried out, in order to compensate for the modeled latency of 100ms. The used functions are shown and explained in `main.cpp` at line 127ff.

All provided values from the simulator are in an absolute reference system, which means map or simulator coordinates. I have chosen to transform these values to the vehicle coordinate system, which are easier to process.

The center of the track (see yellow line on image above) is also provided by the simulator. This line is the basis to calculate control commands. It is processed, by fitting a 3rd degree polynomial function to them. The polynomial is used for all further calculations.


### Model description
A model predictive controller was developed.

It is consisting of 6 state variables
- longitudinal position `x`
- lateral position `y`
- yaw angle `psi`
- velocity `v`
- cross-track-error `cte`
- yaw error compared to track `epsi`

and two actuator variables:
- steering angle `delta`
- acceleration value `a`


The MPC controller is defined in the file `MPC.cpp`. Necessary inputs are the prepared state vector including actuator and the coefficients obtained from the polynomial fit described earlier.

Based on this state vector, a cost-function and some further hyper parameters, the MPC controller is optimizing the actuator commands by minimizing the cost function. This is done not only for the current time-step, but for the series of upcoming time-steps. It is necessary to define a vehicle model beforehand, so the vehicle trajectory can be estimated by the MPC controller.

The equations for the vehicle model can be found in line 119ff of `MPC.cpp`.

The cost function is defined in line 53ff and also shown below:

```
// This part of the cost is based on the reference state.
for (size_t t = 0; t < N; ++t) {
  fg[0] +=  500.0 * CppAD::pow(vars[cte_start + t], 2);   // Cross-track-error CTE
  fg[0] += 2000.0 * CppAD::pow(vars[epsi_start + t], 2);  // Yaw error compared to track
  // Target velocity deviation, including conversion of target speed from MPH to m/s
  fg[0] +=    0.6 * CppAD::pow(vars[v_start + t] - ref_v * 0.44704, 2);
}

// Minimize the use of actuators: steering angle and throttle/brake
for (size_t t = 0; t < N - 1; ++t) {
  fg[0] +=    0.1 * CppAD::pow(vars[delta_start + t], 2);   // Steering angle
  fg[0] +=   10.0 * CppAD::pow(vars[a_start + t], 2);       // Acceleration/Deceleration
}

// Minimize the value gap between sequential actuations / time-steps.
for (size_t t = 0; t < N - 2; ++t) {
  // Change in steering angle
  fg[0] +=  200.0 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  // Change in acceleration
  fg[0] +=   10.0 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
  // Change of yaw error compared to lane (reduces wobbling and allows higher speed)
  fg[0] += 1000.0 * CppAD::pow(vars[epsi_start + t + 1] - vars[epsi_start + t], 2);
}
```

Many of the equations were provided in the Udacity lessons and can be understood right away.

Here, I want to point out the last of these equations. I introduced it to achieve higher vehicle speeds by reducing the vehicle oscillations (comparison of yaw error between two time-steps). This was one key aspect to eventually reach 100 MPH.


### Timestep length & elapsed duration

The amount of time-steps `N` and time-step duration `dt` were optimized.

It was found, that the amount of steps `N` should not be lower than 10, because the MPC predicted trajectory became unstable. Increasing the number beyond 10 was also not feasible, because some timing issues started to arise due to CPU limitations.

For the duration `dt`, a starting value of 0.1s was selected, at first. This value was later found to be too low, since the resulting distance, which the MPC was predicting, was not sufficient to keep the vehicle properly on track: the values needs to be higher.  

Eventually, A combination of `N = 10` timesteps with a duration of `dt = 0.12` seconds was found to perform reasonably well.


### Latency compensation

In reality, there will be a delay between the issue of commands and the physical reaction of the vehicle. This is modeled as a delay of 100ms in the simulation environment.

The model's accuracy can be further improved, if the MPC controller takes this latency into account.

I have modeled the latency in my MPC controller, by updating the state vector before it is passed to the MPC routine. Basically, the telemetry data from the simulator is extrapolated using the basic kinematic "bicycle model" along with the assumed 100ms latency. The equations can be found in the main routine in line 127ff.

## Project summary

Overall, a MPC controller was developed, which is able to realize vehicle speeds of 100 MPH.

The vehicle is a bit shaky, but this is due to race-trim which was applied. If the desired vehicle speed is reduced to 80MPH or lower, the vehicle operation becomes much more stable.

A video of the realization can be found on [Youtube](https://youtu.be/HP4NiK15VLQ).

---

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
