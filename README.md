
## My Project Reflections
The Model Predictive Controller (MPC) succesfully maneuvers the vehicle around the track following the waypoints. A video of the MPC driving the vehicle around the track can be found here:
https://youtu.be/pTl6FehfqYA

# The Model
The model used in this model predictive controller includes the following states:
-x (x position)
-y (y position)
-psi (orientation)
-v (velocity)

This state is then propogated by the control inputs of the steering angle and throttle values as follows:
x1 = (x0 + v0 * cos(psi0) * dt);
y1 = (y0 + v0 * sin(psi0) * dt);
psi1 = (psi0 + v0 * delta0 / Lf * dt);
v1 = (v0 + a0 * dt);

Where Lf is the is the between the front of the car and its center of gravity, dt is the time between controller updates, a0 is the throttle value, and delta0 is the steering angle. 

The cross-track error and orientation errors are updated as follows:
cte1 = ((f0 - y0) + (v0 * sin(epsi0) * dt));
epsi1 = ((psi0 - psides0) + v0 * delta0 / Lf * dt);


These update equations are all implemented on lines 124-131 of MPC.cpp.

The steering angle is limited between plus and minus 25 degrees, and the throttle is limited between plus and minus 1. This is limited in lines 201-210 of MPC.cpp.

The cross-track error and orientation errors are updated as follows:
cte1 = ((f0 - y0) + (v0 * sin(epsi0) * dt));
epsi1 = ((psi0 - psides0) + v0 * delta0 / Lf * dt);



# Timestep and Elapsed Duration
Through trial and error, I determined that a timestep (dt) of 0.1 second with a horizon of 1 second (N = 10) works pretty well. This also aligns with roughly the amount of latency in the system, which means the latency implementation is easy.

# Polynomial Fitting and MPC Preprocessing
The lake_track_waypoints.csv file provides the waypoints for the car to follow, but in order to interpolate between these for the optimizer, I fit a 3rd order polynomial to the waypoints. However, before doing so, I transformed these points from the global frame of reference to the vehicle fixed frame of reference. This frame of reference has the x direction facing forward. The coordinate transformation between the global and vehicle frame is as follows:
veh_x[i] = (pts_x[i] - px)*cos(-psi) - (pts_y[i] - py)*sin(-psi);
veh_y[i] = (pts_x[i] - px)*sin(-psi) + (pts_y[i] - py)*cos(-psi);

Where pts_x/y are the points of interest, (px,py) is the location of the vehicle, and psi is the orientation of the vehicle. This can be found on lines 102-107 of main.cpp.

Once transformed, the 3rd order polynomial was fit to the on line 111 of main.cpp. Finally, the initial crosstrack error (cte) and orientation error (epsi) were computed using this polynomial. Some simplification is possible because the vehicle is at x=0, particularly for orientation error, which is the negative atan of the derivative of the polynomial evaulated at 0, which is just the first coefficient. This is computed on lines  114 and 117 of
cte = polyeval(coeffs, 0);
epsi = -atan(coeffs[1]);


Finally, once the optimal solution is computed, I apply an average of the first three steering inputs as the control input, and the first of the throttle inputs. The steering value, in radians, is converted to a value between -1 and 1 for the unity simulator by dividing by converting to degrees and dividing by 25 (or the total fo dividing by .46332).This is on lines 127-136 of main.cpp

# Model Predictive Control with Latency
In order to address the 100ms of latency in the system, the model predictive controller assumes that the previous optimal control output is applied for the first 100ms of its prediction horizon. This is implemented by applying an equality constraint on the control inputs for the first timestep. This can be seen on lines 222 to 229 of MPC.cpp


## Original README:
# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
