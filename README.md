# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Solution details

The result of implementing the solution could be found [here](https://youtu.be/_QDy-cHXgRo).

### Model description
The model used in the project is the kinematic bicycle model, which ignores
tire forces, gravity, mass and air resistance.

The model consists of state, actuators, errors and update equations.
The state consists of:
```
x, y -> the location of the vehicle
psi -> the orientation of the vehicle
v -> the speed of the vehicle along the current orientation
```
The actuators are:

```
delta -> the steering angle
a -> the acceleration of the vehicle
```

The error measures are:

```
cte -> the cross-track error, the distance of the vehicle to the desired position
epsi -> orientation error, the difference between the desired orientation and the current orientation
```

The model also has update equations:
```
x[t + 1] = x[t] + v[t] * cos(psi[t]) * dt;
y[t + 1] = y[t] + v[t] * sin(psi[t]) * dt;
psi[t + 1] = psi[t] - (v[t] / Lf) * delta[t] * dt;
v[t + 1] = v[t] + a[t] * dt;
cte[t + 1] = (f(x[t]) - y[t]) + v[t] * epsi[t] * dt;
epsi[t + 1] = (psi[t] - psides[t]) - (v[t] / Lf) * delta[t] * dt;
```
where `t` is the current time step and `t + 1` is the predicted one, which is to happen after `dt` time.
`Lf` is the distance between the center of mass of the vehicle and the front wheels and represents
the maneuverability of the vehicle. `f(x[t])` is the target trajectory of the vehicle, evaluated at time `t`.
`psides[t]` is the desired orientation of the vehicle at time `t`.

The update equations are used in setting up the constraints for the optimization problem in `FG_eval`.

### Optimization problem

For finding the best actuators for following the desired curve, an optimizer is used. The optimizer tries
to minimize a predefined cost function in `FG_eval`, lines 53 - 70. In order to emphasise the importance of
some of the variables over the rest of them, different coefficients are used. As it could be seen
from the source code, we setup the cost function such that the optimizer makes minimizing
`cte` and `epsi` its highest priority. We also setup the cost function such that the actuators are used
carefully, without rapid changes between timesteps and possibly not used as frequently. The least important
contribution to the cost function is the ability of the vehicle to drive at a desired speed set as 
`ref_v`. That is we prefer the car to stay on track at any cost, we also prefer pleasant rather than
jerky and surprising moves and we would like to drive at the reference speed, but only when possible.

### Choice of timestep length and duration

Due to the fast changing nature of the environment and the track, we want to predict only `1s` ahead
since that is enough for good results and the intuition is that humans don't try to predict more than `1s` ahead.
We also want to smoothen eventual errors caused by the inaccurate model, so we would like to do the predictions
at intervals of `100ms`. Smaller intervals will be insufficiently small for making decisions and larger ones will
work with outdated information for too long. Therefore our first choice of `N = 10` and `dt = 0.1` appeared to work
good for our needs.

### Preprocessing waypoints

The model update equations have this simple form only when the coordinates and the orientation angle are in vehicle
coordinate system. The coordinates of the waypoints are in global coordinate system, so we need to transform them for
the sake of simpler calculations later on. The transformation happens as follows:

```
double dx = ptsx[i] - px;
double dy = ptsy[i] - py;

ptsx_tr[i] = dx * cos(0 - psi) - dy * sin(0 - psi);
ptsy_tr[i] = dx * sin(0 - psi) + dy * cos(0 - psi);
```
where `ptsx` and `ptsy` are the waypoints, `px` and `py` are the current coordinates of the vehicle in global space
and `psi` is the current orientation of the vehicle.

### Adding latency

In order to account for latency, before running the optimizer, we can set the initial state of the vehicle to
what it should look in `x ms` from now according to our model and current understanding of the world and
if nothing was to change during that time. This prediction is only approximate, but it turns out to work for our needs.

The state approximation is done as follows:
```
double x_latency = v * latency;
double psi_latency = -(v / Lf) * delta * latency;
double v_latency = v + acceleration * latency;
```

The initial state then is:
```
state << x_latency, 0, psi_latency, v_latency, cte, epsi;
```
instead of
```
state << 0, 0, 0, v, cte, epsi;
```

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
