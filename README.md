# Self-Driving Car with MPC

[image1]: images/MPC_Setup.png "MPC_Setup"

---
## Objective
This project use MPC(Model Predictive Control) technology to control udacity simulate car. MPC can solve optmization problem to find best input arguments that minimizes the cost functions base model and constraints.

The MPC is consisted by state, model, constraints and cost fucntion.

## State
The state is sytem variables and errors references: [x,y,psi,v,cte,epsi]. 

* x and y is for the vehicle position, 
* psi is the vehicle orientation, 
* v is the vehicle speed and finally, 
* cte and epsi is for the cross track error and orientation error.


## Model, Constraints and Cost
![alt text][image1]

* Model equations is the image as above.
Lf measures the distance between the front of the vehicle and its center of gravity
* Constraints
 * steering angle is from -25 to 25
 * acceleration and brake is from 1 to -1
* cost function
* ```c
        // The part of the cost based on the reference state.
        for (int t = 0; t < N; t++) {
            fg[0] += (50 + t * 10) * CppAD::pow(vars[cte_start + t], 2);
            fg[0] += (50 + t * 10) * CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += 5 * CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        // Minimize change rate.
        for (int t = 0; t < N - 1; t++) {
            fg[0] += 1600000 * CppAD::pow(vars[delta_start + t], 2);
            fg[0] += 50 * CppAD::pow(vars[a_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int i = 0; i < N - 2; i++) {
            fg[0] += 40000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i] , 2);
            fg[0] += 10 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i] , 2);
        }
}
```

## Time Step and Duration(N & dt)

* T(Predition horizon) equal N*dt. Higher N will have more computational cost. For this project I tried N values from 10 to 20, dt is from 0.05 to 0.2. Finally, I chose N=20, dt=0.05 that result is good. 

## Latency
Because of latency, I put the latency before predict next state.

```c
                    // Step . Set latency 0.1second
                    double dt = 0.1;
                    double x1=0, y1=0,  psi1=0, v1=v, cte1=cte, epsi1=epsi;
                    x1 += v * cos(0) * dt;
                    y1 += v * sin(0) * dt;
                    psi1 += - v/Lf * delta * dt;
                    v1 += a * dt;
                    cte1 +=   v * sin(epsi1) * dt;
                    epsi1 += - v * delta / Lf * dt;
```

## Demo

# reference velocity = 70 miles
[![MPC 70 miles](http://img.youtube.com/vi/LNQsHva3xjc/0.jpg)](https://youtu.be/LNQsHva3xjc
 "MPC 70 miles")

# reference velocity = 80 miles
[![MPC 80 miles](http://img.youtube.com/vi/ZCAd6MSTZHk/0.jpg)](https://youtu.be/ZCAd6MSTZHk
 "MPC 80 miles")
 
# reference velocity = 100 miles
[![MPC 100 miles](http://img.youtube.com/vi/aXCqoZm-Ul8/0.jpg)](https://youtu.be/aXCqoZm-Ul8
 "MPC 100 miles")
 
# reference velocity = 110 miles(Ha! car crash) 
[![MPC 110 miles](http://img.youtube.com/vi/yWo6RdpJwXg/0.jpg)](https://youtu.be/yWo6RdpJwXg
 "MPC 110 miles")

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

