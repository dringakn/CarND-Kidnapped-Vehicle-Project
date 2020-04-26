[Code]: https://github.com/dringakn/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp

[//]: # (Image References)

[image1]: ./examples/Convergance_Result.png "Final Result"
[video1]: ./examples/Particle_Filter_For_Robot_Kidnap_Problem.gif "Video"

# Overview

The goal of this project is to localize a car, using a particle filter, which has lost it's position estimate for a while (kidnapped robot problem). The car has a map of it's location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data. The localization is implemented using a 2D a particle filter algorithm using a C++ program. For the code implementation the reader may refer ![particle_filter.cpp][Code] within the `src` directory.

![Example][video1]

Particle filter algorithm can be considered as a Monte Carlo simulation, where a number of guess poses for the car position and orientation have been used. Each guess pose has a associated weight, which depends on the car's observations. The higher the observation match with car's guess pose, the higher is the weight of the corresponding particle. All those poses which has less weights are finally filtered using a resampling step. The map of the car is implemented using data inside `map_data.txt` file. It includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id
   
## Running the Code
This project involves the Simulator which can be downloaded [here][(https://github.com/udacity/self-driving-car-sim/releases)]. After cloning the project the user can use the follwing commands for compilation and exectuion of the program.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

## Reflection:

The response of the algorithm depends on the number of particles, the higher number of particles can make the algorithm more roboust but increases the computationl cost.

 * The algorithm first initialize the particles, considering vehilce position and standard deviation of the error in x, y, and theta. `ParticleFilter::init()` function sets the number of particles. Afterwards, it initializes the random number generator and three independent gaussian distributations. Each particle is initialized using a range based for loop.
 * `ParticleFilter::prediction()` function predict each particle's pose considering it's current pose and the twist (linear and angular velocity). Some noise in the pose is added as if the prediction step is imperfect.
 * The function `ParticleFilter::dataAssociation()` provides the implementation of a neasrest neighbourhood based search algorithms for finding id of each landmark. The number of observations may be less than the total number of landmarks as some of the landmarks may be outside the range of sensor range.
 * `ParticleFilter::updateWeights()` contains the meat of the program. It first search for the nearby landmarks in the list by filtering the landmarks which are beyond `sensor_range`. Since, the landmark observations are given in the VEHICLE'S coordinate system. Therefore, they are transferred to the MAP'S coordinate system. Afterwards, the landmarks are associated and the weight of each particle is calculated.
 * The function `ParticleFilter::resample()` implents a survival of the fittest re-sampling algorithm, where the probability of each particle to survive depends on it's weight, i.e. particles with higher weights has higher chances of survival. 

Here is the final result.
![result.][image1]