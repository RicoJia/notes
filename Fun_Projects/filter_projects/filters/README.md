# An Ea
## Part 1 Particle_Filter
### Introduction
A particle is composed of a guess of the current state (e.g, the speed and location of a moving robot) and a weight (probability of the guess being the true state). The main idea of the particle filter is to iteratively generate a group of such "particles" to **describe the probability distribution of the true current state.** The higher probability a particle carries, the more likely that particle's state will appear in the final state estimate. Particles with lower weights will be filtered out.

For updating each particle, the state estimate part is updated based on the system dynamics (control model), and their weight is re-evaluated by the possiblity to generate the most recent observation.

In our face tracker, we use a box to represent the object. Our state vector is: ```[x_center, y_center, vx, vy, hx, hy, scale_change_rate]```.```[x_center. y_center]``` describes the location of the center of the box, ```[vx, vy]``` is the velocity of the box along x, y direction. ```[hx, hy]``` describes the sides of the box, and ```scale_change_rate``` describes the percentage change in sides of the box.

The user selects an Region Of Interest (ROI) as the object they'd like to track. Later, state vector of each particle is updated by control model, and its histogram of pixels' RGB is compared with that of the ROI, and generating a weight. The more similar the two histograms, the higher weight the particle gets.

Since the particle filter is such a widely used baysian filtering technique, I extracted out its common framework and its structure looks like this: 
=======
## Particle_Filter
1. Introduction
>>>>>>> parent of 64b4eba... README
    <p align="center">
    <img src="https://user-images.githubusercontent.com/77752418/128048513-1b1e405f-d3ff-46e4-9517-7795ede05908.png" height="500" width="width"/>
    <figcaption align="center">Workflow </figcaption>
    </p>

<<<<<<< HEAD
### Step 3 Control Update
The user of the framework is responsible to issue a control signal to each particle and update the state estimate accordingly. Also, noise should be added to account for the randomness from the real world. As a continuation of the above example, the robot's position will be updated with its ```speed*time``` and noise. The distribution of all states are effectively shifted and dispersed (from noise). [Image Source](https://www.youtube.com/watch?v=tvNPidFMY20)

<p align="center">
<img src="https://user-images.githubusercontent.com/39393023/123560583-c2d14f80-d768-11eb-8666-2df30086fa90.png" height="300" width="width"/>
</p>

### Step 4 Weight Update (or Observation Update)
The user of the framework is responsible to evaluate the probability (weight) of each state. In this step, we should have already received a measurement ```z(t)```, and we will evaluate **the likelihood of getting z(t), if the true state of the system is x(t)**. Mathmatically, it's represented as ```P(z|x)```, or an "observation model". **Mathematical Model, TODO**

### Step 5 Weight Normalization
The sum of all states' weights may not be one, on the otherhand, we **DO** need it to be one since it's a probablity distribution. Therefore, will normalize all weights at this stage. 

### Step 6 Return Average Belief
Return the average of all states as the final belief of the iteration. 

    
## Tips For Implementation

   

