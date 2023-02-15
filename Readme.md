# Introduction
This code contains the main function for a robot localization program using Particle Filtering algorithm written in C. The program is designed to run on e-puck mobile robot and uses its IR sensors. For an introduction to e-puck, please visit: https://cyberbotics.com/doc/guide/epuck

This is the final project of my Advanced Robotics course.

See the below videos for a demostration of the localization process:


# How to use
To use this program, you need to have the following:

1) an e-puck mobile robot
2) Webots simulator

Follow this tutorial to learn how to create a simulation containing a simple environment and the e-puck mobile robot: https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots

You can use the controler provided in this repository for the robot.

# Functionality

The main function of the program initializes the necessary sensors and devices, and sets up a loop that runs continuously while the robot is active. During each iteration of the loop, the program reads the sensor data from the distance sensors and accelerometer, processes it using the particle filter algorithm, and calculates the robot's position in the environment.

The program also uses clustering to group the particles, and checks to see if a sufficient number of particles are clustered together to indicate that the robot has been localized. If localization is achieved, the program stops and returns the final position.