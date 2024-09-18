# ENPM662 Project1 Robot Car
<!-- CAD Modeling &amp; Simulation using ROS-Gazebo Environment -->
# Introduction

This is an introductory robot modeling project that demonstrates how to design and simulate a robot car using **CAD modeling in Solidworks** and **simulation using ROS (Robot Operating System) and Gazebo**. The project includes the Solidworks design files for the robot car, as well as the ROS packages required for simulation and control.

# CAD Modeling

The robot car was designed using Solidworks, a 3D CAD software. The design consists of a chassis, wheels, and a controller. The chassis was designed to hold the wheels and the controller, while the wheels were designed to provide mobility to the robot. The controller is used to control the movement of the robot. The Solidworks files for the design can be found in the `CAD Files Robot_Car` directory.

# Simulation

The robot car was simulated using ROS and Gazebo, an open-source robotics simulation platform. ROS was used to control the movement of the robot, while Gazebo was used to simulate the environment. The simulation was used to test the design of the robot and to ensure that it would function as intended. The ROS packages required for simulation and control can be found in the `root` directory.

## Instructions to run the project:

* Download all the files in the package 
* Change the directory to catkin workspace using "cd ~/aditya97_satish_catkin_ws
* Ensure to source the aditya97_satish_catkin_ws prior to building the workspace
* Run catkin_make clean && catkin_make
* If some of the packages couldn't build, make sure to download the deependencies such as ros controllers from their website. 
* After successful cmake, now launch the project using
        
         roslaunch spiderbot spiderbot world.launch

* To run the teleop:
	Open a new terminal and run:

		rosrun spiderbot spiderbot_teleop.py

	
    ### The Keys are:
		
The control keys represents control of joint of robot for 8 DOF in orientation: 

q&emsp;&emsp; w<br>
&emsp;i&emsp;o<br>
&emsp;j&emsp;k<br>
a&emsp;&emsp; s<br>

q/w : movement of forward links - link 1
a/s : movement of rear links  - link 1

i/o : movement of forward links - link 0
j/k : movement of rear links  - link 0

space key, s : force stop

CTRL-C to quit

	
* To run forward kinematics validation:
	
		rosrun spiderbot_fwd_kinematics.py

### The workspace study and inverse kinematics joint angles plots and Simulation are added in the Drive Link. 
Link - https://drive.google.com/drive/folders/19MLxybskd75QSh_gTdxyU184A0bkAgcp?usp=share_link
#### Required packages are: 
* Python 3
* matplotlib and 
* math library 

# How to Run the Robot_Car file

1. Install ROS and Gazebo on your computer.
2. Clone the repository to your local machine using `https://github.com/Tys0nus/ENPM662-Project1-Robot_Car.git`
3. Build the ROS packages by running `catkin_make` in the `ros` directory
4. Run the simulation by launching the appropriate launch file using `roslaunch <robot_car_rev08> <robot_car_rev08_unified.launch>`

# ROS packages

The project includes the following ROS packages:

- **robot_car_rev08**: This package contains the URDF (Unified Robot Description Format), XACRO(XML Macros) file for the robot, as well as the STL files imported from the Solidworks design.
- **robot_car_publisher**: This package contains the ROS nodes that are responsible for controlling the movement of the robot. It includes a simple control algorithm that moves the robot forward and steer control.
- **robot_car_subscriber**: This package contains ROS nodes to corroborate the vehicle forward velocity and steer control parameters are subscribed by the robot car ros controllers

# Dependencies

The project depends on the following software:
- **ROS**
- **Gazebo**
- **Solidworks**

[Video 1](https://drive.google.com/file/d/1exGQE69sUVnIPqdVEfN2HyqEFimZgEAD/view?usp=share_link) : This Video demonstrates the process to control the Robot Car with `w a s d` keys 

[Video 2](https://drive.google.com/file/d/1GPtPinrDI5i7DjVGwFRS_OJyxce2KyqR/view?usp=share_link) : This Video demonstrates the Robot Car traversing in a circular trajectory


Authors - 
- Aditya Chaugule (aditya97@umd.edu)
- Satish Vennapu (satish@2umd.edu)
  
# Conclusion

This project demonstrates how to design and simulate a robot car using CAD modeling in Solidworks and simulation using ROS and Gazebo. It provides a basic understanding of how to create a robot model, how to simulate it in a virtual environment and how to control it using ROS. A Lidar Sensor is integrated with the car for object detection. The project can be further developed by adding sensors and control systems to the robot to make it more advanced.
