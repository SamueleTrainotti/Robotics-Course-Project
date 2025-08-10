# Robotics-Course-Project
Final project for the course "Fundamentals of Robotics", held by professors Palopoli Luigi and Sebe Niculae (2021/2022).
The course is part of the third year (out of three) of "Bachelor’s Degree in Computer, Communications and Electronic Engineering", available at **University of Trento** (Trento, Italy).
The project was developed with [@Andreolli Fabio](https://github.com/fabiof123) and [@Moiola Christian](https://github.com/christianmoiola).

## Introduction
The project is divided into 4 assignment, each with increasing difficulty.
The main purpose of this work is to simulate a robotic arm to operate in a simulated environment, performing some pick and place operation.

We used UR5 from [Universal Robot](https://github.com/ros-industrial/universal_robot), the [Robotiq 2f-85](https://github.com/ros-industrial/robotiq) as gripper, and simulated the various scenario in Gazebo (version 11) and ROS. Other plugin used were [Gazebo Ros Link Attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) and [Gazebo Grasp Plugin](https://github.com/JenniferBuehler/gazebo-pkgs).

Details can be found in [Project1.pdf](https://github.com/SamueleTrainotti/Robotics-Course-Project/blob/main/Project1.pdf).
Our presentation is also available here [Presentation.pdf](https://github.com/SamueleTrainotti/Robotics-Course-Project/blob/main/Presentation.pdf).
Some videos for each assignment were uploaded to google drive. For videos, training images and other files refer to this [shared folder](https://drive.google.com/drive/folders/1gfYYFXrYf67fyxEjsdsrf31MOUgTB5Uj?usp=sharing).

## Installation

Clone the project
```bash
  git clone https://github.com/SamueleTrainotti/Robotics-Course-Project.git
```

Go to the project directory
```bash
  cd Robotics-Course-Project
```

## Install dependencies
You must have ROS and Gazebo installed. 

The project was developed with Gazebo 11 and Ros Noetic.

You need python3 in order to run pytorch, which handles yolo detection.

You also need OpenCV installed.

To use OpenCV with ROS, install the following package.
```bash
  sudo apt-get install ros-(ROS version name)-cv-bridge
```

Compile workspace
```bash
  catkin_make
```

## Environment Variables

If Gazebo doesn't find the lego models during launch, you need to pu Brick-stl path in `GAZEBO_MODEL_PATH`.

## Start simulation

To run the simulation execute the following command in workspace root folder (i.e. outside the src/ folder)

```bash
  source devel/setup.bash
  roslaunch gripper_01 complete.launch
```
After the world loading ended, you can run Yolo in order to start the pick and place task.
```bash
  rosrun detector detector_node
```
