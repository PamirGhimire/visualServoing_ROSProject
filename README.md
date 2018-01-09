# ROS - Visual Servoing IBVS with Blobs Detection

**Table-of-contents**

* [Dependencies](#dependencies)
* [Process](#process)
* [Execution](#execution)

More description is given in the subsections.

## Dependencies

The following project has been tested with **Ubuntu 14.04 LTS**.

Since this project concerns ROS Indigo, following libraries are needed:

* ROS Indigo - `sudo apt-get install ros-indigo-desktop-full`
* Gazebo Simulation - `sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control`
* Turtlebot for Gazebo - `sudo apt-get install ros-indigo-turtlebot_gazebo`

The development part has been performed using CPP and ViSP:

* ROS Indigo ViSP - `sudo apt-get install ros-indigo-visp-*`
* ViSP - `sudo apt-get install libvisp-dev libvisp-doc visp-images-data`

## Getting your marker in the Gazebo world:
Paste the 'marker0' folder in 'gazeboResources' into your gazebo resources directory.

Usually, this is a hidden folder in your 'home' directory, named '.gazebo'. Paste the 'marker0' folder inside '.gazebo/models/'. In your Gazebo, you'll now be able to 'insert' marker0, check the insert pane!

The 'qr.world' file available in 'world' folder contains a Gazebo world with a pattern containing 5 dots. 


## Process

The following code performs multiple operations to compute **visual servoing task** using **ROS Indigo** and a **Turlebot**.

* Extraction of image from the Kinect ( and dependant topic) .
* Manual initialization of blobs to be tracked 
* Tracking of blobs in successive image frames
* Compute velocities to minimize error between current positions of blobs and their desired positions
* Stop the velocity publishing and computation when error has dropped below a threshold.

## Pseudocode

![Base QR](ressources/pseudocodepython.png)

## Execution

### Preliminary commands
You'll first need to do a minimal launch and a 3dsensor launch on your turtlebot (tip: use ssh!)
Running the minimal launch and the 3Dsensor launch from ros.

* minimal - `roslaunch turtlebot-bringup minimal.launch`
* 3dsensor - `roslaunch turtlebot-bringup 3dsensor.launch`

### Running PointsBased IBVS

* Reality : `rosrun IBVS_PointsBased IBVS_PointsBased`
* Simulation : 
1 : `roslaunch turtlebot_gazebo turtlebot_world.launch world_file:='/world/NameofWorld.world' `
2: `rosrun IBVS_PointsBased IBVS_PointsBased_gazebo`

## Provided in the repository

### launch

This folder contains all the launch files to run the code in real world or Gazebo.

### scripts

This folder contains all the cpp files to run the code in real world or Gazebo.

### model

This folder contains all the files concerning the template to track.

### world

This folder contains the Gazebo world to launch the code under simulation.

## Materials

#### Real View
[![Watch the video](ressources/ibvs1.png)](https://www.youtube.com/watch?v=bUESEUgN75Q)
#### Computer View
[![Watch the video](ressources/ibvs2.png)](https://www.youtube.com/watch?v=yDBamqhc0QQ)

