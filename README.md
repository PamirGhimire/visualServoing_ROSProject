# ROS - Visual Servoing PBVS

**Table-of-contents**

* [Project](#project)
* [Visual Servoing](#visualservoing)
* [Getting a template in the Gazebo world](#getting-a-template-in-the-gazebo-world)
* [Materials-Videos](#materials)
* [MultiMaster - Task Combination](#multimaster-&-task-combinations)


More description is given in the subsections.

## Project

The project is developped under **ROS** environment and using **Ubuntu 14.04 LTS** with a ** TurtleBot ** runing on **indigo**. 
The key idea of the project is to develop an application runing on the robot to solve a navigation / object displacement problem.
This big picture can be sub-divided into 3 parts, Navigation, Visual Servoing and Pick & Place. The related work has been performed on Visual Servoing Part.
The goal of the Fine Positionning task is to be able to move from the landing position of the navigation program to a table where a robotic arm will operate.


## VisualServoing

### PBVS - Pose Based Visual Servoing

### IBVS - Image Based Visual Servoing 

### Comparison of processes & choiches of implementation


## Getting a template in the Gazebo world
Paste a folder containing your template in 'gazeboResources' into your gazebo resources directory.

Usually, this is a hidden folder in your 'home' directory, named '.gazebo'. Paste the folder inside '.gazebo/models/'. In your Gazebo, you'll now be able to 'insert' your template, check the insert pane!

Now, Gazebo is able to add template into a world. 

## Materials

### PBVS - Pose Based Visual Servoing
#### Video of the code live on robot
[![Watch the video](ressources/vide.png)](https://www.youtube.com/watch?v=K4BQ3v-MSrs)
#### Video of simulation
[![Watch the video](ressources/video.png)](https://www.youtube.com/watch?v=qCdgKvE52iY)

### IBVS - Qr Point Based Visual Servoing 
This implementation is not working due to perspective and bad projection of the tagrget. The problem can be solved by using image pre-processing to re-align the target and compute the provided program.
The decision to move on another project (**IBVS - Blob Point Based Visual Servoing**) has been done in order to complete an IBVS process with a correctly estimated working-time instead of spending time on something that could take much longer.

### IBVS - Blob Point Based Visual Servoing 
#### Real View
[![Watch the video](ressources/ibvs1.png)](https://www.youtube.com/watch?v=yDBamqhc0QQ)
#### Computer View
[![Watch the video](ressources/ibvs2.png)](https://www.youtube.com/watch?v=bUESEUgN75Q)
## MultiMaster - Task Combinations
