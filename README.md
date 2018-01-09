# ROS - Visual Servoing PBVS

**Table-of-contents**

* [Project](#project)
* [Visual Servoing](#visualservoing)
* [Materials-Videos](#materials)


More description is given in the subsections.

## Project

## VisualServoing


## Getting your marker in the Gazebo world:
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
