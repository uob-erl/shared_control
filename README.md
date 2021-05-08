# Paper regarding this repository

https://arxiv.org/abs/2011.05228

# shared_control
These are ERL's shared control packages

This repository holds packages that impliment a Husky simulation in a disaster arena. This version features a shared control Level of Autonomy (LOA) among Teleoperation and Autonomy. **IMPORTANT** You might need access to the lab's private repositories to complete the installation. 

## How to set up:

### 1. Make a ROS workspace and add this repository to the src directory.

    *NOTE: There is another repository for fuzzy_mi_controller in the ERL page. However there are some changes that need to be made in order for Shared Control            
     LOA to work, so I've included the changed version here for convenience.*
     
### 2. Make sure that the fuzzy_mi_controller pkg is in the src/ directory and NOT in the shared_control/ directory

### 3. Add navigation stack to 'src' directory

    $ git clone https://github.com/ros-planning/navigation.git
    $ cd navigation
    $ git checkout origin/kinetic-devel
        
### 4. Add husky drivers to 'src' directory

    $ git clone https://github.com/uob-erl/husky.git
    $ cd husky
    $ git checkout origin/learning_effect_exp

### 5. Add the 'variable_autonomy_utilities' package to 'src' directory

    $ git clone https://github.com/uob-erl/variable_autonomy_utilities.git
       
### 6. Installing ROS dependencies

Most ROS dependancies should be taken care of by ``rosdep install --from-paths src --ignore-src -r -y`` from your catkin workspace. If not:

```
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-robot-localization
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-audio-common
sudo apt-get install ros-kinetic-frontier-exploration
sudo apt-get install ros-kinetic-hector-slam
sudo apt-get install ros-kinetic-p2os-urdf
```

### 7. Installing fuzzylite 6 cpp library dependency

   For the controller to compile and run, the fuzzylite library is needed. The controller uses version 6 of fuzzylite. Please use the instructions below to           install fuzzylite 6.
   
   1. Once download from https://fuzzylite.com/ , do a ``cd`` into the fuzzylite folder.
   2. Run ``./build.sh release``
   3. Use checkinstall to create a debian file instead of installing the library old school. It makes life easier e.g. you can install or remove library via apt-get.

```
sudo apt-get install checkinstall
cd release
sudo checkinstall --pkgname=fuzzylite6
```

Alternatively:

```
cd release
sudo make install
```

In order to unistall, if make install was used, ``cat install_manifest.txt | xargs echo sudo rm | sh``.
   
   4. The library is installed.
   
### 8. **IMPORTANT** Gazebo issues

A fresh installation of ROS Kinetic comes with Gazebo v.7.0.0. This version of Gazebo is buggy and causes problems. If that is the case for you, uninstall Gazebo entirely with:

```
sudo apt-get purge gazebo7*
```

and make sure you have v7.04.0 or greater. To do that type:

```
sudo apt-get install ros-kinetic-gazebo-*
```

This should give you Gazebo v.7.16.0


### 9. Build with:

    $ catkin_make
   
### 10. Run the simulation with 

    $ roslaunch experiments_launch husky_gazebo_mi_training.launch
    
   or 
  
    $ roslaunch experiments_launch husky_gazebo_mi_experiment.launch
    
 ### 11. If for some reason everything runs but you can not control the robot; go to src/fuzzy_mi_controller/experiments_launch/launch/husky_gazebo_mi_experiment.launch file and try out different js[number]
 
 Example: 
 
  node respawn="true" pkg="joy" type="joy_node" name="joy_node"
    param name="dev" type="string" value="/dev/input/js0"
    param name="deadzone" value="0.15"
  ....
    param name="dev" type="string" value="/dev/input/js1"
    remap from="/joy" to="/joy_experimenter"
   
   OR 
   
  node respawn="true" pkg="joy" type="joy_node" name="joy_node"
    param name="dev" type="string" value="/dev/input/js1"
    param name="deadzone" value="0.15"
  ....
    param name="dev" type="string" value="/dev/input/js2"
    remap from="/joy" to="/joy_experimenter"
 
  
  
  
  
  
  
  
  
  
