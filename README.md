# shared_control
These are ERL's shared control packages

This repository holds packages that impliment a Husky simulation in a disaster arena. This version features a shared control Level of Autonomy (LOA) among Teleoperation and Autonomy.

##How to set up:

-Make a ROS workspace and add this repository to the src directory.

*NOTE: There is another repository for fuzzy_mi_controller in the ERL page. However there are some changes that need to be made in order for Shared Control LOA to work, so I've included the changed version here for convenience.*

- **IMPORTANT**
Make sure that the 'fuzzy_mi_controller' directory is NOT inside the 'shared_control' directory.

**CORRECT**
```
$ ls ~/'your_workspace'/src/

>>CMakeLists.txt shared_control fuzzy_mi_controller
```
AND
```
$ ls ~/'your_workspace'/src/shared_control/

>>delay_node  LICENSE  loa_vfh  README.md  shared_control_node
```


**NOT CORRECT** 
```
$ ls ~/'your_workspace'/src/

>>CMakeLists.txt shared_control 
```
AND
```
$ ls ~/'your_workspace'/src/shared_control/

>>delay_node           LICENSE  README.md
fuzzy_mi_controller  loa_vfh  shared_control_node
```

-Add navigation stack to 'src' directory

    ```
    $ git clone https://github.com/ros-planning/navigation.git
    $ cd navigation
    $ git checkout origin/kinetic-devel
    ```
    
-Add husky drivers to 'src' directory

    ```
    $ git clone https://github.com/uob-erl/husky.git
    $ cd husky
    $ git checkout origin/learning_effect_exp
    ```

-Add the 'variable_autonomy_utilities' package to 'src' directory

    ```
    $ git clone https://github.com/uob-erl/variable_autonomy_utilities.git
    ```
-Build with 
```
$ catkin_make
```

-Run the simulation with 
 ```
 $ roslaunch experiments_launch husky_gazebo_mi_training.launch
 ```
 or 
 ```
 $ roslaunch experiments_launch husky_gazebo_mi_experiment.launch
 ```
 
  
  
  
  
  
  
  
  
  
