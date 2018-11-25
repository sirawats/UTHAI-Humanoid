# UTHAI-Common
 ROS packages for the model description and the gazebo setting of UTHAI


### Useful command

Convert `.xacro` file to `.urdf` file.
```ros
rosrun xacro xacro --inorder -o model.urdf model.xacro
```

### การใช้งาน UTHAI Simulation

1. go to workspace folder.
    ```
    $ cd ~/catkin_ws/src
    ```
1. clone UTHAI-Common repository.
    ```
    $ git clone https://github.com/UTHAI-Humanoid/UTHAI-Common.git
    ```
1. make catkin workspace.
    ```
    $ cd ~/catkin_ws
    $ catkin_make
    ```
    > หลังจากนั้น ปิด Terminal แล้วเปิดใหม่
1. run uthai gazebo simulation.
    ```
    roslaunch uthai_gazebo uthai_world.launch
    ```
    ![](https://github.com/UTHAI-Humanoid/UTHAI-Documents/blob/master/repo-images/uthai_gazebo_sim.png?raw=true)

1. test script
    
    เปิด Terminal อีกอันขึ้นมา
    ```
    rosrun uthai_gazebo test_gazebo.py
    ```
    ![](https://github.com/UTHAI-Humanoid/UTHAI-Documents/blob/master/repo-images/uthai_gazebo_sim.gif?raw=true)

    