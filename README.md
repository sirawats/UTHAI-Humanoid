# UTHAI-Humanoid
Static Walk of Humanoid Robot video

[![Static Walk of Humanoid Robot](https://img.youtube.com/vi/JMVGSxT79rk/0.jpg)](https://www.youtube.com/watch?v=JMVGSxT79rk)

### Fast Startup Simulation (Kinetic-devel)

0. install lib
    ```
    sudo apt install liborocos-kdl1.3 ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-controller-
    sudo pip install pyexcel_ods
    ```
1. create workspace.
    ```
    mkdir -p ~/uthai_ws/src
    ```
1. clone UTHAI-Common repository.
    ```
    cd ~/uthai_ws/src
    git clone https://github.com/oeyyey/UTHAI-Humanoid.git
    ```
2. make.
    ```
    cd ~/uthai_ws/
    catkin_make
    ```
3. demo
    ```
    source ~/uthai_ws/devel/setup.bash
    roslaunch uthai_simulation uthai_demo.launch
    ```    
### Simulation 
1.  Click Start
    ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/one.png?raw=true)
2.  - Use 2D Pose Estimate for initial start point
    - Use 2D Nav goal for destination point
    ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/two.png?raw=true)
3.  Wait a while for path searching
    ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/three.png?raw=true)
4.  Robot's going to start walking (slowly)
    ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/four.png?raw=true)

### rqt_graph
   ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/graph.png?raw=true)


  
