# UTHAI-Humanoid
Static Walk of Humanoid Robot video
[![Static Walk of Humanoid Robot](https://img.youtube.com/vi/JMVGSxT79rk/0.jpg)](https://www.youtube.com/watch?v=JMVGSxT79rk)

### Fast Startup Simulation (Kinetic-devel)

0. install lib
    ```
    sudo apt install liborocos-kdl1.3 ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-controller-*
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
    ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/one.png?raw=true)
    ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/two.png?raw=true)
    ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/three.png?raw=true)
    ![](https://github.com/oeyyey/UTHAI-Humanoid/blob/master/Tutorial/four.png?raw=true)


    
