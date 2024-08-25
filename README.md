# imav24

Indoor and outdoor ROS2 nodes for IMAV24.

## Pre-requisites

 - ROS2 Humble installed. Steps can be found *[here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)*.

 - MicroXRCE-DDS Client installed, to do so you can run : 
 
 ```sh
 git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
 ```
 
 Other installations steps and more details are described *[here](https://docs.px4.io/main/en/middleware/uxrce_dds.html)*.

  - Aruco-OpenCV ROS2 Package :

  ```sh
  sudo apt install ros-humble-aruco-opencv
  ```

  Package *[page](https://index.ros.org/p/aruco_opencv/)* on ROS Index is provided as reference.

## Installation

 1. Create a new workspace folder.

 ```sh
 cd
 mkdir ~/imav_ws
 mkdir ~/imav_ws/src
 ```

2. Clone px4_msgs repo, along with this repo.

```sh
cd ~/imav_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
git clone https://github.com/DronKab/imav24.git
```
**_NOTE_** : The px4_msgs version should match the major version firmware on your PX4 SITL or your PX4 Hardware.

3. Build the workspace, this might take some minutes the first time.

```sh
cd ~/imav_ws
colcon build
```

**_NOTE_** : Don't forget to source your ros2 installation if you haven't. 

You can add the new package to your .bashrc after the build if it didn't fail. This will auto-source the package on every new terminal you open.

```sh
echo "source ~/imav/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

 ## How to run

1. Start your SITL/Gazebo, check *[this repo](https://github.com/DronKab/imav24_sim.git)* for that. Then, run the launch file with :

```sh
ros2 launch imav24 simulador.launch.py 
```

This will run px4_driver node, Aruco detection node, and UXRCE agent. Let it run in the background.

2. Take off and change to Offboard mode. You can run your node afterwards with *ros2 run imav24 <node_name>* in another terminal.
