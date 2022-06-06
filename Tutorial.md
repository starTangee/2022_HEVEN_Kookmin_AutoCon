# Tutorial for Kookmin_AutoCon prereminary assingments
This is the simple tutorial for Kookmin university Autonomous contest preliminary assignments. You have to install some setups and build your workspace in the following workflow.

## Prerequisites
### Environments
Ubuntu 18.04 & ROS melodic

### Installation
Install setups\
`rosbridge`
```Terminal
sudo apt update
sudo apt install ros-melodic-rosbridge-server
```
dependency packages\
`ar-track-alvar`, `pygame`, `pillow`
```terminal
sudo apt update
sudo apt-get install ros-melodic-ar-track-alvar
pip2 install pygame==1.9.6
pip2 install pillow==6.2.2
```

### Create workspace
Create catkin_workspace and build for Kookmin_AutoCon repository.
```Terminal
mkdir -p ~/kookmin_ws/src
cd ~/kookmin_ws/src
git clone https://github.com/jhforstudy/2022_HEVEN_Kookmin_AutoCon.git .
cd ..
catkin_make
```
If your catkin_make complete, now you are ready to launch the code.

## Run shortcuts
### Assignment1
Launch the rosbridge
```terminal
cd ~/kookmin_ws
source devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
Run the simulator
```terminal
cd ~/kookmin_ws/src/xycar_sim_driving
./xycar3Dsimulator.x86_64
```
Launch the driving code for assignment1
```terminal
cd ~/kookmin_ws
source devel/setup.bash
roslaunch assignment1 driving.launch
```

### Assignment2
Launch the parking code and the simulator
```terminal
cd ~/kookmin_ws
source devel/setup.bash
roslaunch assignment2 parking.launch
```




