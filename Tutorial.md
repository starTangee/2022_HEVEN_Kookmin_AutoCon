# Tutorial for Kookmin_AutoCon prereminary assingments

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
```Terminal
mkdir -p ~/kookmin_ws/src
cd kookmin_ws
catkin_make
cd ~/kookmin_ws/src
git clone https://github.com/jhforstudy/2022_HEVEN_Kookmin_AutoCon.git .
```
