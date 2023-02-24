# open_vr_ros
## Forked from [vive_ros](https://github.com/robosavvy/vive_ros) and ported for use on Windows

## **This was tested and worked on Windows 10 with ROS Noetic a Varjo Aero VR Headset, and 2 Valve Index Base Stations**
Because openvr is VR Headset independent, it'll work with any headset which has drivers for openvr. So this should work with the Varjo Aero headset just as well as with the Vive

<br>

## Installation instructions
### Follow this tutorial for installing ROS on windows and setting up an x64 Native Tools Command Prompt for Visual Studio
http://wiki.ros.org/noetic/Installation/Windows  
#### **NOTE: You MUST have an x64 Native Tools Command Prompt for Visual Studio ROS Terminal set up for this to work.** 
#### **You do not need to use Windows Terminal if you don't want to**

<br>
<br>

## Install the Hardware, Steam and SteamVR
Follow this guide for HTC Vive: [Vive-Setup-Guide](https://github.com/osudrl/CassieVrControls/wiki/Vive-Setup-Guide)  
Follow this guide for Varjo Aero: [Setting Up Aero](https://varjo.com/use-center/get-started/varjo-headsets/setting-up-your-headset/setting-up-aero/)

## Prep
Set an environmental variable for %HOME% equal to your %USERPROFILE%:  
[windows env var tutorial](https://docs.oracle.com/en/database/oracle/machine-learning/oml4r/1.5.1/oread/creating-and-modifying-environment-variables-on-windows.html)



## Download and build Valve's OpenVR SDK:

      cd ~
      mkdir libraries
      cd libraries
      git clone https://github.com/ValveSoftware/openvr.git
      cd openvr
      mkdir build
      cd build
      cmake -DCMAKE_BUILD_TYPE=Release ../
      make

if make doesn't work, just use `cmake --build . --config Release`

## Post
Copy the win64 openvr_api.dll to C:\Windows\System32  
run `%HOME%/workspaces/devel/setup.bat` to source the development environment so ROS can find the open_vr_ros package

## Running:
1. Run Varjo Base
2. Turn on steamvr through Varjo Base GUI
1. Start a `roscore`
2. Launch the node: `roslaunch open_vr_ros open_vr.launch`
1. run `rosrun rviz rviz` to start rviz
1. change world frame to `world_open_vr`
1. add by topic


# Troubleshooting
Some things you might have to do:  
Install glew to windows32: https://glew.sourceforge.net/install.html  
Install sdl2.dll to windows32: https://github.com/libsdl-org/SDL/releases/tag/release-2.26.3  
choco upgrade chocolatey  
choco source disable -n=chocolatey  
