## Sensor Driver installation


1. Install Opencv3.1 (Not essential)
Refer to `OpenCV-Installation` in third-party

2. Install ROS kinetic

[Ros install url](http://wiki.ros.org/kinetic/Installation/Ubuntu)

3. Install QT in third-party (Not essential)

```
chmod 777 ./install_qt.sh
./install_qt.sh
```

4. Install CUDA 8.0

step 4 and 5 is need for ZED camera. 
If you don't use ZED, jump to step 6.

5. Install ZED driver in third-party.

```
chmod 777 ./ZED_SDK_Linux_Ubuntu16_CUDA80_v1.2.0.run
./ZED_SDK_Linux_Ubuntu16_CUDA80_v1.2.0.run
```

6. Run essential_install in third-party.

```
chmod 777 ./essential_install.sh
./essential_install.sh
```

7. Make catkin_ws and src folder

```
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
cd src
catkin_init_workspace
cd ..
catkin_make
```

8. Use wstool for installing drivers

```
$ cd ~/catkin_ws/src
$ wstool init
$ wstool merge sensor_drivers/drives.rosinstall
$ wstool update
``

9. build

```
cd ~/catkin_ws
catkin_make
```

