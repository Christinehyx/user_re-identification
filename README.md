user_re-identification
====
4th year project, Heriot-Watt University

Step 1: Install OpenCV-python library, dilb library, and face_recognition library
-------
Install the compiler 
```Bash
sudo apt-get install cmake
sudo apt-get install libboost-python-dev
sudo apt-get install python-pip
sudo apt-get install python-numpy
sudo apt-get install python-scipy
```
Install OpenCV
```Bash
sudo pip install opencv-python
sudo pip install opencv-contrib-python
```
Install dlib
```Bash
sudo apt-get install cmake
sudo apt-get install libboost-python-dev
sudo pip install dlib
```
install face_recognition
```Bash
sudo pip install face_recognition
```
If you cannot install face_recognition in this way, you can find more instruction in: https://github.com/ageitgey/face_recognition

Step 2: Install Pepper python-SDK
-------
The link of this package is: https://community.aldebaran.com/en/resources/software

After downloading the package, you need to set the environment variable:
```Bash
sudo gedit devel/setup.bash
```
Add the path of python-SDK at the end of the file:
```Bash
export PYTHONPATH=${PYTHONPATH}:/path/to/python-sdk
```
Step 3: Install the ROS iot_bridge package 
--------
The link of the package is: https://github.com/corb555/iot_bridge
Clone it to your ROS workspace
```Bash
cd catkin_ws/src
git clone address-from-above
cd ..
catkin_make
```
Step 4: Run the system
-------
The link of the program for this system is: 
Before running the program for this system, you need to run:
```Bash
roscore
roslaunch iot_bridge iot.launch
```
Now, you can run the the ros package which include the code above.
