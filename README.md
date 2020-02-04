# ROS platform for KUKA iiwa and ReFlex TakkTile control

This project presents a modular ROS platform for the teleoperation of the KUKA iiwa (7 R800 or 14 R820) robot manipulator and an attached ReFlex TakkTile robotic hand. The system supports control of both devices using the Leap Motion gesture tracker in a virtual reality environment. The project also presents a first attempt at autonomous grasping prediction.

## Folder Structure 

| Folder Name | Contents                                                         |
|-------------|------------------------------------------------------------------|
| cad         | CAD files of 3D printed parts                                    |
| common      | library of common functions used by leap_hand and leap_kuka      |
| docs        | project documentation                                            |
| grasping    | code for autonomous grasping prediction (work in progress)       |
| kuka        | code for controlling the KUKA robot                              |
| leap_hand   | code to interface between Leap Motion node and robotic hand node |
| leap_kuka   | code to interface between Leap Motion node and KUKA node         |
| leap_rig    | files to start Leap Motion control of KUKA and robotic hand      |
| modules     | third party software                                             |
| vision      | all vision related code (camera and virtual reality)             |

## Hardware Components

* KUKA iiwa 7 R800
* ReFlex TakkTile Robotic Hand
* Leap Motion
* Oculus Rift DK2
* Asus Xtion Pro Live
* Switch
* 3 ethernet cables (category 5 or higher)

## Software Components

The system was tested using the following software. 

* Ubuntu Xenial 16.04 LTS
* Python 2.7
* numpy 1.11.0
* MATLAB Version 9.6 (R2019a) 
* MATLAB Instrument Control Toolbox Version 4.0 (R2019a)
* MATLAB [Robotics System Toolbox](https://uk.mathworks.com/help/robotics/ug/install-robotics-system-toolbox-support-packages.html) Version 2.2         (R2019a)
* MATLAB [Robotics System Toolbox Interface for ROS Custom Messages](https://uk.mathworks.com/matlabcentral/fileexchange/49810-robotics-system-toolbox-interface-for-ros-custom-messages)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Desktop Install recommended)
* KUKA MatlabToolboxServer on KUKA Robot Controller. See user guide of [KUKA Sunrise Toolbox](https://github.com/Modi1987/KST-Kuka-Sunrise-Toolbox)
* Only for grasp prediction: CUDA V10.1.168 (currently not needed)
* Only for grasp prediction: Conda 4.6.14 (installed miniconda2)
* Only for grasp prediction: Conda environment with dependencies in grasping/env/environment.yaml

## Installation

1. Install all of the above software components.
2. Initialize a clean catkin workspace. Open a terminal and type
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
3. Clone this repository with all its submodules in the src folder of your workspace
```bash
git clone --recursive git@github.ic.ac.uk:KukaProject/alex_code.git
```
4. Install all dependencies. You might need to [setup rosdep first](http://wiki.ros.org/rosdep)
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y 
```
5. Build workspace with catkin build
```bash
cd ~/catkin_ws
catkin build 
```
6. Remember to source the setup.bash file or add it to your .bashrc
```bash
source /opt/ros/kinetic/setup.bash
or
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```
7. Call ```rosgenmsg("~/catkin_ws/src/kuka/kuka_msgs")``` in Matlab command window and follow onscreen instructions. See further instructions [here](https://uk.mathworks.com/help/robotics/ug/create-custom-messages-from-ros-package.html). 

## Networking Setup
* ReFlex TakkTile Robotic Hand (Address: 10.1.1.10, Netmask: 254.0.0.0, Gateway: 0.0.0.0, check the "Require IPv4 addressing" button). Also make sure your ethernet connection shows up as "eth0". If it does not [this link](https://askubuntu.com/questions/767786/changing-network-interfaces-name-ubuntu-16-04) might help. Please view the Robotic Hand's [documentation](https://www.labs.righthandrobotics.com/reflex-quickstart) for more details. In their instructions you can skip all sections on cloning and building the drivers as they will already be included in this package.
* KUKA Robot (in our case, Address: 172.31.1.55, Netmask: 16, Gateway: 172.31.1.110).

## Further Steps
* To run the grasp prediction script it is highly recommended to create a conda environment first. You can use the included environment.yaml file.
```bash
conda env create -f ~/catkin_ws/src/grasping/env/environment.yml
conda env list
```
* To use the grasp prediction script download the a pretrained neural network using the ```download_pretrained_ggcnn.sh``` shell script. 
* Perform camera calibration (RGB and depth recommended) before using scripts that use the RGBD camera (especially AprilTag detection). Use the [camera_calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) node contained in the image_pipeline module which will directly save the calibration parameters to ```~/.ros/camera_info```. A backup of the calibration results of the camera used in this project can be found in the vision/calibration folder. Example commands to calibrate rgb and depth camera.
```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0245 image:=/camera/rgb/image_raw camera:=/camera/rgb
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0245 image:=/camera/ir/image camera:=/camera/ir 
```
* When printing the universal flange connector make sure to stop the 3D printer at the right layer to insert the nuts.

## Authors

**Alexander Koenig**
* Maintainer of repository
* Research Assistant at Imperial College London (April 2019 - September 2019)
* [Mechatronics in Medicine Laboratory](https://www.imperial.ac.uk/mechatronics-in-medicine), Department of Mechanical Engineering
* E-Mail: awckoenig@gmail.com
* [LinkedIn](https://de.linkedin.com/in/alexander-koenig-95b9a0134)

**Dr Riccardo Secoli**
* Project supervisor
* Research Associate at Imperial College London
* [Mechatronics in Medicine Laboratory](https://www.imperial.ac.uk/mechatronics-in-medicine), Department of Mechanical Engineering
* E-Mail: r.secoli@imperial.ac.uk

**Dr Fabio Tatti**
* Project supervisor
* Research Associate at Imperial College London
* [Mechatronics in Medicine Laboratory](https://www.imperial.ac.uk/mechatronics-in-medicine), Department of Mechanical Engineering
* E-Mail: f.tatti@imperial.ac.uk

**Prof Ferdinando Rodriguez y Baena**
* Project supervisor
* Professor of Medical Robotics at Imperial College London
* [Mechatronics in Medicine Laboratory](https://www.imperial.ac.uk/mechatronics-in-medicine), Department of Mechanical Engineering
* E-Mail: f.rodriguez@imperial.ac.uk

## Acknowledgments

* Dr. Riccardo Secoli for providing scripts for the control of the Robotic Hand
* M. Safeea and P. Neto, "KUKA Sunrise Toolbox: Interfacing Collaborative Robots With MATLAB," in IEEE Robotics & Automation Magazine, vol. 26, no. 1, pp. 91-96, March 2019.
* D. Morrison, P. Corke and J. Leitner, "Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach" in Robotics: Science and Systems (RSS), 2018.
* Maintainers of third party repositories