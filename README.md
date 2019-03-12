# spinnaker_camera_driver

These are the ros drivers for running the Pt Grey (FLIR) cameras that use the Spinnaker SDK with hardware trigger mode.  This code has been tested with various Point Grey Blackfly S (BFS) cameras. 

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

The pre-requisites for this repo include:
* ros-kinetic-desktop-full
* spinnaker (download from [Pt Grey's website](https://www.ptgrey.com/support/downloads))
* ros-kinetic-cv-bridge
* ros-kinetic-image-transport
* libunwind-dev
* libtbb-dev

```bash
# after installing spinnaker verify that you can run your cameras with SpinView

# after installing ros, install other pre-requisites with: 

sudo apt install libtbb-dev libunwind-dev ros-kinetic-cv-bridge ros-kinetic-image-transport
```

### Installing
To install the spinnaker drivers, just copy this repo to your catkin workspace, then:
```bash
catkin_make
source devel/setup.bash
# add this to ~/.bashrc to make this permanent 
```

## Running the drivers

Modify the `params/xxxxxxxx.yaml` file replacing the cam-ids and master cam serial number to match your camera's serial number. Then run the code as:
```bash
roslaunch spinnaker_camera_driver acquisition_<X>.launch
# Test that the images are being published by running
rosrun image_view image_view image:=/RGB_camera/<cam_aliases>/image_raw
```
## Parmeters
All the parameters can be set via the launch file or via the yaml config_file.  It is good practice to specify all the 'task' specific parameters via launch file and all the 'system configuration' specific parameters via a config_file.  

### Task Specific Parameters
* ~binning (int, default: 1)  
  Binning for cameras, when changing from 2 to 1 cameras need to be unplugged and replugged
* ~color (bool, default: true)  
  Should color images be used (only works on models that support color images)
* ~exp (int, default: 0)  
  Exposure setting for cameras
* ~save (bool, default: false)  
  Flag whether images should be saved or not (via opencv mat objects to disk)
* ~save_path (string, default: "\~/projects/data")  
  Location to save the image data
* ~save_type (string, default: "tiff")  
  Type of file type to save to when saving images locally: binary, tiff, bmp, jpeg etc.
* ~to_ros (bool, default: true)  
  Flag whether images should be published to ROS.  When manually selecting frames to send to rosbag, set this to False.  In that case, frames will only be sent when 'space bar' is pressed
* ~config_file (String, default: params/cam_L_params.yaml)  
    Path to the config file that contains camera serial number and camera types, camera aliases, camera configuration, and etc.

### System configuration parameters
* ~cam_ids (yaml sequence or array)  
  This is a list of camera serial numbers in the order which it would be organized.  The convention is to start from left to right.
* ~cam_aliases (yaml squence or array)  
This is the names that would be given to the cameras for filenames and rostopics in the order specified above eg. cam0, cam1 etc.
* ~cam_types (int, default: )  
  This is type for camera. 1 for BFS and 0 for BlackFly.


### Camera info message details
* ~image_width (int)
* ~image_height (int)
* ~distortion_model (string)  
  Distortion model for the camera calibration.  Typical is 'plumb_bob'
* ~distortion_coeffs (array of arrays)  
  Distortion coefficients of all the cameras in the array.  Must match the number of cam_ids provided.
* ~intrinsic_coeff (array of arrays)  
  Intrinsic coefficients of all the cameras in the array.  Must match the number of cam_ids provided.
  Specified as [fx  0 cx 0 fy cy 0  0  1]
* ~projection_coeffs (array of arrays)  
  Projection coefficients of all the cameras in the array.  Must match the number of cam_ids provided.
* ~rectification_coeffs (array of arrays)  
  Rectification coefficients of all the cameras in the array.  Must match the number of cam_ids provided.

**GPIO Pinouts for Blackfly S**  
<img src="docs/images/bfs_GPIO.png" alt="GPIO Pinouts for Blackfly S" width="640" align="middle">

**GPIO Connections for Master/Slave config**  
<img src="docs/images/gpio_connections.png" alt="GPIO Connections for Master/Slave setup" width="360" align="middle">  

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
