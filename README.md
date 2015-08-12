Synchronized Image Provider
===========================
The synchronized image provider is a multi-view camera emulation tool that allows to send images from multiple directories synchronously into the ROS system. The images are send via the image_transport package so that available image transport plugins are reusable. The main applications of the tool are (a) to simulate your system with previously recorded image sequences (e.g. from benchmarks) and (b) to bundle synchronous image streams into a rosbag file for re-distribution. The main advantages of the tool are the storage efficiency, i.e. that large images do not have to be bundled into a rosbag file for streaming the images into the ROS system, and the flexibility of starting, stopping and looping the image streams at custom locations. 

Validity
--------
This tool has yet only been tested on Ubuntu 14.04 with gcc-4.9.2 and ROS Jade Turtle.

Installation
------------
0. Clone this GitHub repository into the source directory of your catkin workspace. 
0. Build the package with catkin_make or the catkin tools.

Assumptions
-----------
In case of a multi-view setup, this tool assumes that the number and names of images in **all** directories are the same. Otherwise an error message is returned.

Parameters
----------
The tool is supposed to be started by the in-package launch file *image_provider.launch* that accepts the following parameters:

* **startpos**: The start position to start the image sequences from. If the start position is larger than the number of available images per directory, an error is returned. The default is: 0.
* **length**: The length of the sequence to provide. If the length is larger than 'number of files' - 'start position', no error is returned, but the all images are provided until the end of the sequence. The default is to provide images until the end of the sequences.
* **loop**: Indicates if the sequence is re-started after reaching the end of the sequence. The default is: false.
* **rate**: The rate (in Hz) to provide the image sequences with. The default is: 1Hz.
* **topicprefix**: This sets the root path wherein the topic names for the image transport topics are created. The default is: cameras (in the parent namespace of the node).
* **configfile**: The path of the configuration file that contains the specificatons of the image sequences to provide (see below for details).

Configuration File
------------------
The configuraton is written in YAML format describing where the image sequences are located. An example is shown below:

```
configuration:
  srcdir: /home/hellmund/MRT/Data/Opel/Opel_2015_03_20_Autobahn/seqs
  filepattern: \d{10}.png
  cameras:
    camera_front: 
      directory: IMAGE_FRONT/10.reduced
      imagetype: mono8
      buffersize: 1
    camera_back:
      directory: IMAGE_BACK/10.reduced
      imagetype: mono8
      buffersize: 1
```

The following options are available:

* **srcdir**: The root directory where the image sequences are provided. If this option is omitted, the current working directory is assumed.
* **filepattern**: The file pattern to match image files in the directory. The file pattern has to specified using a regular expression (Boost regular expression).
* **cameras**: This section defines the names of the of the cameras which are used as ROS topic names. There is one configuration entry for every camera name. Each camera configuration accepts the following parameters:
   * **directory**: The directory where the image sequences are found. If the directory name does **not** start with a / (aka. root directory), it is assumed to be relative to the srcdir above. This is a mandatory parameter.
   * **imagetype**: The type of the image. Currently the following formats are supported: *mono8*, *rgb8* and *bgr8*. This is a mandatory parameter.
   * **buffersize**: The queue size of the ROS topic. The default value is 1.
The configuration file uses the YAML format

Running the provider
--------------------
The image provider is supposed to be **always** run through provided launch file. In case you would like to debug the image provider, please use the *launch-prefix* parameter inside the ROS launch file. The following example shows howto execute the image provider:

```
roslaunch ros_synced_image_provider image_provider.launch configfile:=/tmp/config.yaml startpos:=10 length:=10 rate:=10 loop:=true topicprefix:=/my_root/somewhere
```

Feedback
========
If you have any feedback or faced whatever kinds of problems, please let me directly know via [hellmund@fzi.de](mailto:hellmund@fzi.de?subject=ROS Synced Image Provider).
