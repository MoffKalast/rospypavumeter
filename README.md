# rospypavumeter

## Overview

Analog VU meter developed using the PulseAudio library, using a Python wrapper of the original C library. Composed by one node, vumeter_node, explained below. 

## Install

To fix all dependencies run the install.sh script in /scripts first.

## Run

Example of a launcher for a vumeter node. For more information try to run the node using rosrun
```sh
$ rosrun rospypavumeter vumeter -h
```
For running using roslaunch
```sh
$ roslaunch rospypavumeter levels_monitor.launch robot:=<robot_name>
```

## ROS API

### Nodes

#### vumeter_node

PulseAudio allows any sound sink to be monitored. The PulseAudio API is asynchronous and callback based, and the documentation is primarly just an API reference. To help with volume meter style applications, PulseAudio even allows you to ask for peak level measurements, which means you can sample the monitor sink at a low frequency, with low CPU utilisation, but still get useful values. When this feature is used, each sample read indicates the peak level since the last sample. 

This program is designed to detect whatever sink is active in any moment in a dynamic manner, instead of previous implementations which have this parameter harcoded, and read the stream of audio coming from it, publishing the detected peaks through ROS.  

##### Published Topics

`audio_level`(*std_msgs/UInt8*):
    Integer with the value of the peak received from the sink.

##### Parameters

`~meter_rate`(*int*, default: *5*):
    Rate of audio checks.

`~display_scale`(*int*, default: *0*):
    For scaling purposes, not used.

## LICENSE

The license of the packages is custom LASR-UC3M (Licencia Académica Social Robotics Lab - UC3M), an open, non-commercial license which enables you to download, modify and distribute the code as long as you distribute the sources.  

## ACKNOWLEDGEMENTS

![RoboticsLab](http://ieee.uc3m.es/images/thumb/b/b6/Roboticslab_text_new.jpg/128px-Roboticslab_text_new.jpg)
![UC3M](http://ieee.uc3m.es/images/thumb/6/6b/Logo_uc3m_letras.png/256px-Logo_uc3m_letras.png)
