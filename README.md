# WGS84-UTM-Converter

This repository gives and C++ code to transform WGS84 to UTM converter using ROS framework <br/>

<br/>

## Tested Environment

- Ubuntu 18.04
- ROS Melodic

<br/>

## Summary

subscribe a topic as **sensor_msgs/NavSatFix** <br/>

publish a topic as **nav_msgs/Path** <br/>

## How to use

### 1. Prerequisites

[GeographicLib](https://geographiclib.sourceforge.io/html/index.html)

<br/>

### 2. Clone and Build

Clone this repository

~~~bash
$ git clone https://github.com/bigbigpark/WGS84-UTM-Converter.git
~~~

Build

~~~bash
# You can build this 'catkin build' or 'catkin_make'
$ catkin build
~~~

<br/>

### 3. Run

~~~bash
$ roslaunch wgs84_to_utm wgs84_to_utm.launch
~~~

<br/>

### 4. Parameter Configuration

in `/launch/wgs84_to_utm.launch`, <br/>

All you need to do is change **utm_zone** and **hemishpere** <br/>

That value is default of South Korea (UTM 52N) <r/>

~~~xml
<!-- Default setting: UTM 52N in South Korea -->
<param name="utm_zone" type="int" value="52" />
  
<!-- Hemishpere setting: if your country is on North, set it as true, vice versa -->
<param name="hemisphere" type="bool" value="true" />  
~~~

<br/>

## Output

GPS lat, lon is successfully converted to UTM coordinates <br/>

![](/output.png)

<br/>

This is rqt_graph, <br/>

![](/rqt_graph.png)

<br/>

## Known error

~~~bash
CMake Error at CMakeLists.txt:13 (find_package):
  By not providing "FindGeographicLib.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "GeographicLib", but CMake did not find one.

  Could not find a package configuration file provided by "GeographicLib"
  with any of the following names:

    GeographicLibConfig.cmake
    geographiclib-config.cmake

  Add the installation prefix of "GeographicLib" to CMAKE_PREFIX_PATH or set
  "GeographicLib_DIR" to a directory containing one of the above files.  If
  "GeographicLib" provides a separate development package or SDK, be sure it
  has been installed
~~~

This error occurs are result of cannot find `GeoraphicLib_DIR` correctly <br/>

So all you need to do is set `GeographicLib` to be correctly <br/>

<br/>

As an example, in your CMakeLists.txt, <br/>

add that line before `find_package()` , this is **important**<br/>

you should change GeographicLib_DIR to your installed path <br/>

~~~cmake
set(GeographicLib_DIR /home/${user_name}/tools/geographic/lib/cmake/GeographicLib)
~~~

<br/>



