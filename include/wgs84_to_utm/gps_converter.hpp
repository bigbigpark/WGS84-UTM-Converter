/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-07 11:24
 */

#pragma once

#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <GeographicLib/TransverseMercator.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <string>
#include <cmath>

using namespace std;
using namespace GeographicLib;

// Define a UTM projection for an arbitrary ellipsoid
  /**
   * @param a       equatorial radius
   * @param f       flattening
   * @param zone    the UTM zone (52N <- Korea)
   * @param northp  hemisphere (북반구: N, 남반구: S)
   */
class GpsConverter
{
public:
  GpsConverter(double a, double f, int zone, bool northp);
  ~GpsConverter();

  // Init
  void init(ros::NodeHandle& nh);

  // WGS84 to UTM
  void forward(double lat, double lon, double& x, double& y);

  // UTM to WGS84
  void reverse(double x, double y, double& lat, double& lon);

  // ROS funcion
  void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  // Set for private member
  void set_utm_x(double utm_x_) {this->utm_x_ = utm_x_;}
  void set_utm_y(double utm_y_) {this->utm_y_ = utm_y_;}

  // Get for private member
  double get_utm_x() {return utm_x_;}
  double get_utm_y() {return utm_y_;}

private:
  bool isFirstReceived;

  // For utm coordinates
  double utm_x_;
  double utm_y_;

  // For utm path under ROS message
  nav_msgs::Path path_msgs_;

  // ROS publiser and subscriber
  ros::Subscriber gps_sub_;
  ros::Publisher path_pub_;

  TransverseMercator tm_;       // The projection
  double lon0_;                 // Central longitude
  double falseeasting_, falsenorthing_;
};
