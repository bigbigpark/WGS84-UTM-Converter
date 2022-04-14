/**
 * @file wgs84_to_utm.cpp
 * @author Seongchang Park (scsc1125@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-07 11:22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <wgs84_to_utm/gps_converter.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wgs84_to_utm_converter");
  ros::NodeHandle nh;

  int utm_zone;
  bool hemi;

  if (nh.getParam("utm_zone", utm_zone))
  if (nh.getParam("hemisphere", hemi));

  ROS_INFO("You selected utm zone as %d", utm_zone);

  GpsConverter GC(6378137, 1/298.257223563, utm_zone, hemi);
  GC.init(nh);
  
  ros::spin();

  return 0;  
}