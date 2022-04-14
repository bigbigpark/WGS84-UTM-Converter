/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-07 11:24
 */

#include <wgs84_to_utm/gps_converter.hpp>

using namespace GeographicLib;

GpsConverter::GpsConverter(double a,    
               double f,    
               int    zone,    
               bool   northp)  
    : tm_(a, f, Constants::UTM_k0())
    , lon0_(6 * zone - 183)
    , falseeasting_(5e5)
    , falsenorthing_(northp ? 0 : 100e5)
{
  if (!(zone >= 1 && zone <= 60))
    throw GeographicErr("input utm_zone is not in [1,60]");
}

GpsConverter::~GpsConverter()
{

}

// Init
void GpsConverter::init(ros::NodeHandle& nh)
{
  isFirstReceived = false;

  utm_x_ = 0.0;
  utm_y_ = 0.0;

  path_msgs_.header.frame_id = "camera_init";

  path_pub_ = nh.advertise<nav_msgs::Path>("/gps_path", 1);
  gps_sub_ = nh.subscribe("/gps_data", 1, &GpsConverter::gps_callback, this);
}

// WGS84 to UTM
void GpsConverter::forward(double lat, double lon, double& x, double& y)
{
  // lon0_은 경도의 중심
  tm_.Forward(lon0_, lat, lon, x, y);
  x += falseeasting_;
  y += falsenorthing_;
}
 // UTM to WGS84
void GpsConverter::reverse(double x, double y, double& lat, double& lon)
{
  x -= falseeasting_;
  y -= falsenorthing_;
  tm_.Reverse(lon0_, x, y, lat, lon);
}

void GpsConverter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  double e, n;
  
  // IsReceived?
  if (!isFirstReceived)
  {
    isFirstReceived = true;

    ROS_WARN("Get first WGS84 information !");
  }

  // Transform WGS84 to UTM coordinates
  forward(msg->latitude, msg->longitude, e, n);

  // Update utm_x, utm_y
  set_utm_x(e);
  set_utm_y(n);

  // Print Current position in utm format:
  cout << fixed << setprecision(9);
  cout << "utm_x:  " << get_utm_x() << endl;
  cout << "utm_y:  " << get_utm_y() << endl;

  path_msgs_.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";

  pose_stamped.pose.position.x = get_utm_x();
  pose_stamped.pose.position.y = get_utm_y();

  path_msgs_.poses.push_back(pose_stamped);

  path_pub_.publish(path_msgs_);
}
