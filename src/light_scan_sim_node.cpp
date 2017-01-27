#include <ros/ros.h>
#include <stdint.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <random>

nav_msgs::OccupancyGrid::Ptr map;
cv::Mat map_mat;
tf::Transform map_to_image;
std::default_random_engine generator;

void mapCallback(const nav_msgs::OccupancyGrid::Ptr& msg)
{
  ROS_INFO("Got occupancy grid");
  map = msg;
  map_mat = cv::Mat(map->info.height, map->info.width, CV_8UC1, map->data.data());
  // Set empty space (255) to 0 (free)
  cv::threshold(map_mat, map_mat, 254, 255, 4);
  
  map_to_image.setOrigin(tf::Vector3(map->info.origin.position.x, map->info.origin.position.y, map->info.origin.position.z));
  map_to_image.setRotation(tf::createQuaternionFromRPY(0, 0, 0));  // Image is in standard right hand orientation

}

/**
 * @brief Return the first collision point (hit) and true, or false if no collision
 *
 * @param mat The map to raytrace on
 * @param p1  The start point. Will be clipped to image.
 * @param p2  The end point. Will be clipped to image.
 * @param hit The collision point
 *
 * @return true if there was a collision or false
 */
bool rayTrace(const cv::Mat &mat, cv::Point &p1, cv::Point &p2, cv::Point &hit) {

  // Ensure that the line is in the mat
  if (!cv::clipLine(mat.size(), p1, p2)) {
    return false;
  }

  // Iterate from p1 to p2
  cv::LineIterator it(mat, p1, p2, 8);
  for(int i = 0; i < it.count; i++, ++it) {
    if (map_mat.at<uint8_t>(it.pos())>0) {
      hit = it.pos();
      return true;
    }
  }
  return false;
}

/**
 * @brief Create a simulated laser scan
 * 
 * @param mat The map
 * @param p The origin point. px
 * @param a0 The start angle
 * @param a1 The end angle
 * @param inc The angle increment
 * @param min The min range. m
 * @param max The max range. m
 *
 * @return a laser scan message
 */
sensor_msgs::LaserScan scan(const cv::Mat &mat, cv::Point p, double a0, double a1, double inc, double min, double max) {
  sensor_msgs::LaserScan scan;

  scan.angle_min = -3.1415/2.0;
  scan.angle_max = 3.1415/2.0;
  scan.range_min = min;
  scan.range_max = max;
  scan.angle_increment = inc;
  scan.header.frame_id = "/initialpose";
  scan.header.stamp = ros::Time::now();
  std::normal_distribution<double> dist(0.0, 0.01);

  cv::Point hit;
  double max_p = max/map->info.resolution;
 
  for (double a = a0; a<=a1; a+=inc) {
    cv::Point p2 = cv::Point(p.x + max_p*cos(a), p.y + max_p*sin(a));
    // ROS_INFO_STREAM(" p2.x: " << p2.x << " p2.y: " <<p2.y << " a: " << a << " max_p: " << max_p ); 
    
    if(rayTrace(mat, p, p2, hit)) {
      double value = cv::norm(hit-p);  // distance from start to hit
      value *= map->info.resolution;  // convert back to m
      // Add gaussian noise
      value += dist(generator);

      // todo: Apply min
      scan.ranges.push_back(value);
    } else {
      // ROS_WARN("Laser didn't hit map");
      scan.ranges.push_back(max+1.0);  // Out of range
    }
  }

  return scan;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "light_scan_sim");

  ros::NodeHandle node;
  static tf::TransformBroadcaster br;

  tf::TransformListener listener;
  ros::Subscriber sub = node.subscribe("/map", 1, mapCallback);
  ros::Publisher pub = node.advertise<sensor_msgs::LaserScan>("/scan", 1);
  cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );

  ros::Rate rate(40.0);
  while (node.ok()){
    ros::spinOnce();

    if (map == NULL) {
      ROS_ERROR("LSSN: No /map");
      ros::Duration(1.0).sleep();
      continue;
    } else {
      br.sendTransform(tf::StampedTransform(map_to_image, ros::Time::now()+ros::Duration(1.0), "map", "map_corner"));
    }

    ros::Time now = ros::Time::now();

    tf::StampedTransform transform;
    try{
      listener.waitForTransform("/map_corner", "/initialpose", now, ros::Duration(0.1));
      listener.lookupTransform("/map_corner", "/initialpose", now, transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("LSSN: %s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    ROS_INFO_STREAM("transform get, has map");
    // Convert pose to map frame
    double roll, pitch, yaw;
    transform.getBasis().getRPY(roll, pitch, yaw);

    cv::Point p1(transform.getOrigin().x()/map->info.resolution, transform.getOrigin().y()/map->info.resolution);

    sensor_msgs::LaserScan s = scan(map_mat, p1, yaw-3.1415/2.0, yaw+3.1415/2.0, 0.01, 0, 20.0);
    s.header.stamp = now;  // Set time so that TFs work correctly
    pub.publish(s);

    /*
    cv::Point p2 = cv::Point(p1.x + 50*cos(yaw), p1.y + 50*sin(yaw));

    ROS_INFO_STREAM("X: " << p1.x << " Y: " << p1.y << " Yaw:" << yaw);
    cv::Point hit;
    if (rayTrace(map_mat, p1, p2, hit)) {
      // Calculate distance
      cv::circle(map_mat, hit, 4, 255);
      ROS_INFO_STREAM("X: " << hit.x << " Y: " << hit.y << " Yaw:" << yaw);
    }
    

    cv::imshow( "Display window", map_mat );
    cv::waitKey(1);
    */ 

    rate.sleep();
  }
  return 0;
};
