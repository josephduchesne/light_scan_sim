/**
 * light_scan_sim ray_cast.cpp
 * @brief Cast simulated laser rays on an image.
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#include "light_scan_sim/ray_cast.h"

/**
 * @brief Return the first collision point (hit) and true, or false if no collision
 *
 * @param start The start point. Will be clipped to image.
 * @param end   The end point. Will be clipped to image.
 * @param hit   The collision point
 *
 * @return true if there was a collision or false
 */
bool RayCast::Trace(cv::Point &start, cv::Point &end, cv::Point &hit) {

  // Ensure that the line is in the map
  if (!cv::clipLine(map_.size(), start, end)) {
    return false;
  }

  // Iterate from start to end
  cv::LineIterator it(map_, start, end, 8);  // 8 way connectivity, smoother than 4 way
  for(int i = 0; i < it.count; i++, ++it) {
    if (map_.at<uint8_t>(it.pos())>0) {
      hit = it.pos();
      return true;
    }
  }
  return false;
}

/**
 * @brief Create a simulated laser scan
 * 
 * @param start The origin point. px
 * @param yaw The origin angle of the scan
 *
 * @return a laser scan message
 */
sensor_msgs::LaserScan RayCast::Scan(cv::Point start, double yaw) {
  sensor_msgs::LaserScan scan;

  scan.angle_min = angle_min_;
  scan.angle_max = angle_max_;
  scan.range_min = ray_min_;
  scan.range_max = ray_max_;
  scan.angle_increment = angle_inc_;
  std::normal_distribution<double> gaussian_dist(0.0, noise_std_dev_);

  cv::Point hit;
  double max_px = ray_max_/m_per_px_;
 
  for (double a = angle_min_; a <= angle_max_; a+=angle_inc_) {
    cv::Point end = cv::Point(start.x + max_px*cos(yaw+a),
                              start.y + max_px*sin(yaw+a));
    
    if (Trace(start, end, hit)) {
      double range = cv::norm(hit-start);  // distance from start to hit
      range *= m_per_px_;  // convert back to m
      // Add gaussian noise
      range += gaussian_dist(random_generator_);

      // todo: Apply min
      scan.ranges.push_back(range);
    } else {
      scan.ranges.push_back(ray_max_+1.0);  // Out of range, represented by value>max
    }
  }

  return scan;
}
