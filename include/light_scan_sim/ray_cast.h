/**
 * light_scan_sim ray_cast.h
 * @brief Cast simulated laser rays on an image.
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#ifndef LIGHT_SCAN_SIM_RAY_CAST_H
#define LIGHT_SCAN_SIM_RAY_CAST_H

#include <sensor_msgs/LaserScan.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <random>

class RayCast {
  std::default_random_engine random_generator_;
  cv::Mat map_;
  double m_per_px_;

  // Default configuration variables
  double ray_min_ = 1.0;  // m
  double ray_max_ = 20.0;  // m

  double angle_min_ = -M_PI_2;
  double angle_max_ = M_PI_2;
  double angle_inc_ = 0.01;  // rad

  double noise_std_dev_ = 0.01;  // std. deviation of laser noise

  public:
    RayCast() {}
    RayCast(double ray_min, double ray_max,
            double angle_min, double angle_max, double angle_inc, double noise) {
      ray_min_ = ray_min;
      ray_max_ = ray_max;
      angle_min_ = angle_min;
      angle_max_ = angle_max;
      angle_inc_ = angle_inc;
      noise_std_dev_ = noise;
    };

    void SetMap(cv::Mat& map, double m_per_px) {
      map_ = map;
      m_per_px_ = m_per_px;
    };

    bool Trace(cv::Point &start, cv::Point &end, cv::Point &hit);
  
    sensor_msgs::LaserScan Scan(cv::Point start, double yaw);
};

#endif

