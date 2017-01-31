/**
 * light_scan_sim wall_segment_sim.h
 * @brief Simulate laser rays against wall segments
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#ifndef LIGHT_SCAN_SIM_WALL_SEGMENT_SIM_H
#define LIGHT_SCAN_SIM_WALL_SEGMENT_SIM_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <light_scan_sim/SegmentList.h>
#include <light_scan_sim/MaterialList.h>

class WallSegmentSim {
  private:
    light_scan_sim::SegmentList segments_;
    light_scan_sim::MaterialList materials_;
  
    void InitializeWorld();

  public:
    WallSegmentSim(light_scan_sim::SegmentList segments, light_scan_sim::MaterialList materials);

    bool Trace(cv::Point &start, double theta, double length, double &range);
  
};

#endif

