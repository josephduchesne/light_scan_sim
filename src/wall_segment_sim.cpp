#include "light_scan_sim/wall_segment_sim.h"

/**
 * Construct the WallSegmentSim
 * 
 * @param segments  The list of segments making up the world
 * @param materials The list of materials making up the world
 */
WallSegmentSim::WallSegmentSim(light_scan_sim::SegmentList segments, light_scan_sim::MaterialList materials) {
  segments_ = segments;
  materials_ = materials;

  // Create physics world
  this->InitializeWorld();
};

/**
 * @brief Trace a point across the wall segments
 *
 * @param start  The scan trace start point
 * @param theta  The scan trace angle
 * @param length The scan max length (in pixels)
 * @param range  The return distance to point of contact
 *
 * @return true if hit, false otherwise
 */
bool WallSegmentSim::Trace(cv::Point &start, double theta, double length, double &range) {

}


/**
 * @brief Initialize the physics world
 */
void WallSegmentSim::InitializeWorld() {
  ROS_ERROR("Initialize world called");
}
