#include "light_scan_sim/wall_segment_sim.h"
#include <cmath>
#include <light_scan_sim/Segment.h>

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
bool WallSegmentSim::Trace(double x, double y, double theta, double length, double &range) {
  if (isnan(theta)) {
    return false;
  }

  //set up input
  b2RayCastInput input;
  input.p1 = b2Vec2(x, y);
  input.p2 = input.p1 + length*b2Vec2(cos(theta), sin(theta));
  input.maxFraction = 1;


  // Todo: Create list of all collided lines, sort and process in order
  
  //check every fixture of every body to find closest
  b2RayCastOutput closest;
  closest.fraction = 1.1;
  bool hit = false;

  for (b2Body* body = world_->GetBodyList(); body; body = body->GetNext()) {
    for (b2Fixture* fixture = body->GetFixtureList(); fixture; fixture = fixture->GetNext()) {
      b2RayCastOutput output;
      const light_scan_sim::Segment *segment = static_cast<light_scan_sim::Segment*>(body->GetUserData());
      // ROS_INFO_STREAM("fixture: " << segment->start[0] << ", " << segment->start[1]);
      if ( ! fixture->RayCast( &output, input, 0) ) {
        continue;
      }
      if ( theta < 0.01 && theta > 0){
        ROS_INFO_STREAM("check: " << input.p1.x << "," << input.p1.y << " " << input.p2.x << "," << input.p2.y);
        ROS_INFO_STREAM("fixture: " << segment->start[0] << ", " << segment->start[1]);
        ROS_INFO_STREAM("Hit" << output.fraction);
      }
      if ( output.fraction >= 0 && output.fraction <= 1.0 && output.fraction < closest.fraction ) {
        closest = output;
        hit = true;
      }
    }
  }
  // b2Vec2 intersectionPoint = input.p1 + closest.fraction * (input.p2 - input.p1); 
  
  if (hit) {
    range = closest.fraction * length;
    // ROS_INFO_STREAM("Closest: " << range);
  }

  return hit;
}


/**
 * @brief Initialize the physics world
 */
void WallSegmentSim::InitializeWorld() {
  world_ = std::make_shared<b2World>(b2Vec2(0,0));

  for (auto i : segments_.segments) {
    ROS_INFO_STREAM("Initializing segment of type " << materials_.materials[i.type].name);

    // Create shape representing segment
    b2EdgeShape edge_shape;
    ROS_INFO_STREAM("Edge output: " << i.start[0] << "," << i.start[1] <<  " " << i.end[0] << "," << i.end[1]); 
    edge_shape.Set( b2Vec2(i.start[0], i.start[1]), b2Vec2(i.end[0], i.end[1]) );
   
    // Create fixture definition of the shape
    b2FixtureDef fixture_def; 
    fixture_def.shape = &edge_shape;
  
    // Create body to contain fixture
    b2BodyDef body_def; 
    body_def.type = b2_staticBody; // can't move
    body_def.position.Set(0, 0); // origin

    b2Body* segment_body = world_->CreateBody(&body_def);
    segment_body->CreateFixture(&fixture_def);  // Attach fixture to body
    segment_body->SetUserData(&i);  // Reference this segment as user data
  }

  ROS_ERROR("Initialize world completed");
}
