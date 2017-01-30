#!/usr/bin/env python  
# Reads material definitions and publishes visualization message of segment list

import roslib
roslib.load_manifest("light_scan_sim")

import rospy
from light_scan_sim.msg import SegmentList
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3

markers = MarkerArray()

# Upate the global pose
def segment_callback(data):
    global markers

    markers.markers = []

    for i in range(len(data.segments)):
      segment = data.segments[i]

      # Generate marker for each segment
      m = Marker();
      m.header.frame_id = data.frame_id
      m.header.stamp = rospy.get_rostime()
      m.id = i
      m.type = 4  # Line list
      m.pose.orientation.x = 0
      m.pose.orientation.y = 0
      m.pose.orientation.z = 0
      m.pose.orientation.w = 1.0
      m.scale.x = 0.05
      m.scale.y = 0.05
      m.scale.z = 0.05
      m.action = 0
      m.points.append(Vector3(segment.start[0], segment.start[1], 0))
      m.points.append(Vector3(segment.end[0], segment.end[1], 0))
      m.color.a = 1.0
      markers.markers.append(m)

    print "Got segment list with %d segments" % len(markers.markers)

# Publish the 
if __name__ == '__main__':
    rospy.init_node('visualize_segment_list')

    # Todo: Load the material list

    rospy.Subscriber(rospy.get_param('~input_topic', '/segments'), SegmentList, segment_callback)
    pub = rospy.Publisher(rospy.get_param('~output_topic', '/segments_vis'), MarkerArray, queue_size=1)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        # Publish the MarkerArray
        pub.publish(markers)
        rate.sleep()
