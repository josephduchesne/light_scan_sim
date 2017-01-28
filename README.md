# light_scan_sim

A very lightweight 2d laser scan simulator for ROS.

This is designed as an alternative to running Gazebo, or Player Stage. 

Here's a basic rundown:
- Input: a tf transform representing your laser's position (base_laser_link or laser_joint on a TurtleBot for example)
- Input: An OccupancyGrid (you can publish this using a [map_server](http://wiki.ros.org/map_server) node)
- Output: A [LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) against the OccupancyGrid

Features:
- Gaussian noise (optional, disable by setting laser/noise to 0)
- ~~Unit tests~~
- ~~Full RosParam Configuration~~
- ~~Vector obstacles~~
- ~~Glass~~
- ~~Reflectivity~~

## License

MIT License

Copyright (c) 2017 Joseph Duchesne

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
