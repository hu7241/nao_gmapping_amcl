------ Currently Under Development -----------

INSTRUCTION:
 
Use amcl.launch for navaigation using odometry provided by NAO robot.
Use amcl_scanmatch.launch for navigation using laser scan data as odometry.
Use gtest.launch for mapping using gmapping
use htest.launch for mapping using hector mapping

In the launch files make sure to change the paths to config files accordingly.

KNOWN BUGS:
1. Move_base demonstrates weird behaviors (seemingly unnecessary rotation).
2. Laser_scan_matcher loses target when robot is rotated in place of approximately 90 degrees or more.
3. Error while transforming pose: "base_footprint" passed to lookupTransform argument target_frame does not exist. 
 
