# Currently Under Development

**1. INSTRUCTION:**

* Use amcl.launch for navaigation using odometry provided by NAO robot.
* Use amcl_scanmatch.launch for navigation using laser scan data as odometry.
* Use gtest.launch for mapping using gmapping
* Use htest.launch for mapping using hector mapping
* -[x] In the launch files make sure to change the paths to _config_, _map_, and _rviz_ files accordingly.

* To use gmapping and hector mapping, joint_states.py needs to be changed (uncomment ln160) 
 
**2. KNOWN BUGS:**

 1. Move_base demonstrates weird behaviors (seemingly unnecessary rotation).
 
 2. Laser_scan_matcher loses target when robot is rotated in place of approximately 90 degrees or more.
 
 3. Error message "Error while transforming pose: "base_footprint" passed to lookupTransform argument target_frame does not exist." when assigning a goal in rviz.
 
 4. For larger maps, path planner is not able to plan a complete (only a short segment).
