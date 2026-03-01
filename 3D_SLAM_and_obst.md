o what reference is slam calculated? to global map?
is the poincloud used for any navigation or is it just visualized
can be poincloud used for some navigational purposes? obsicle detection? 3d global map?
These are conceptual questions — I can answer them directly from the codebase knowledge.

1. What reference is SLAM calculated to?

SLAM (slam_toolbox) publishes the map → odom transform. The chain is:


map  ←(SLAM corrects this)→  odom  ←(RF2O/odometry)→  base_link
odom is the local dead-reckoning frame — it drifts over time but is smooth
map is the global fixed frame — SLAM continuously corrects map→odom to minimize scan-to-map mismatch
So yes: the robot's absolute position is expressed in map (global), SLAM is the thing keeping it honest
2. Is the PointCloud2 used for navigation?

No — currently it is visualization only. Checking every navigation config:

slam_toolbox_async.yaml: scan_topic: "/scan" → LaserScan only
nav2_diffbot.yaml + nav2_slam.yaml: both costmaps use data_type: "LaserScan" on /scan
RF2O: also uses /scan_safe (LaserScan)
The /points topic is published by data_relay and displayed in RViz with 3s decay, nothing else subscribes to it.

3. Can PointCloud2 be used for navigation?

Yes, and it would be a meaningful upgrade. The Ouster OS-1 is a 32-ring 3D lidar — the LaserScan throws away 31 of those 32 rings, keeping only a single 2D horizontal slice. Options:

Use case	How	Benefit
3D obstacle detection	Add PointCloud2 as a second observation_source in Nav2 costmaps (data_type: "PointCloud2")	Detects low obstacles (kerbs, steps) and hanging obstacles that the 2D scan misses
3D SLAM / global map	Replace slam_toolbox with rtabmap or cartographer	Full volumetric map, loop closure over longer distances
Elevation-filtered obstacle layer	Keep slam_toolbox for 2D map, add Nav2 PointCloud2 layer with min/max_obstacle_height filter	Easy, keeps existing stack, adds height-aware obstacles
The easiest win is option 3: add /points as a second obstacle source in the Nav2 costmaps with a height filter (e.g., 0.05–1.5m) so it catches things the floor-level scan misses, while slam_toolbox continues to use the 2D scan. Would you want to plan that out?