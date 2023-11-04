This code subscribes to a pointcloud topic in ROS2. The pointcloud data is then preprocessed.
The closest object infront of the robot will be chosen as the object that will be reconstructed.
First the centroid of the object will be calculated, then goal points around the object will be generated. 
The robot will move to the goal points around the object and will create pcd files at these goal points.
In the end, the robot will merge all the pcd files into one file which will include a 3d point cloud reconstruction for the object.
