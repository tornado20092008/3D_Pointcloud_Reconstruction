#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <chrono> // Add this header for the std::chrono functions
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/obj_io.h>

class PCL_Euk : public rclcpp::Node
{
public:
  PCL_Euk() : Node("pcl_euk"), closest_centroid_found_(false), current_goal_index_(0), moving_towards_goal_(false), reached_last_goal_(true)
  {
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(  // Change the subscription type to PointCloud2
  "/zed2/zed_node/point_cloud/cloud_registered", 10, std::bind(&PCL_Euk::cloud_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("extracted_clusters", 10);
  centroid_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("centroid", 10);
  goal_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("goal_points", 10);
  goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
  goal_reached_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&PCL_Euk::publish_next_goal_and_check_reached, this));
  colored_cloud= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  // Create the transform-related objects
  try {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "Exception during shared pointer initialization: %s", ex.what());
  }


  // Subscribe to the odom topic
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  "/zed2/zed_node/odom", 10, std::bind(&PCL_Euk::odom_callback, this, std::placeholders::_1));

  // Initialize the current robot pose
  robot_current_pose_.position.x = 0.0;
  robot_current_pose_.position.y = 0.0;
  robot_current_pose_.position.z = 0.0;
  robot_current_pose_.orientation.x = 0.0;
  robot_current_pose_.orientation.y = 0.0;
  robot_current_pose_.orientation.z = 0.0;
  robot_current_pose_.orientation.w = 1.0;
  }

private:
  // Add member variables to store the current robot pose and the centroid
  geometry_msgs::msg::Pose robot_current_pose_;
  sensor_msgs::msg::PointCloud2::SharedPtr msg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr goal_points_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> closest_centroid_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud; // Declare colored_cloud as a member variable
  std::vector<std::string> pcd_filenames; // Create a vector to store filenames
  bool closest_centroid_found_;
  bool timer_elapsed_;
  std::chrono::seconds wait_duration_;
  size_t current_goal_index_;
  bool moving_towards_goal_;
  bool reached_last_goal_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centroid_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr goal_points_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::TimerBase::SharedPtr goal_reached_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
  if (!reached_last_goal_)
  {
  RCLCPP_INFO(this->get_logger(), "Waiting to reach the last goal point.");
  return;
  }
    this->msg = msg; // Storing the received msg

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*this->msg, *cloud); // Using this->msg


    // Downsample the point cloud using a voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_downsampled);

		//debuggimg1
		std::cout<< "number of points in downsampled cloud: " << cloud_downsampled->size() <<std::endl;


		//perform statistical outlier removal
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud_downsampled);
		sor.setMeanK(10);
		sor.setStddevMulThresh(0.001);
		sor.filter(*cloud_downsampled);


		//debuggimg1
		std::cout<< "number of points after stat. outlier: " << cloud_downsampled->size() <<std::endl;


		pcl::PointCloud<pcl::PointXYZ>::Ptr ground (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_downsampled));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
		ransac.setDistanceThreshold (0.05);
		ransac.computeModel();


		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		ransac.getInliers (inliers->indices);
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_downsampled);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*ground);

		extract.setNegative (true);
		extract.filter (*cloud_downsampled);


		//filter the ground out
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr ground (new pcl::PointCloud<pcl::PointXYZ>);*/
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud_downsampled);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(-0.6,1.5);  //keep points within this range
		pass.filter(*cloud_downsampled);

		//debuggimg1
		std::cout<< "number of points after filtering ground: " << cloud_downsampled->size() <<std::endl;

    // Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_downsampled);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); // 20cm
    ec.setMinClusterSize(40);
    ec.setMaxClusterSize(40000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_downsampled);
    ec.extract(cluster_indices);


		std::cout<< "number of clusters: " << cluster_indices.size() <<std::endl;

  // Check if any clusters were found
  if (cluster_indices.empty()) {
      RCLCPP_INFO(this->get_logger(), "No clusters found.");
      return;
  }




	// Create a new point cloud containing only the extracted clusters

	colored_cloud->header.frame_id = msg->header.frame_id;
	closest_centroid_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	float closest_distance_squared = std::numeric_limits<float>::max();
	int current_cluster = 0;
	const int num_clusters = cluster_indices.size();
	const float hue_range = 360.0f / (float)num_clusters;
	for (const auto& indices : cluster_indices)
	{
		  float hue = current_cluster * hue_range;
		  pcl::PointXYZ centroid(0, 0, 0); // Centroid of the cluster

		  for (const auto& index : indices.indices)
		  {
		      pcl::PointXYZRGB point;
		      point.x = cloud_downsampled->points[index].x;
		      point.y = cloud_downsampled->points[index].y;
		      point.z = cloud_downsampled->points[index].z;

		      // Convert HSV color to RGB
		      float saturation = 1.0f;
		      float value = 1.0f;
		      float c = value * saturation;
		      float x = c * (1 - fabs(fmod(hue / 60.0f, 2) - 1));
		      float m = value - c;
		      if (hue < 60.0f) {
		          point.r = 255 * (c + m);
		          point.g = 255 * (x + m);
		          point.b = 255 * (0 + m);
		      } else if (hue < 120.0f) {
		          point.r = 255 * (x + m);
		          point.g = 255 * (c + m);
		          point.b = 255 * (0 + m);
		      } else if (hue < 180.0f) {
		          point.r = 255 * (0 + m);
		          point.g = 255 * (c + m);
		          point.b = 255 * (x + m);
		      } else if (hue < 240.0f) {
		          point.r = 255 * (0 + m);
		          point.g = 255 * (x + m);
		          point.b = 255 * (c + m);
		      } else if (hue < 300.0f) {
		          point.r = 255 * (x + m);
		          point.g = 255 * (0 + m);
		          point.b = 255 * (c + m);
		      } else {
		          point.r = 255 * (c + m);
		          point.g = 255 * (0 + m);
		          point.b = 255 * (x + m);
		      }

		      colored_cloud->push_back(point);

          

		      // Calculate centroid
		      centroid.x += point.x;
		      centroid.y += point.y;
		      centroid.z += point.z;
		  }

		  // Average the centroid coordinates
		  centroid.x /= indices.indices.size();
		  centroid.y /= indices.indices.size();
		  centroid.z /= indices.indices.size();

		  // Check if this centroid is closer than the current closest centroid
		  float distance_squared = centroid.x * centroid.x + centroid.y * centroid.y;
		  if (distance_squared < closest_distance_squared)
		  {
		      closest_centroid_->points.clear();
		      closest_centroid_->push_back(centroid);
		      closest_distance_squared = distance_squared;
		  }

		  current_cluster++;
	}

  geometry_msgs::msg::PointStamped centroid_base_link;
  centroid_base_link.header.frame_id = "zed2_left_camera_frame"; // Assuming the centroid coordinates are in the "base_link" frame
  centroid_base_link.point.x = closest_centroid_->points[0].x;
  centroid_base_link.point.y = closest_centroid_->points[0].y;
  centroid_base_link.point.z = closest_centroid_->points[0].z;

  try
  {
  geometry_msgs::msg::PointStamped centroid_map;
  tf_buffer_->transform(centroid_base_link, centroid_map, "map");

  // Now, use the transformed coordinates for further calculations
  closest_centroid_->points[0].x = centroid_map.point.x;
  closest_centroid_->points[0].y = centroid_map.point.y;
  closest_centroid_->points[0].z = centroid_map.point.z;

  // Publish the colored point cloud in the "map" frame
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*colored_cloud, output);
  output.header = msg->header;
  publisher_->publish(output);


  // Publish the closest centroid point cloud in the "map" frame
  sensor_msgs::msg::PointCloud2 centroid_output;
  pcl::toROSMsg(*closest_centroid_, centroid_output);
  centroid_output.header.frame_id = "map";  // Set the frame ID to "map"
  centroid_publisher_->publish(centroid_output);


  if (!closest_centroid_found_)
       {
           // Calculate the maximum distance from the centroid to the object's perimeter
           double max_distance = 0.0;
           for (const auto& indices : cluster_indices)
           {
              // Calculate centroid of the current cluster
              pcl::PointXYZ centroid(0, 0, 0);
              for (const auto& index : indices.indices)
              {
              centroid.x += cloud_downsampled->points[index].x;
              centroid.y += cloud_downsampled->points[index].y;
              centroid.z += cloud_downsampled->points[index].z;
              }
              centroid.x /= indices.indices.size();
              centroid.y /= indices.indices.size();
              centroid.z /= indices.indices.size();

              // Calculate the furthest distance from the centroid within the current cluster
              for (const auto& index : indices.indices)
              {
              double dx = cloud_downsampled->points[index].x - centroid.x;
              double dy = cloud_downsampled->points[index].y - centroid.y;
              double distance = std::sqrt(dx * dx + dy * dy);
              if (distance > max_distance)
              {
              max_distance = distance;
              }
              }
           }
       
  // Calculate the desired spacing between goal points (adjust this value)
  double desired_spacing = 0.7; // Adjust as needed
  max_distance = max_distance/1.3;
  // Determine the number of goal points based on the maximum distance
  int min_num_points = 8; // Minimum number of points
  int max_num_points = static_cast<int>(max_distance / desired_spacing); // Maximum number of points based on spacing
  max_num_points = std::max(max_num_points, min_num_points); // Ensure a minimum number of points

  // Print the calculated num_points
  RCLCPP_INFO(this->get_logger(), "Calculated num_points: %d", max_num_points);

  // Calculate the angle increment between goal points
  double angle_increment = 2 * M_PI / max_num_points;


         pcl::PointCloud<pcl::PointXYZI>::Ptr goal_points(new pcl::PointCloud<pcl::PointXYZI>); // Use PointXYZI to store intensity (yaw angle)
         for (int i = 0; i < max_num_points; ++i)
         {
          double angle = i * angle_increment;
          pcl::PointXYZI goal_point;
          goal_point.x = closest_centroid_->points[0].x + max_distance * cos(angle);
          goal_point.y = closest_centroid_->points[0].y + max_distance * sin(angle);
          goal_point.z = closest_centroid_->points[0].z;

          // Calculate the yaw angle (angle between goal_point and centroid)
          double dx = goal_point.x - closest_centroid_->points[0].x;
          double dy = goal_point.y - closest_centroid_->points[0].y;
          double yaw_angle = atan2(dy, dx);
          yaw_angle += M_PI;
          goal_point.intensity = yaw_angle; // Save the yaw angle in the intensity field

          goal_points->push_back(goal_point);

          // Print the coordinates and yaw angle of the goal points
          std::cout << "Goal Point " << i << " (Map): " << goal_point.x << ", " << goal_point.y << ", " << goal_point.z
          << ", Yaw Angle: " << yaw_angle << std::endl;
         }
  // Publish the goal points as a new PointCloud
  sensor_msgs::msg::PointCloud2 goal_points_output;
  pcl::toROSMsg(*goal_points, goal_points_output);
  goal_points_output.header.frame_id = "map";  // Set the frame ID to "map"
  goal_points_publisher_->publish(goal_points_output);

  closest_centroid_found_ = true;
  current_goal_index_ = 0;
  moving_towards_goal_ = false;
  goal_points_ = goal_points;
  }
  }
  catch (tf2::TransformException& ex)
  {
  RCLCPP_ERROR(this->get_logger(), "Failed to transform the centroid coordinates: %s", ex.what());
  return;
  }
  }

  void publish_next_goal_and_check_reached()
  {
  if (goal_points_ && current_goal_index_ < goal_points_->size())
  {
  // Update the robot's current pose before checking if the goal is reached
	geometry_msgs::msg::PoseStamped robot_pose_base_link;
	robot_pose_base_link.header.frame_id = "odom"; // Assuming the pose is in the "base_link" frame
	robot_pose_base_link.pose = robot_current_pose_; // Assign the current robot pose

	geometry_msgs::msg::PoseStamped robot_pose_map; // Declare robot_pose_map outside the try block

	try
	{
		  // Transform the robot's pose to the "map" frame
		  tf_buffer_->transform(robot_pose_base_link, robot_pose_map, "map");


		  // Do something with the transformed pose if needed
	}
	catch (tf2::TransformException& ex)
	{
		  RCLCPP_ERROR(this->get_logger(), "Failed to transform the robot's pose: %s", ex.what());
		  // Handle the transformation failure if necessary
	}




  if (!moving_towards_goal_)
  {
  // Print the coordinates of the goal point in the map frame
  std::cout << "Goal Point (Map): " << goal_points_->points[current_goal_index_].x << ", "
  << goal_points_->points[current_goal_index_].y << ", " << goal_points_->points[current_goal_index_].z << std::endl;

         // Publish the next goal pose in the map frame
         geometry_msgs::msg::PoseStamped goal_pose_map;
         goal_pose_map.header.frame_id = "map";
         goal_pose_map.pose.position.x = goal_points_->points[current_goal_index_].x;
         goal_pose_map.pose.position.y = goal_points_->points[current_goal_index_].y;
         goal_pose_map.pose.position.z = goal_points_->points[current_goal_index_].z;

         // Retrieve the yaw angle from the intensity field and set it as the orientation
         double yaw_angle = goal_points_->points[current_goal_index_].intensity;
         tf2::Quaternion quaternion;
         quaternion.setRPY(0.0, 0.0, yaw_angle);
         goal_pose_map.pose.orientation.x = quaternion.x();
         goal_pose_map.pose.orientation.y = quaternion.y();
         goal_pose_map.pose.orientation.z = quaternion.z();
         goal_pose_map.pose.orientation.w = quaternion.w();
     
         // Debugging cout lines
         std::cout << "Goal Pose (Map): x=" << goal_pose_map.pose.position.x << ", y=" << goal_pose_map.pose.position.y << ", z=" << goal_pose_map.pose.position.z << std::endl;
         std::cout << "Goal Points (Map): x=" << goal_points_->points[current_goal_index_].x << ", y=" << goal_points_->points[current_goal_index_].y << ", z=" << goal_points_->points[current_goal_index_].z << std::endl;

         goal_pose_publisher_->publish(goal_pose_map);

         moving_towards_goal_ = true;
  }

					else {
                            // Check if the robot has reached the goal
                            double position_tolerance = 0.35; // Adjust this value as per your requirement
                            double orientation_tolerance = 0.25; // Adjust this value as per your requirement

                            // Calculate position error
                            double dx = robot_pose_map.pose.position.x - goal_points_->points[current_goal_index_].x;
                            double dy = robot_pose_map.pose.position.y - goal_points_->points[current_goal_index_].y;
                            double position_distance = std::sqrt(dx * dx + dy * dy);

                            // Calculate orientation error
                            double goal_yaw_angle = goal_points_->points[current_goal_index_].intensity;
                            // Extract the quaternion components
														double qx = robot_pose_map.pose.orientation.x;
														double qy = robot_pose_map.pose.orientation.y;
														double qz = robot_pose_map.pose.orientation.z;
														double qw = robot_pose_map.pose.orientation.w;

														// Calculate the yaw angle (rotation around the Z-axis)
														double current_yaw_angle = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

                            double orientation_error = goal_yaw_angle - current_yaw_angle;

                            // Adjust orientation error to be in the range [-pi, pi]
                            if (orientation_error > M_PI)
                            {
                                orientation_error -= 2 * M_PI;
                            }
                            else if (orientation_error < -M_PI)
                            {
                                orientation_error += 2 * M_PI;
                            }

                            if (position_distance <= position_tolerance && std::abs(orientation_error) <= orientation_tolerance)
                            {
                                RCLCPP_INFO(this->get_logger(), "Robot has reached the goal point: (%f, %f)", goal_points_->points[current_goal_index_].x, goal_points_->points[current_goal_index_].y);
                                current_goal_index_++;
                                moving_towards_goal_ = false;
																	// Call the function to save the point cloud when the goal is reached
																  if (this->msg)
																	{
																		savePointCloudCallback(this->msg, *tf_buffer_);
									 							}									

																	// Check if there are more goals to reach
																	if (current_goal_index_ >= goal_points_->size()) {
																			reached_last_goal_ = true;
																			closest_centroid_found_ = false; // Reset the flag to find the next closest centroid and goal points;
																	 
																	 
																			// Merging PCD files
																			pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

																			for (const std::string& filename : pcd_filenames) {
																					RCLCPP_INFO(this->get_logger(), "Reading PCD file: %s", filename.c_str()); // Print the filename being read
																					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
																					if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
																							RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", filename.c_str());
																							continue;
																					}
																					*merged_cloud += *cloud;
																			}

																			// Save the merged point cloud to a new PCD file
																			std::string merged_pcd_filename = "merged_point_cloud.pcd";
																			pcl::io::savePCDFileASCII(merged_pcd_filename, *merged_cloud);
																			RCLCPP_INFO(this->get_logger(), "Saved merged point cloud data to %s", merged_pcd_filename.c_str());

																			// Load the merged point cloud from the saved PCD file
																			pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
																			if (pcl::io::loadPCDFile<pcl::PointXYZ>(merged_pcd_filename, *loaded_merged_cloud) == -1) {
																					RCLCPP_ERROR(this->get_logger(), "Couldn't read merged PCD file %s", merged_pcd_filename.c_str());
																					return;

																			
																			}
																			else {

																					// Publish the next goal pose
																					geometry_msgs::msg::PoseStamped goal_pose_map;
																					goal_pose_map.header.frame_id = "map";
																					goal_pose_map.pose.position.x = goal_points_->points[current_goal_index_].x;
																					goal_pose_map.pose.position.y = goal_points_->points[current_goal_index_].y;
																					goal_pose_map.pose.position.z = goal_points_->points[current_goal_index_].z;
																					goal_pose_map.pose.orientation.x = 0.0;
																					goal_pose_map.pose.orientation.y = 0.0;
																					goal_pose_map.pose.orientation.z = 0.0;
																					goal_pose_map.pose.orientation.w = 1.0;

																					goal_pose_publisher_->publish(goal_pose_map);

																					// Print the coordinates of the next goal point in the map frame
																					std::cout << "Next Goal Point (Map): " << goal_points_->points[current_goal_index_].x << ", "
																					<< goal_points_->points[current_goal_index_].y << ", " << goal_points_->points[current_goal_index_].z << std::endl;

																			}
																	 }
																	} 
													else {
															RCLCPP_INFO(this->get_logger(), "Robot is at position: (%f, %f). Distance to goal: %f", robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, position_distance);
													}
												

      }
    }  
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
  // Update the robot's current pose with the odometry data
  robot_current_pose_ = msg->pose.pose;
  }
  
	void savePointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, tf2_ros::Buffer& tf_buffer)
	{
		  static int index = 0; // Variable to keep track of frame index
		  std::cout << "now will take msg to cloud " << std::endl;
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		  pcl::fromROSMsg(*msg, *cloud);

		  // Apply Passthrough filter in the X dimension
		  pcl::PassThrough<pcl::PointXYZ> passx;
		  passx.setInputCloud(cloud);
		  passx.setFilterFieldName("x");
		  passx.setFilterLimits(0, 2); // Adjust these limits as needed
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZ>);
		  passx.filter(*cloud_filtered_xy);

		  // Apply Passthrough filter in the Y dimension
		  pcl::PassThrough<pcl::PointXYZ> passy;
		  passy.setInputCloud(cloud_filtered_xy);
		  passy.setFilterFieldName("y");
		  passy.setFilterLimits(-1, 1); // Adjust these limits as needed
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		  passy.filter(*cloud_filtered_xyz);


		  pcl::toROSMsg(*cloud_filtered_xyz, *msg);
		  // Transform the point cloud to the map frame
		  try
		  {
		      geometry_msgs::msg::TransformStamped transform_stamped;
		      transform_stamped = tf_buffer.lookupTransform("map", msg->header.frame_id, msg->header.stamp);

		      sensor_msgs::msg::PointCloud2 transformed_cloud;
		      tf2::doTransform(*msg, transformed_cloud, transform_stamped);

		      pcl::fromROSMsg(transformed_cloud, *cloud_filtered_xyz); // Transform the already filtered cloud
		  }
		  catch (tf2::TransformException& ex)
		  {
		      RCLCPP_ERROR(rclcpp::get_logger("point_cloud_transform"), ex.what());
		      return;
		  }

		  // Apply Z and Voxel Grid filters
		  pcl::PassThrough<pcl::PointXYZ> passz;
		  passz.setInputCloud(cloud_filtered_xyz);
		  passz.setFilterFieldName("z");
		  passz.setFilterLimits(0.04, 1.2); // Keep points within this range
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		  passz.filter(*cloud_filtered);

		  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		  voxel_filter.setInputCloud(cloud_filtered);
		  voxel_filter.setLeafSize(0.01, 0.01, 0.01); // Set the leaf size as per your requirement
		  voxel_filter.filter(*cloud_filtered);

		  // Perform statistical outlier removal
		  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		  sor.setInputCloud(cloud_filtered);
		  sor.setMeanK(10);
		  sor.setStddevMulThresh(0.001);
		  sor.filter(*cloud_filtered);

		  std::string frames_dir = "frames";
		  mkdir(frames_dir.c_str(), 0777); // Create 'frames' directory if it doesn't exist

		  std::stringstream ss;
		  ss << frames_dir << "/frame_" << std::setfill('0') << std::setw(6) << index; // Generate file name with index
		  std::string frames = ss.str();

		  pcl::io::savePCDFileASCII(frames + ".pcd", *cloud_filtered);

		  std::cout << "Saved " << cloud_filtered->size() << " data points to " << frames << ".pcd." << std::endl;
		  // Store the file name in the vector
   	  pcd_filenames.push_back(frames + ".pcd");

		  index++;
	}


  
  
  
  
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto pcl_euk_node = std::make_shared<PCL_Euk>();
  rclcpp::spin(pcl_euk_node);
  rclcpp::shutdown();
  return 0;
}
