#include "rclcpp/rclcpp.hpp"

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/ackermann_kinematics.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"

using namespace racecar_simulator;

class RacecarSimulator : public rclcpp:: Node {
  private:
    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // The car state and parameters
    Pose2D pose;
    double wheelbase;
    double speed;
    double steering_angle;
    double previous_seconds;
    double scan_distance_to_base_link;
    double max_speed, max_steering_angle;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;
    double map_free_threshold;

    // Joystick parameters
    int joy_speed_axis, joy_angle_axis;
    double joy_max_speed;

    // A ROS node
    //ros::NodeHandle n;
    //auto node = rclcppp::Node::make_shared("racecar_simulator"); 



    // For publishing transformations
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    //tf2_ros::TransformBroadcaster br;

    // A timer to update the pose
    //ros::Timer update_pose_timer;
    rclcpp::TimerBase::SharedPtr update_pose_timer;


    // Listen for drive and joystick commands
    // ros::Subscriber drive_sub;
    // ros::Subscriber joy_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    // Listen for a map
    //ros::Subscriber map_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    // ros::Subscriber pose_sub;
    // ros::Subscriber pose_rviz_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_rviz_sub;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    // ros::Publisher scan_pub;
    // ros::Publisher odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  public:

    RacecarSimulator() : Node("racecar_simulator") {
      // Initialize the node handle
      //n = ros::NodeHandle("~"); taken care of by node inheritance?
      br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      // Initialize the pose and driving commands
      pose = {0, 0, 0};
      speed = 0;
      steering_angle = 0;
      //previous_seconds = ros::Time::now().toSec();
      previous_seconds = rclcpp::Clock().now().seconds();


      // Declare parameters
      this->declare_parameter("joy_topic");
      this->declare_parameter("drive_topic");
      this->declare_parameter("map_topic");
      this->declare_parameter("scan_topic");
      this->declare_parameter("pose_topic");
      this->declare_parameter("odom_topic");
      this->declare_parameter("pose_rviz_topic");

      this->declare_parameter("map_frame");
      this->declare_parameter("base_frame");
      this->declare_parameter("scan_frame");
      this->declare_parameter("broadcast_transform");
      this->declare_parameter("joy");
      this->declare_parameter("joy_speed_axis");
      this->declare_parameter("joy_angle_axis");
      this->declare_parameter("joy_max_speec");

      this->declare_parameter("wheelbase");
      this->declare_parameter("max_speed");
      this->declare_parameter("max_steering_angle");
      this->declare_parameter("update_pose_rate");
      this->declare_parameter("scan_beams");
      this->declare_parameter("scan_field_of_view");
      this->declare_parameter("scan_distance_to_base_link");
      this->declare_parameter("scan_std_dev");
      this->declare_parameter("map_free_threshold");





      // Get the topic names
      std::string joy_topic, drive_topic, map_topic, 
        scan_topic, pose_topic, pose_rviz_topic, odom_topic;
      this->get_parameter("joy_topic", joy_topic);
      this->get_parameter("drive_topic", drive_topic);
      this->get_parameter("map_topic", map_topic);
      this->get_parameter("scan_topic", scan_topic);
      this->get_parameter("pose_topic", pose_topic);
      this->get_parameter("odom_topic", odom_topic);
      this->get_parameter("pose_rviz_topic", pose_rviz_topic);

  
      RCLCPP_INFO(this->get_logger(), "got this %s", joy_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "got this %s", drive_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "got this %s", map_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "got this %s", scan_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "got this %s", odom_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "got this %s", pose_rviz_topic.c_str());

      // Get the transformation frame names
      this->get_parameter("map_frame", map_frame);
      this->get_parameter("base_frame", base_frame);
      this->get_parameter("scan_frame", scan_frame);

      // Fetch the car parameters
      int scan_beams;
      double update_pose_rate, scan_field_of_view, scan_std_dev;
      this->get_parameter("wheelbase", wheelbase);
      this->get_parameter("update_pose_rate", update_pose_rate);
      this->get_parameter("scan_beams", scan_beams);
      this->get_parameter("scan_field_of_view", scan_field_of_view);
      this->get_parameter("scan_std_dev", scan_std_dev);
      this->get_parameter("map_free_threshold", map_free_threshold);
      this->get_parameter("scan_distance_to_base_link", scan_distance_to_base_link);
      this->get_parameter("max_speed", max_speed);
      this->get_parameter("max_steering_angle", max_steering_angle);

      // Get joystick parameters
      bool joy;
      // n.getParam("joy", joy);
      // n.getParam("joy_speed_axis", joy_speed_axis);
      // n.getParam("joy_angle_axis", joy_angle_axis);
      // n.getParam("joy_max_speed", joy_max_speed);
      this->get_parameter("joy", joy);
      this->get_parameter("joy_speed_axis", joy_speed_axis);
      this->get_parameter("joy_angle_axis", joy_angle_axis);
      this->get_parameter("joy_max_speed", joy_max_speed);

      // Determine if we should broadcast
      //n.getParam("broadcast_transform", broadcast_transform);
      this->get_parameter("broadcast_transform", broadcast_transform);


      // Initialize a simulator of the laser scanner
      scan_simulator = ScanSimulator2D(
          scan_beams,
          scan_field_of_view,
          scan_std_dev);

      // Make a publisher for laser scan messages
      //scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);
      scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 1);

      // Make a publisher for odometry messages
      //odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);
      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);

      // Start a timer to output the pose
      //update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose, this);
      update_pose_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(update_pose_rate * 1000)),
                                                std::bind(&RacecarSimulator::update_pose, this));

      // If the joystick is enabled
      if (joy)
        // Start a subscriber to listen to joystick commands
        //joy_sub = n.subscribe(joy_topic, 1, &RacecarSimulator::joy_callback, this);
        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic, 1, std::bind(&RacecarSimulator::joy_callback, this, std::placeholders::_1));


      // Start a subscriber to listen to drive commands
      //drive_sub = n.subscribe(drive_topic, 1, &RacecarSimulator::drive_callback, this);
      drive_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1, std::bind(&RacecarSimulator::drive_callback, this, std::placeholders::_1));


      // Start a subscriber to listen to new maps
      //map_sub = n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);
      map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 1, std::bind(&RacecarSimulator::map_callback, this, std::placeholders::_1));

      // Start a subscriber to listen to pose messages
      //pose_sub = n.subscribe(pose_topic, 1, &RacecarSimulator::pose_callback, this);
      //pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback, this);
      pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(pose_topic, 1, std::bind(&RacecarSimulator::pose_callback, this, std::placeholders::_1));
      pose_rviz_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_rviz_topic, 1, std::bind(&RacecarSimulator::pose_rviz_callback, this, std::placeholders::_1));
    }

    void update_pose() {

      // Update the pose
      //ros::Time timestamp = ros::Time::now();
      rclcpp::Time timestamp = this->now();
      
      //double current_seconds = timestamp.toSec();
      double current_seconds = timestamp.seconds();
      pose = AckermannKinematics::update(
          pose, 
          speed,
          steering_angle,
          wheelbase,
          current_seconds - previous_seconds);
      previous_seconds = current_seconds;

      // Convert the pose into a transformation
      geometry_msgs::msg::Transform t;
      t.translation.x = pose.x;
      t.translation.y = pose.y;
      tf2::Quaternion quat;
      quat.setEuler(0., 0., pose.theta);
      t.rotation.x = quat.x();
      t.rotation.y = quat.y();
      t.rotation.z = quat.z();
      t.rotation.w = quat.w();

      // Add a header to the transformation
      geometry_msgs::msg::TransformStamped ts;
      ts.transform = t;
      ts.header.stamp = timestamp;
      ts.header.frame_id = map_frame;
      ts.child_frame_id = base_frame;

      // Make an odom message as well
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = timestamp;
      odom.header.frame_id = map_frame;
      odom.child_frame_id = base_frame;
      odom.pose.pose.position.x = pose.x;
      odom.pose.pose.position.y = pose.y;
      odom.pose.pose.orientation.x = quat.x();
      odom.pose.pose.orientation.y = quat.y();
      odom.pose.pose.orientation.z = quat.z();
      odom.pose.pose.orientation.w = quat.w();
      odom.twist.twist.linear.x = speed;
      odom.twist.twist.angular.z = 
        AckermannKinematics::angular_velocity(speed, steering_angle, wheelbase);

      // Publish them
      if (broadcast_transform) br->sendTransform(ts);
      odom_pub->publish(odom);
      // Set the steering angle to make the wheels move
      set_steering_angle(steering_angle, timestamp);

      // If we have a map, perform a scan
      if (map_exists) {
        // Get the pose of the lidar, given the pose of base link
        // (base link is the center of the rear axle)
        Pose2D scan_pose;
        scan_pose.x = pose.x + scan_distance_to_base_link * std::cos(pose.theta);
        scan_pose.y = pose.y + scan_distance_to_base_link * std::sin(pose.theta);
        scan_pose.theta = pose.theta;

        // Compute the scan from the lidar
        std::vector<double> scan = scan_simulator.scan(scan_pose);

        // Convert to float
        std::vector<float> scan_(scan.size());
        for (size_t i = 0; i < scan.size(); i++)
          scan_[i] = scan[i];

        // Publish the laser message
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = timestamp;
        scan_msg.header.frame_id = scan_frame;
        scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
        scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
        scan_msg.angle_increment = scan_simulator.get_angle_increment();
        scan_msg.range_max = 100;
        scan_msg.ranges = scan_;
        scan_msg.intensities = scan_;

        scan_pub->publish(scan_msg);

        // Publish a transformation between base link and laser
        geometry_msgs::msg::TransformStamped scan_ts;
        scan_ts.transform.translation.x = scan_distance_to_base_link;
        scan_ts.transform.rotation.w = 1;
        scan_ts.header.stamp = timestamp;
        scan_ts.header.frame_id = base_frame;
        scan_ts.child_frame_id = scan_frame;
        br->sendTransform(scan_ts);
      }
    }

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
      pose.x = msg->position.x;
      pose.y = msg->position.y;
      geometry_msgs::msg::Quaternion q = msg->orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      pose.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr  msg) {
      pose_callback(std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose));
    }

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
      set_speed(msg->drive.speed);
      set_steering_angle(msg->drive.steering_angle, rclcpp::Clock().now());
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
      set_speed(
          joy_max_speed * msg->axes[joy_speed_axis]);
      set_steering_angle(
          max_steering_angle * msg->axes[joy_angle_axis],
          rclcpp::Clock().now());
    }

    void set_speed(double speed_) {
      speed = std::min(std::max(speed_, -max_speed), max_speed);
    }

    void set_steering_angle(double steering_angle_, rclcpp::Time timestamp) {
      steering_angle = std::min(std::max(steering_angle_, -max_steering_angle), max_steering_angle);

      // Publish the steering angle
      tf2::Quaternion quat;
      quat.setEuler(0., 0., steering_angle);
      geometry_msgs::msg::TransformStamped ts;
      ts.transform.rotation.x = quat.x();
      ts.transform.rotation.y = quat.y();
      ts.transform.rotation.z = quat.z();
      ts.transform.rotation.w = quat.w();
      ts.header.stamp = timestamp;
      ts.header.frame_id = "front_left_hinge";
      ts.child_frame_id = "front_left_wheel";
      br->sendTransform(ts);
      ts.header.frame_id = "front_right_hinge";
      ts.child_frame_id = "front_right_wheel";
      br->sendTransform(ts);
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      // Fetch the map parameters
      size_t height = msg->info.height;
      size_t width = msg->info.width;
      double resolution = msg->info.resolution;
      // Convert the ROS origin to a pose
      Pose2D origin;
      origin.x = msg->info.origin.position.x;
      origin.y = msg->info.origin.position.y;
      geometry_msgs::msg::Quaternion q = msg->info.origin.orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      origin.theta = tf2::impl::getYaw(quat);

      // Convert the map to probability values
      std::vector<double> map(msg->data.size());
      for (size_t i = 0; i < height * width; i++) {
        if (msg->data[i] > 100 or msg->data[i] < 0) {
          map[i] = 0.5; // Unknown
        } else {
          map[i] = msg->data[i]/100.;
        }
      }

      // Send the map to the scanner
      scan_simulator.set_map(
          map,
          height,
          width,
          resolution,
          origin,
          map_free_threshold);
      map_exists = true;
    }
};

int main(int argc, char ** argv) {
  //ros::init(argc, argv, "racecar_simulator");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RacecarSimulator>();
  //RacecarSimulator rs;
  //ros::spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}