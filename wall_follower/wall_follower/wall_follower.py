#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from wall_follower.RANSAC import *
from wall_follower.visualization_tools import *


class WallFollower(Node):

    
    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")


        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        self.get_logger().debug('My log message %s' % (self.DRIVE_TOPIC))

        self.WALL_TOPIC = "/wall"
        self.P = 1.5
        self.D = 3
        self.LOOK_AHEAD_DIST = 2
        self.NUM_RANSAC_ITERATIONS = 100
        self.RANSAC_EPSILON = 0.1
        
        # A publisher for the visualization
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)
    
        # A publisher for navigation commands
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 1)
        
        # A subscriber to laser scans
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 10)

    def laser_callback(self, laser_scan):
        """
        Using laser data, this function computes
        and publishes a navigation command for the
        racecar which moves the racecar forward, while
        maintaining a set distance from either the left
        or right wall.
        It also publishes a visualization of the 
        detected wall for display in rviz.
        Args:
            laser_scan: a ros LaserScan message.
            See "rosmsg info LaserScan" for more
            information.
        """

        # Draw a parabola as a demo
        # x = np.linspace(-2.,2.,num=20)
        # y = np.square(x)
        # VisualizationTools.plot_line(x, y, self.line_pub, frame = "/laser")
        # return
        ranges = np.array(laser_scan.ranges, dtype='float32')

        # Compute an array of angles
        angles = np.linspace(
                laser_scan.angle_min,
                laser_scan.angle_max,
                num = ranges.shape[0])
        # TODO:
        # angles[1] - angles[0] != laser_scan.angle_increment
        # An off by 1 error with the number of ranges

        # Convert the ranges to Cartesian coordinates.
        # Consider the robot to be facing in the
        # positive x direction.
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        xy = np.concatenate((
            np.expand_dims(x, -1),
            np.expand_dims(y, -1)),
            axis=1)

        # Filter out values that are out of range
        # and values on the wrong side
        valid_points = self.SIDE * y > 0
        valid_points = np.logical_and(
                valid_points,
                x < self.LOOK_AHEAD_DIST)
        valid_points = np.logical_and(
                valid_points,
                x > 0)
        valid_points = np.logical_and(
                valid_points, 
                ranges < laser_scan.range_max - 0.2)
        valid_points = np.logical_and(
                valid_points, 
                ranges >= laser_scan.range_min)

        xy = xy[valid_points]
        if xy.shape[0] == 0: return

        # Perform RANSAC on the points to find
        # a line that describes them.
        a, b, c = RANSAC.run(
                xy,
                self.NUM_RANSAC_ITERATIONS,
                self.RANSAC_EPSILON)

        # Plot the line.
        x, y = self.abc_to_points(a, b, c, xy)
        VisualizationTools.plot_line(x, y, self.line_pub, frame = "/laser")

        # Extract the y-intersect and
        # it's derivative.
        y0 = -c/b
        tan_theta = a/b
        dy0dt = -self.VELOCITY * tan_theta

        # Use a PD controller.
        steering_angle = \
                self.P * (y0 - self.SIDE * self.DESIRED_DISTANCE) + \
                self.D * dy0dt

        # Create a drive command.
        drive_forward = AckermannDriveStamped()
        drive_forward.drive.speed = self.VELOCITY
        drive_forward.drive.steering_angle = steering_angle

        # Publish the drive command.
        self.drive_pub.publish(drive_forward)

    def abc_to_points(self, a, b, c, xy):
        """
        This function computes points on the
        line ax + by + c that are within the
        region containing the points xy.
        Args:
            a, b, c: Parameters that define
            the line ax + by + c = 0
            xy: An array of coordinate tuples.
        
        Returns:
            x: An array of x coordinates.
            y: An array of y coordinates.
        """
        # Create a bounding box around the points.
        xy_min = np.amin(xy, axis=0) - self.RANSAC_EPSILON
        xy_max = np.amax(xy, axis=0) + self.RANSAC_EPSILON

        # Find the intersection points with the
        # line and the bounding box.
        intersect_points = []
        intersect_points.append([xy_min[0], -(a * xy_min[0] + c)/b])
        intersect_points.append([xy_max[0], -(a * xy_max[0] + c)/b])
        intersect_points.append([-(b * xy_min[1] + c)/a, xy_min[1]])
        intersect_points.append([-(b * xy_max[1] + c)/a, xy_max[1]])
        intersect_points = np.array(intersect_points)

        # Filter out intersection points that
        # are outside the bounding box.
        within_lower = intersect_points >= xy_min
        within_upper = intersect_points <= xy_max
        valid_points = np.logical_and(
                np.all(within_lower, axis=1),
                np.all(within_upper, axis=1))

        intersect_points = intersect_points[valid_points]

        # Return separate x and y lists.
        return intersect_points[:,0], intersect_points[:,1]


def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    