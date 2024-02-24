#!/usr/bin/env python3


import rclpy
from std_msgs.msg import Float64
from rclpy.node import Node
# import some utils.
import numpy as np
import copy as copy

class InterpolateThrottle(Node):
    def __init__(self):

        super().__init__('throttle_interpolator')

        # Declare all parameters before accessing them
        self.declare_parameter("rpm_input_topic", "/vesc/commands/motor/unsmoothed_speed")
        self.declare_parameter("rpm_output_topic", "/vesc/commands/motor/speed")
        self.declare_parameter("servo_input_topic", "/vesc/commands/servo/unsmoothed_position")
        self.declare_parameter("servo_output_topic", "/vesc/commands/servo/position")
        self.declare_parameter("/vesc/max_acceleration", "default")
        self.declare_parameter("/vesc/vesc_driver/speed_max", "default")
        self.declare_parameter("/vesc/vesc_driver/speed_min", "default")
        self.declare_parameter("/vesc/throttle_smoother_rate", "default")
        self.declare_parameter("/vesc/speed_to_erpm_gain", "default")
        self.declare_parameter("/vesc/max_servo_speed", "default")
        self.declare_parameter("/vesc/steering_angle_to_servo_gain", "default")
        self.declare_parameter("/vesc/servo_smoother_rate", "default")
        self.declare_parameter("/vesc/vesc_driver/servo_max", "default")
        self.declare_parameter("/vesc/vesc_driver/servo_min", "default")
        self.declare_parameter("/vesc/steering_angle_to_servo_offset", "default")
        

        # Allow our topics to be dynamic
        self.rpm_input_topic = self.get_parameter("rpm_input_topic").get_parameter_value().string_value
        self.rpm_output_topic = self.get_parameter("rpm_output_topic").get_parameter_value().string_value
    

        self.servo_input_topic = self.get_parameter("servo_input_topic").get_parameter_value().string_value
        self.servo_output_topic = self.get_parameter("servo_output_topic").get_parameter_value().string_value

        self.max_acceleration = self.get_parameter('/vesc/max_acceleration').get_parameter_value().double_value
        self.max_rpm = self.get_parameter('/vesc/vesc_driver/speed_max').get_parameter_value().double_value
        self.min_rpm = self.get_parameter('/vesc/vesc_driver/speed_min').get_parameter_value().double_value
        self.throttle_smoother_rate = self.get_parameter('/vesc/throttle_smoother_rate').get_parameter_value().double_value
        self.speed_to_erpm_gain = self.get_parameter('/vesc/speed_to_erpm_gain').get_parameter_value().double_value

        self.max_servo_speed = self.get_parameter('/vesc/max_servo_speed').get_parameter_value().double_value
        self.steering_angle_to_servo_gain = self.get_parameter('/vesc/steering_angle_to_servo_gain').get_parameter_value().double_value
        self.servo_smoother_rate = self.get_parameter('/vesc/servo_smoother_rate').get_parameter_value().double_value
        self.max_servo = self.get_parameter('/vesc/vesc_driver/servo_max').get_parameter_value().double_value
        self.min_servo = self.get_parameter('/vesc/vesc_driver/servo_min').get_parameter_value().double_value

        # Variables
        self.last_rpm = 0
        self.desired_rpm = self.last_rpm
        
        self.last_servo = self.get_parameter('/vesc/steering_angle_to_servo_offset').get_parameter_value().double_value
        self.desired_servo_position = self.last_servo

        # Create topic subscribers and publishers
        self.rpm_output = self.create_publisher(Float64, self.rpm_output_topic, 1)
        self.servo_output = self.create_publisher(Float64, self.servo_output_topic, 1)
        
        self.create_subscription(Float64, self.rpm_input_topic, self._process_throttle_command, 1)
        self.create_subscription(Float64, self.servo_input_topic, self._process_servo_command, 1)

        self.max_delta_servo = abs(self.steering_angle_to_servo_gain * self.max_servo_speed / self.servo_smoother_rate)
        self.timer1 = self.create_timer(1/self.servo_smoother_rate, self._publish_servo_command)
        #rospy.Timer(rospy.Duration(1.0/self.servo_smoother_rate), self._publish_servo_command)


        self.max_delta_rpm = abs(self.speed_to_erpm_gain * self.max_acceleration / self.throttle_smoother_rate)
        self.timer2 = self.create_timer(1/self.max_delta_rpm, self._publish_throttle_command)        
        #rospy.Timer(rospy.Duration(1.0/self.max_delta_rpm), self._publish_throttle_command)

    def _publish_throttle_command(self, evt):
        desired_delta = self.desired_rpm-self.last_rpm
        clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
        smoothed_rpm = self.last_rpm + clipped_delta
        self.last_rpm = smoothed_rpm         
        # print self.desired_rpm, smoothed_rpm
        self.rpm_output.publish(Float64(smoothed_rpm))
            
    def _process_throttle_command(self,msg):
        input_rpm = msg.data
        # Do some sanity clipping
        input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)
        self.desired_rpm = input_rpm

    def _publish_servo_command(self, evt):
        desired_delta = self.desired_servo_position-self.last_servo
        clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = self.last_servo + clipped_delta
        self.last_servo = smoothed_servo         
        self.servo_output.publish(Float64(smoothed_servo))

    def _process_servo_command(self,msg):
        input_servo = msg.data
        # Do some sanity clipping
        input_servo = min(max(input_servo, self.min_servo), self.max_servo)
        # set the target servo position
        self.desired_servo_position = input_servo




def main(args=None):
    try:
        rclpy.init(args=args)
        ti = InterpolateThrottle()
        rclpy.spin(ti)
        ti.destroy_node()
        rclpy.shutdown()

    except:
        pass

# Boilerplate node spin up. 
if __name__ == '__main__':
    main()

