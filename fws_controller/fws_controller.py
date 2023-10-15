#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math


class Kinematics(Node):

    def __init__(self):
        super().__init__('kinematics')

        # Initialize Twist Subscriber
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10)

        # Init Action Client
        self.steering_action = ActionClient(
            self,
            FollowJointTrajectory,
            '/steering_position_controller/follow_joint_trajectory')
        self.wheel_action = ActionClient(
            self,
            FollowJointTrajectory,
            '/wheel_velocity_controller/follow_joint_trajectory')

        # Wait For Server
        self.steering_action.wait_for_server()
        self.wheel_action.wait_for_server()

    def twist_callback(self, msg):

        # ================= Calculations ================= #

        # Parameters
        R = 0.065
        L = 0.2

        # Init Output
        delta = 0.0
        v_wheel = 0.0

        # Twist Messages
        x_dot = msg.linear.x
        omega_z = msg.angular.z

        # Twist to Model
        V = x_dot
        Omega = omega_z

        # Vehicle Model
        if V != 0:
            delta = math.atan((Omega * L) / (2 * V))
            v_wheel = V / math.cos(delta)

        # m/s to rad/s
        omega_wheel = v_wheel / R

        # ================= Joint Actions ================= #

        # Steering Goal
        steering_msg = FollowJointTrajectory.Goal()
        steering_msg.trajectory.joint_names = [
            'front_left_steering_joint', 'front_right_steering_joint', 'rear_left_steering_joint', 'rear_right_steering_joint']
        steering_position = JointTrajectoryPoint()
        steering_position.positions = [delta, delta, -delta, -delta]
        steering_msg.trajectory.points.append(steering_position)

        # Wheel Goal
        wheel_msg = FollowJointTrajectory.Goal()
        wheel_msg.trajectory.joint_names = [
            'front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
        wheel_velocity = JointTrajectoryPoint()
        wheel_velocity.positions = [0.0, 0.0, 0.0, 0.0]
        wheel_velocity.velocities = [omega_wheel, omega_wheel, omega_wheel, omega_wheel]
        wheel_msg.trajectory.points.append(wheel_velocity)

        # Action
        try:
            self.steering_action.send_goal_async(steering_msg)
            self.get_logger().info('Steering goal sent.')
            self.wheel_action.send_goal_async(wheel_msg)
            self.get_logger().info('Wheel goal sent.')

        except Exception as e:
            self.get_logger().error(f'Error sending goals: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    node_kinematics = Kinematics()

    rclpy.spin(node_kinematics)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
