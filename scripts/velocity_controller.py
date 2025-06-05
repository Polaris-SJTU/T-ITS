#!/usr/bin/env python3
import rospy
import socket
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose
from tools import get_rotation_matrix, PIDController


class VelocityController:
    def __init__(self):
        rospy.init_node("velocity_controller", anonymous=True)

        self.number_of_robots = rospy.get_param("number_of_robots")
        self.ip_addresses = {robot_id: f"192.168.1.{200 + robot_id}" for robot_id in range(self.number_of_robots)}
        self.port = rospy.get_param("port")

        a = 106 / 1000
        b = 104 / 1000
        r = 37.5 / 1000
        self.matrix = np.array([
            [1, -1, -(a + b)],
            [1, 1, a + b],
            [1, 1, -(a + b)],
            [1, -1, a + b],
        ]) / r * 10

        self.x = {}
        self.u = {}
        self.controller = {}

        for robot_id in range(self.number_of_robots):
            self.x[robot_id] = None
            self.u[robot_id] = None
            self.controller[robot_id] = PIDController(1, 0.1, 0.1)
            rospy.Subscriber(
                f"/mecanum{robot_id}/pose", Pose,
                self.pose_callback(robot_id), queue_size=1
            )
            rospy.Subscriber(
                f"/mecanum{robot_id}/twist", Twist,
                self.twist_callback(robot_id), queue_size=1
            )
            rospy.Subscriber(
                f"/mecanum{robot_id}/twist_setpoint", Twist,
                self.twist_setpoint_callback(robot_id), queue_size=1
            )
        rospy.spin()

    def pose_callback(self, robot_id):
        def callback(message):
            position = message.position
            orientation = message.orientation
            theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
            self.x[robot_id] = np.array([position.x, position.y, theta]).reshape((3, 1))

        return callback

    def twist_callback(self, robot_id):
        def callback(message):
            if self.x[robot_id] is None:
                return
            linear = message.linear
            angular = message.angular
            v = np.array([linear.x, linear.y, angular.z]).reshape((3, 1))
            self.u[robot_id] = get_rotation_matrix(-self.x[robot_id][2, 0]) @ v

        return callback

    def twist_setpoint_callback(self, robot_id):
        def callback(message):
            if self.u[robot_id] is None:
                return
            linear = message.linear
            angular = message.angular
            u = np.array([linear.x, linear.y, angular.z]).reshape((3, 1))

            omega_setpoint = self.matrix @ u
            error = self.matrix @ (u - self.u[robot_id])

            duty_cycles = omega_setpoint + self.controller[robot_id].output(error)
            command = "<%d,%d,%d,%d>" % (duty_cycles[0, 0], duty_cycles[1, 0], duty_cycles[2, 0], duty_cycles[3, 0])

            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto(command.encode(), (self.ip_addresses[robot_id], self.port))

        return callback

    def __del__(self):
        for robot_id in range(self.number_of_robots):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto("<0,0,0,0>".encode(), (self.ip_addresses[robot_id], self.port))


if __name__ == "__main__":
    try:
        VelocityController()
    except rospy.ROSInterruptException:
        pass
