#!/usr/bin/env python3
import rospy
import numpy as np
from tools import remap_angle, get_rotation_matrix
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist


class FormationController:
    def __init__(self):
        rospy.init_node("formation_controller", anonymous=True)

        self.number_of_robots = rospy.get_param("number_of_robots")
        self.leader_id = eval(rospy.get_param("leader_id"))
        self.desired_pose = eval(rospy.get_param("desired_pose"))

        self.k = 0.8
        self.k_u = 1 / 0.06
        self.sigma_u = 0.001
        self.control_rate = 20

        self.t0 = rospy.get_time()
        self.t = {}
        self.pose = {}
        self.hat_u = {}
        self.velocity_publisher = {}

        for robot_id in range(self.number_of_robots):
            self.pose[robot_id] = None
            self.t[robot_id] = rospy.get_time() - self.t0
            self.hat_u[robot_id] = np.zeros((3, 1))

            self.velocity_publisher[robot_id] = rospy.Publisher(
                f"/mecanum{robot_id}/twist_setpoint", Twist, queue_size=1
            )
            rospy.Subscriber(
                f"/mecanum{robot_id}/pose", Pose,
                self.pose_callback(robot_id), queue_size=1
            )
        rospy.spin()

    def pose_callback(self, robot_id):
        def callback(message):
            position = message.position
            orientation = message.orientation
            x = position.x
            y = position.y
            theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
            self.pose[robot_id] = np.array([x, y, theta]).reshape((3, 1))

            t = rospy.get_time() - self.t0
            if t - self.t[robot_id] > 1 / self.control_rate:
                dt = t - self.t[robot_id]
                twist = Twist()
                if robot_id == 0:
                    if t <= 30:
                        vx = 2 / 15 * np.sin(np.pi * t / 30) * np.sin(np.pi * t / 30) * np.sin(
                            np.pi / 2 - np.pi * t / 60 + np.sin(np.pi * t / 15) / 4)
                        vy = 2 / 15 * np.sin(np.pi * t / 30) * np.sin(np.pi * t / 30) * np.cos(
                            np.pi / 2 - np.pi * t / 60 + np.sin(np.pi * t / 15) / 4)
                        vz = -np.pi / 30 * np.sin(np.pi * t / 30) * np.sin(np.pi * t / 30)
                    elif t <= 60:
                        vx = 2 / 15 * np.sin(np.pi * (t - 30) / 30) * np.sin(np.pi * (t - 30) / 30)
                        vy, vz = 0, 0
                    else:
                        vx, vy, vz = 0, 0, 0
                    twist.linear.x = vx
                    twist.linear.y = vy
                    twist.angular.z = vz
                else:
                    xj = self.pose[robot_id]
                    xi = self.pose[self.leader_id[robot_id]]
                    hat_uij = self.hat_u[robot_id]
                    xjdi = np.array(self.desired_pose[robot_id]).reshape((3, 1))

                    xji = get_rotation_matrix(-xi[2, 0]) @ (xj - xi)
                    xji[2, 0] = remap_angle(xji[2, 0])
                    delta_xji = xji - xjdi
                    delta_xji[2, 0] = remap_angle(delta_xji[2, 0])

                    hat_uij += (-self.sigma_u * hat_uij - 1 / self.k_u * delta_xji) * dt
                    hat_uij[2, 0] -= 1 / self.k_u * (
                            xji[0, 0] * delta_xji[1, 0] - xji[1, 0] * delta_xji[0, 0]
                    ) * dt

                    uj = get_rotation_matrix(-xji[2, 0]) @ (-self.k * delta_xji + hat_uij - hat_uij[2, 0] * np.array(
                        [xji[1, 0], -xji[0, 0], 0]
                    ).reshape((3, 1)))
                    self.hat_u[robot_id] = hat_uij

                    twist.linear.x = uj[0, 0]
                    twist.linear.y = uj[1, 0]
                    twist.angular.z = uj[2, 0]

                self.velocity_publisher[robot_id].publish(twist)
                self.t[robot_id] = t

        return callback


if __name__ == "__main__":
    try:
        FormationController()
    except rospy.ROSInterruptException:
        pass
