#!/usr/bin/env python3

from flock_gpt.msg import Vector3StampedArray
import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import MarkerArray, Marker
RATE = 100
class SwarmSimulationNode():

    def __init__(self, drone_grid_shape, drone_grid_step, swarm_pose=(0, 0, 0)) -> None:
        nx, ny = drone_grid_shape
        x = np.linspace(swarm_pose[0], drone_grid_step[0]*(nx-1) + swarm_pose[0], nx)
        y = np.linspace(swarm_pose[1], drone_grid_step[1]*(ny-1) + swarm_pose[1], ny)
        xv, yv = np.meshgrid(x, y)
        xv = xv.flatten()
        yv = yv.flatten()
        zv = swarm_pose[2] * np.ones(yv.shape)
        self.swarm = np.flip(np.hstack((xv.reshape((xv.shape[0], 1)), yv.reshape((yv.shape[0], 1)), zv.reshape((zv.shape[0], 1)))))

        self.pose_piblisher = rospy.Publisher('/swarm/poses', Vector3StampedArray, queue_size=1)
        self.viz_piblisher = rospy.Publisher('/swarm/viz', MarkerArray, queue_size=1)

        self.prev_time = rospy.get_time()
        rospy.Subscriber("/swarm/cmd_vel", Vector3StampedArray, self.callback_cmd, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1/RATE), self.callback_localization)

    def callback_cmd(self, msg:Vector3StampedArray):
        vels = []
        for cmd in msg.vector:
            vels.append(np.array([cmd.x, cmd.y, cmd.z]))
        vels = np.array(vels)
        self.swarm += vels * (0.01)

        self.prev_time = rospy.get_time()

    def callback_localization(self, event):
        vector = Vector3StampedArray()
        vector.header.stamp = rospy.Time.now()
        vector.header.frame_id = "map"
        viz = MarkerArray()
        i = 0
        for pose in self.swarm:
            point = Vector3()
            point.x = pose[0]
            point.y = pose[1]
            point.z = pose[2]

            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.type = 2
            marker.action = 0
            marker.id = i
            i+=1
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = pose[2]
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.9
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1
            marker.lifetime = rospy.Duration(1/RATE)

            vector.vector.append(point)
            viz.markers.append(marker)
        self.pose_piblisher.publish(vector)
        self.viz_piblisher.publish(viz)


def shutdown():
    pass

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdown)
        rospy.init_node('swarm_controller_node', anonymous=True)
        controller = SwarmSimulationNode((10,10), (0.5, 0.5), swarm_pose=(1,1,1))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass