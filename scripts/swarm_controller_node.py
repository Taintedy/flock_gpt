#!/usr/bin/env python3

from flock_gpt.msg import Vector3StampedArray
from geometry_msgs.msg import Vector3
import numpy as np
import rospy
from sdf import box, sphere, write_binary_stl, rounded_box, capsule
from sdf.mesh import generate
import functions.pawn as ex
from apf_controller import APFSwarmController
from gpt_sdf import SDFDialog, SDFModel
import threading


from point_distributor import PointDistributer
RATE = 10

class SwarmControllerNode():

    def __init__(self, goals=[]) -> None:
        rospy.Subscriber("/swarm/poses", Vector3StampedArray, self.callback_state, queue_size=1)
        self.cmd_vel_publisher = rospy.Publisher('/swarm/cmd_vel', Vector3StampedArray, queue_size=1)
        
        
        self.controller = APFSwarmController(max_vel=1)
        self.model = SDFModel()
        self.dialog = SDFDialog()
        self.goals = goals
        self.start_poses =  None
        # self.timer = rospy.Timer(rospy.Duration(1/RATE), self.callback_control)
        threading.Thread(target=self.continuous_input_prompt, daemon=True).start()

    def callback_state(self, msg:Vector3StampedArray):
        if (np.array(self.goals).size == 0):
            pass
        else:
            poses = []
            for pose in msg.vector:
                poses.append(np.array([pose.x, pose.y, pose.z]))
            poses = np.array(poses)
            

            if self.start_poses is None:
                self.goals = self.controller.distribute_goals(poses, self.goals)
                self.start_poses = poses

            vels = self.controller.get_control(poses)
            cmd_vel = Vector3StampedArray()
            for vel in vels:
                vect = Vector3()
                vect.x = vel[0]
                vect.y = vel[1]
                vect.z = vel[2]
                cmd_vel.vector.append(vect)
            self.cmd_vel_publisher.publish(cmd_vel)

    def continuous_input_prompt(self):
        while True:

            user_input = input("What to build?\n")
            self.process_user_input(user_input)
            self.start_poses = None
            

    def process_user_input(self, user_input):
        sdf_code = self.dialog.get_next_sdf_code(user_input)
        if sdf_code:
            local_vars = {"f": None}
            try: 
                f=10
                exec(sdf_code, globals(), local_vars)  # Pass the global dictionary to exec
                f = local_vars.get("f")
                if f is not None:
                    print("f is", f)
                    pd = PointDistributer(f)
                    print("generation:" + str(pd))
                    points = pd.generate_points(64)
                    print("end generation")
                    self.goals = points
                    # self.goals = self.controller.distribute_goals(self.start_poses, self.goals)
                else:
                    print("f is None")
            except Exception as e:
                print("Error: ", e)
def shutdown():
    pass

if __name__ == "__main__": 
    try:
        rospy.on_shutdown(shutdown)
        rospy.init_node('swarm_controller_node', anonymous=True)
        controller = SwarmControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass