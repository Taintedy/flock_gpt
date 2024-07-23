#!/usr/bin/env python3


import numpy as np
import rospy
from sdf import box, sphere
import functions.pawn as ex
from pycrazyswarm import Crazyswarm
from pynput.keyboard import Key, Listener
import sys

from apf_controller import APFSwarmController
from point_distributor import PointDistributer

RATE = 10
import os
path2launch = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../crazyswarm/launch'))
class SwarmControllerNode():

    def __init__(self, cost_func, max_vel, min_dist) -> None:
        self.max_vel = max_vel
        self.controller = APFSwarmController(max_vel=max_vel, min_dist=min_dist)
        self.swarm = Crazyswarm(crazyflies_yaml=path2launch + '/crazyflies.yaml')
        self.all_cfs = self.swarm.allcfs.crazyflies
        self.timeHelper = self.swarm.timeHelper
        self.start_poses = np.array([[-2, 1.5, 1.0],
                                [-2, 0.5, 1.0],
                                [-2, -0.5, 1.0],
                                [-2, -1.5, 1.0],
                                [2, 1.5, 1.0],
                                [2, 0.5, 1.0],
                                [2, -0.5, 1.0],
                                [2, -1.5, 1.0]])
        self.mode = 'idle'
        self.current_cost = 0
        self.cost_func = cost_func

        self.pd = PointDistributer(self.cost_func[self.current_cost])
        print("generating points...")
        print(len(self.all_cfs))
        goals = self.pd.generate_points(len(self.all_cfs))
        print("points generated")
        self.distribute_goals(goals)

        


        self.timer = rospy.Timer(rospy.Duration(1/RATE), self.callback_control)
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def callback_control(self, event):
        if self.mode == 'takeoff':
            self.takeoff()
        if self.mode == 'show':
            self.show()
        if self.mode == 'land':
            self.distribute_goals(self.start_poses)
            self.land()
        if self.mode == 'kill':
            self.kill_switch()

    def kill_switch(self):
        for cf in self.all_cfs:
            cf.cmdStop()
        self.mode = 'idle'

    def distribute_goals(self, new_goals):
        poses = []
        for cf in self.all_cfs:
            poses.append(cf.position())
        poses = np.array(poses)
        self.goals = self.controller.distribute_goals(poses, new_goals[:poses.shape[0]])
    
    def takeoff(self):
        for cf in self.all_cfs:
            cf.takeoff(targetHeight=1.0, duration=2.5)
        self.mode = 'idle'

    def next_cost(self):
        self.current_cost += 1
        if self.current_cost >= len(self.cost_func):
            self.current_cost = 0
        
        self.pd.cost = self.cost_func[self.current_cost]
        print("generating points...")
        goals = self.pd.generate_points(len(self.all_cfs))
        print("points generated")
        self.distribute_goals(goals)
        # self.controller.cost_func = self.cost_func[self.current_cost]

    def land(self):
        poses = []
        for cf in self.all_cfs:
            poses.append(cf.position())
        poses = np.array(poses)
        mean_dist = np.mean(np.linalg.norm(self.controller.goals - poses, axis=1)) 
        if mean_dist <= 0.5 and self.start_poses[0, 2] != 0 :
            print("landing")
            self.controller.goals[:, 2] = np.zeros_like(self.controller.goals[:, 2])
            self.start_poses = self.controller.goals
            self.distribute_goals(self.controller.goals)    
        if self.start_poses[0, 2] == 0 and np.mean(poses[:, 2]) <= 0.1:
            self.kill_switch()
            self.mode = 'idle'

        vels = self.controller.get_control(poses)
        for i, cf in enumerate(self.all_cfs):
            cf.cmdVelocityWorld(vels[i], yawRate=0)



    def on_release(self, key):
        pass


    def on_press(self, key):
        try:
            print('alphanumeric key {0} pressed'.format(
                key.char))
            

            if key.char == '1':
                self.mode = "takeoff"
            if key.char == '2':
                self.mode = "show"

            if key.char == '3':
                self.start_poses[:, 2] = np.ones_like(self.start_poses[:, 2])
                self.mode = 'land'
            if key.char == 'w':
                self.next_cost()
                # print(self.current_cost)
            if key.char == 'e':
                self.mode = 'kill'
                

        except AttributeError:
            print('special key {0} pressed'.format(
                key))
            if key == Key.f10:
                sys.exit()


    def show(self):
        poses = []
        for cf in self.all_cfs:
            poses.append(cf.position())
        poses = np.array(poses)

        # vels =  self.controller.get_control(poses, self.max_vel)
        vels = self.controller.get_control(poses)
        # vels = np.array([[-0.9, 0, 0]])
        for i, cf in enumerate(self.all_cfs):
            cf.cmdVelocityWorld(vels[i], yawRate=0)

        

def shutdown():
    pass

if __name__ == "__main__":
    cost = [sphere(1).translate((0, 0, 2)), box(1).translate((0, 0, 2))]
    try:
        rospy.on_shutdown(shutdown)
        controller = SwarmControllerNode(cost_func=cost, max_vel=0.25, min_dist=0.3)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass