#!/usr/bin/env python3

import numpy as np
from scipy.spatial.distance import cdist
from sdf import *
import functions.pawn as ex
from sklearn.neighbors import BallTree


class PID:

    def __init__(self, P, I, D, error_lim) -> None:
        self.P = P
        self.I = I
        self.D = D
        self.error_lim = error_lim
        self.prev_error = 0
        self.error_sum = 0

    
    def get_control(self, current_pose_axis, goal_pose_axis, dt):

        error = goal_pose_axis - current_pose_axis
        derror = (error - self.prev_error) / dt
        self.error_sum += error * dt

        if abs(self.error_sum) > self.error_lim:
            self.error_sum = np.sign(self.error_sum) * self.error_lim

        self.prev_error = error

        return self.P * error + self.I * self.error_sum + self.D * derror




class APFSwarmController():

    def __init__(self, p_cohesion=1, p_seperation=1, p_alignment=1, max_vel=0.5, min_dist=0.2) -> None:
        
        self.swarm = None
        self.goals = None
        self.min_dist = min_dist
        self.velocities = None
        self.p_separation = p_seperation
        self.p_alignment = p_alignment
        self.p_cohesion = p_cohesion
        self.max_vel = max_vel
    
    def distribute_goals(self, start, goals):
        # print(start)
        # print(goals)
        dist_arr = cdist(start, goals)
        out_goals = np.zeros_like(goals)
        
        for i in range(start.shape[0]):
            ind = np.argmin(dist_arr[i][dist_arr[i]>0])
            out_goals[i] = goals[ind]
            dist_arr[i, :] = np.inf
            dist_arr[:, ind] = np.inf
        self.goals = out_goals


    def get_control(self, poses) -> None:
        if self.velocities is None:
            self.velocities = np.zeros_like(poses)
        ball_tree = BallTree(poses[:, :2], metric='euclidean')

        vel_cohesion = self.p_cohesion*(self.goals - poses)
        for j in range(len(vel_cohesion)):
            vel= np.linalg.norm(vel_cohesion[j])
            if vel >= self.max_vel:
                vel_cohesion[j] = self.max_vel*vel_cohesion[j]/vel

                
        vel_separation = np.zeros_like(vel_cohesion)
        for i, pose in enumerate(poses):
            query_pose = pose[:2]
            nearest_ind = ball_tree.query_radius(query_pose.reshape(1, -1), self.min_dist)[0][1:]
            v = np.zeros(3)
            for ind in nearest_ind:
                v -= (poses[ind] - pose)
                v[2] = 0
            vel_separation[i] = v
            
        control_vels = vel_cohesion + vel_separation

        
        # print(control_vels)
        return control_vels
