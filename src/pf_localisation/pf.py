from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random

from time import time
import numpy as np


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        # TODO: do some experiments to work out what these should actually be
        self.ODOM_ROTATION_NOISE = 0
        self.ODOM_TRANSLATION_NOISE = 0
        self.ODOM_DRIFT_NOISE = 0
 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict
        
        self.PARTICLE_COUNT = 20 # Number of particles
       
    def initialise_particle_cloud(self, initialpose):
        # Set particle cloud to initialpose plus noise
        new_poses = PoseArray()
        new_poses.header.frame_id = 0 # ?
        new_poses.header.stamp = rospy.Time.now()


        # TODO: what should these actually be?
        x_var = 0.1
        y_var = 0.1
        rot_var = 0.1

        for i in range(self.PARTICLE_COUNT):
            new_pose = Pose()
            new_pose.position.x = random.gauss(mu=initialpose.position.x, sigma=x_var)
            new_pose.position.y = random.gauss(mu=initialpose.position.y, sigma=y_var)
            new_pose.orientation = rotateQuaternion(q_orig=initialpose.orientation,
                                                    yaw=random.gauss(mu=0, sigma=rot_var))
            new_poses.poses.append(new_pose)

	return new_poses
 
    
    def update_particle_cloud(self, scan):
        # Update particlecloud, given map and laser scan
        weights = np.fromiter((self.sensor_model.get_weight(scan, pose)
                               for pose in self.particlecloud.poses), float)

        self.particlecloud.poses = np.random.choice(self.particlecloud.poses,
                                                    size=len(self.particlecloud.poses),
                                                    p=weights / weights.sum()).tolist()

        # TODO: what should these actually be?
        x_var = 0.1
        y_var = 0.1
        rot_var = 0.1

        for pose in self.particlecloud.poses:
            pose.position.x = random.gauss(mu=pose.position.x, sigma=x_var)
            pose.position.y = random.gauss(mu=pose.position.y, sigma=y_var)
            pose.orientation = rotateQuaternion(q_orig=pose.orientation,
                                                yaw=random.gauss(mu=0, sigma=rot_var))


    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers

