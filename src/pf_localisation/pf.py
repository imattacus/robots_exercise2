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
        
       
    def initialise_particle_cloud(self, initialpose):
        # Set particle cloud to initialpose plus noise

 
    
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
            pose.rotation = rotateQuaternion(q_orig=pose.rotation,
                                             yaw=random.gauss(mu=0, sigma=rot_var))


    def estimate_pose(self):

	average = Pose();	
	average.position.x = 0
	average.position.y = 0
	average.orientation.x = 0
	average.orientation.y = 0
	average.orientation.z = 0
	average.orientation.w = 0

	for pose in self.particlecloud.poses:
		average.position.x += pose.position.x
		average.position.y += pose.position.y
		average.orientation.x += pose.orientation.x
		average.orientation.y += pose.orientation.y
		average.orientation.z += pose.orientation.z
		average.orientation.w += pose.orientation.w
	
	length = len(self.particlecloud.poses)
	average.position.x = average.position.x / length
	average.position.y = average.position.y / length
	average.orientation.x = average.orientation.x / length
	average.orientation.y = average.orientation.y / length
	average.orientation.z = average.orientation.z / length
	average.orientation.w = average.orientation.w / length

	return average

        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers
