from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random, gauss

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
	rospy.loginfo("initialise_particle_cloud")
        # Set particle cloud to initialpose plus noise
        new_poses = PoseArray()
        new_poses.header.frame_id = 0 # ?
        new_poses.header.stamp = rospy.Time.now()

        x_var = initialpose.pose.covariance[6 * 0 + 0]
        y_var = initialpose.pose.covariance[6 * 1 + 1]
        rot_var = initialpose.pose.covariance[6 * 5 + 5]

        for i in range(self.PARTICLE_COUNT):
            new_pose = Pose()
            new_pose.position.x = gauss(mu=initialpose.pose.pose.position.x, sigma=x_var)
            new_pose.position.y = gauss(mu=initialpose.pose.pose.position.y, sigma=y_var)
            new_pose.orientation = rotateQuaternion(q_orig=initialpose.pose.pose.orientation,
                                                    yaw=gauss(mu=0, sigma=rot_var))
            new_poses.poses.append(new_pose)

	return new_poses
 
    
    def update_particle_cloud(self, scan):
	rospy.loginfo("update_particle_cloud")
        # Update particlecloud, given map and laser scan
        weights = np.fromiter((self.sensor_model.get_weight(scan, pose)
                               for pose in self.particlecloud.poses), float)

        self.particlecloud.poses = np.random.choice(self.particlecloud.poses,
                                                    size=len(self.particlecloud.poses),
                                                    p=weights / weights.sum()).tolist()

        # TODO: what should these actually be?
        x_var = 0
        y_var = 0
        rot_var = 0

        for pose in self.particlecloud.poses:
            pose.position.x = gauss(mu=pose.position.x, sigma=x_var)
            pose.position.y = gauss(mu=pose.position.y, sigma=y_var)
            pose.orientation = rotateQuaternion(q_orig=pose.orientation,
                                                yaw=gauss(mu=0, sigma=rot_var))


    def estimate_pose(self):
	    rospy.loginfo("estimate_pose")
	
	
        numBinsHorizontal = 10
	    numBinsVertical = 10

        binWidth = self.occupancy_map.info.width / numBinsHorizontal
        binHeight = self.occupancy_map.info.height / numBinsVertical

	    bins = np.zeros(numBinsVertical, numBinsHorizontal)


	    for pose in self.particlecloud.poses:
            particle_x = 
            particle_y = 
		    bin_x = int(math.floor(particle_x / binWidth))
            bin_y = int(math.floor(particle_y / binHeight))
            bins[bin_y][bin_x] += 1

        busiestBinX = 0
        busiestBinY = 0 
        busyBin = 0
        for i in range(numBinsVertical):
            for j in range(numBinsHorizontal):
                if busyBin < bins[i][j]:
                    busiestBinX = j
                    busiestBinY = i
                    busyBin = bins[i][j]

        if busiestBinX > 0:
            searchAreaX = (busiestBinX-1) * binWidth
            searchAreaWidth = binWidth * 3
        else:
            searchAreaX = 0
            searchAreaWidth = binWidth * 2

        if busiestBinY > 0:
            searchAreaY = (busiestBinY-1) * binHeight
            searchAreaHeight = binHeight * 3
        else:
            searchAreaY = 0
            searchAreaHeight = binHeight * 2

        
        average = Pose()
        average.position.x = 0
        average.position.y = 0
        average.orientation.x = 0
        average.orientation.y = 0
        average.orientation.z = 0
        average.orientation.w = 0
        count = 0
             
        for pose in self.particlecloud.poses:
            particle_x = 
            particle_y = 
            if (searchAreaX < particle_x and particle_x < searchAreaX+searchAreaWidth):
                if (searchAreaY < particle_y and particle_y < searchAreaY+searchAreaHeight):
                    count += 1
                    average.position.x += pose.position.x
		            average.position.y += pose.position.y
		            average.orientation.x += pose.orientation.x
		            average.orientation.y += pose.orientation.y
		            average.orientation.z += pose.orientation.z
                    average.orientation.w += pose.orientation.w

        average.position.x = average.position.x / count
	    average.position.y = average.position.y / count
	    average.orientation.x = average.orientation.x / count
	    average.orientation.y = average.orientation.y / count
	    average.orientation.z = average.orientation.z / count
        average.orientation.w = average.orientation.w / count

        return average

        
                
                
	


        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers

