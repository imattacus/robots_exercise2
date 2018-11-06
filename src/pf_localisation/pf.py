from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random, gauss
import random as rand

from time import time
import numpy as np
from copy import deepcopy


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
        self.NUMBER_PREDICTED_READINGS = 50  # Number of readings to predict

        self.PARTICLE_COUNT = 200  # Total number of particles
        self.INITIAL_PARTICLE_COUNT = 50 # Number of particles around the initial pose (<= PARTICLECOUNT)
	self.RESAMPLE_PARTICLE_COUNT = 150 # Number of particles to generate on resampling (<= PARTICLECOUNT)

    def random_pose(self):
	new_pose = Pose()
        rand_angle = rand.uniform(0, 2 * math.pi)
        new_pose.orientation = rotateQuaternion(q_orig=Quaternion(0, 0, 0, 1),
		                                yaw=rand_angle)

	while True:
            x, y = self.map_coords_to_world(map_x=rand.uniform(0, self.occupancy_map.info.width - 1),
                                            map_y=rand.uniform(0, self.occupancy_map.info.height - 1))
            new_pose.position.x = x
            new_pose.position.y = y

	    if not self.map_cell_occupied(new_pose):
	        #rospy.loginfo("random x={}, y={}".format(new_pose.position.x, new_pose.position.y))
	        break

	return new_pose

    def initialise_particle_cloud(self, initialpose):
        rospy.loginfo("initialise_particle_cloud")
        # Set particle cloud to initialpose plus noise
        new_poses = PoseArray()
        new_poses.header.frame_id = 0  # ?
        new_poses.header.stamp = rospy.Time.now()
        x_var = initialpose.pose.covariance[6 * 0 + 0]
        y_var = initialpose.pose.covariance[6 * 1 + 1]
        rot_var = initialpose.pose.covariance[6 * 5 + 5]

        for i in range(self.INITIAL_PARTICLE_COUNT):
            new_pose = Pose()
            new_pose.position.x = gauss(mu=initialpose.pose.pose.position.x, sigma=x_var)
            new_pose.position.y = gauss(mu=initialpose.pose.pose.position.y, sigma=y_var)
            new_pose.orientation = rotateQuaternion(q_orig=initialpose.pose.pose.orientation,
                                                    yaw=gauss(mu=0, sigma=rot_var))

            new_poses.poses.append(new_pose)

        new_poses = self.topup_poses_with_random(new_poses)

        return new_poses

    def resample(self, particles, weights, count):
        return [deepcopy(np.random.choice(particles, p=weights / weights.sum())) for i in range(count)]

    def pose_to_map_coords(self, pose):
        ox = pose.position.x
        oy = pose.position.y

        map_x = math.floor((ox - self.sensor_model.map_origin_x) / self.sensor_model.map_resolution + 0.5) + self.sensor_model.map_width / 2
        map_y = math.floor((oy - self.sensor_model.map_origin_y) / self.sensor_model.map_resolution + 0.5) + self.sensor_model.map_height / 2

        return int(math.floor(map_x)), int(math.floor(map_y))

    def map_coords_to_world(self, map_x, map_y):
        x = self.sensor_model.map_resolution * (map_x - self.sensor_model.map_width / 2) + self.sensor_model.map_origin_x
        y = self.sensor_model.map_resolution * (map_y - self.sensor_model.map_height / 2) + self.sensor_model.map_origin_y

        return x, y

    def map_cell_occupied(self, pose):
        x, y = self.pose_to_map_coords(pose)
        threshold = 80
        prob_occupied = self.occupancy_map.data[x + y * self.occupancy_map.info.width]

        return prob_occupied == -1 or prob_occupied > threshold

    def topup_poses_with_random(self, poses):
	while len(poses.poses) < self.PARTICLE_COUNT:
	    poses.poses.append(self.random_pose())
	return poses

    def update_particle_cloud(self, scan):
        scan.ranges = np.fromiter((0.0 if np.isnan(r) else r for r in scan.ranges), float)

        # Update particlecloud, given map and laser scan
        self.particlecloud.poses = [particle for particle in self.particlecloud.poses
                                    if not self.map_cell_occupied(particle)]
        weights = np.fromiter((self.sensor_model.get_weight(scan, pose)
                               for pose in self.particlecloud.poses), float)

        new_poses = self.resample(self.particlecloud.poses, weights, self.RESAMPLE_PARTICLE_COUNT)

	self.particlecloud.poses = new_poses
	self.particlecloud = self.topup_poses_with_random(self.particlecloud)

        # TODO: what should these actually be?
        x_var = 0.05
        y_var = 0.05
        rot_var = 0.05

        for pose in self.particlecloud.poses:
            pose.position.x = gauss(mu=pose.position.x, sigma=x_var)
            pose.position.y = gauss(mu=pose.position.y, sigma=y_var)
            pose.orientation = rotateQuaternion(q_orig=pose.orientation,
                                                yaw=gauss(mu=0, sigma=rot_var))


    def kMeans(X, K, maxIters = 10, plot_progress = None):
        centroids = X[np.random.choice(np.arange(len(X)), K), :]
        for i in range(maxIters):
            # Cluster Assignment step
            C = np.array([np.argmin([np.dot(x_i-y_k, x_i-y_k) for y_k in centroids]) for x_i in X])
            # Move centroids step
            centroids = [X[C == k].mean(axis = 0) for k in range(K)]
            if plot_progress != None: plot_progress(X, C, np.array(centroids))
        return np.array(centroids) , C
    
    def estimate_pose(self):
        rospy.loginfo("estimate_pose")

        positions = []

        for pose in self.particlecloud.poses:
            positions.append([pose.position.x, pose.position.y])

        #whitened = whiten(means)
        centroids, labels = kMeans(positions, 5)

        centroidGroups = []	
        for i in len(centroids):
            group = []
            for n in labels:
                if i == n:
                    group.append(self.particlecloud.poses[n])
            centroidGroups.append(group)

        mostMembersCluster = 0
        maxMembers = len(centroidGroups[0])

        for i in centroidGroups:
            noMembers = len(centroidGroups[i])
            if maxMembers < noMembers:
                maxMembers = noMembers
                mostMembersCluster = i

        meanPose = Pose()
        meanPose.position.x = centroids[mostMembersCluster][0]
        meanPose.position.y = centroids[mostMembersCluster][1]
        meanPose.orientation.x = 0.0
        meanPose.orientation.y = 0.0
        meanPose.orientation.z = 0.0
        meanPose.orientation.w = 0.0

        for pose in centroidGroups[mostMembersCluster]:
            meanPose.orientation.x += pose.orientation.x
            meanPose.orientation.y += pose.orientation.y
            meanPose.orientation.z += pose.orientation.z
            meanPose.orientation.w += pose.orientation.w

        meanPose.orientation.x /= maxMembers
        meanPose.orientation.y /= maxMembers
        meanPose.orientation.z /= maxMembers
        meanPose.orientation.w /= maxMembers

        return meanPose
