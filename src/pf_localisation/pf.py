from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random, gauss
import random as rand

from time import time
import numpy as np
import scipy.cluster.vq
import vq, kmeans, whiten
from copy import deepcopy


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # Set motion model parameters
        # TODO: do some experiments to work out what these should actually be
        self.ODOM_ROTATION_NOISE = -10.0 * math.pi / 180
        self.ODOM_TRANSLATION_NOISE = -0.2
        self.ODOM_DRIFT_NOISE = 0

        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 50  # Number of readings to predict

        self.PARTICLE_COUNT = 200  # Number of particles
        self.RANDOM_PARTICLE_COUNT = 150

    def initialise_particle_cloud(self, initialpose):
        rospy.loginfo("initialise_particle_cloud")
        # Set particle cloud to initialpose plus noise
        new_poses = PoseArray()
        new_poses.header.frame_id = 0  # ?
        new_poses.header.stamp = rospy.Time.now()

        x_var = initialpose.pose.covariance[6 * 0 + 0]
        y_var = initialpose.pose.covariance[6 * 1 + 1]
        rot_var = initialpose.pose.covariance[6 * 5 + 5]

        for i in range(self.PARTICLE_COUNT - self.RANDOM_PARTICLE_COUNT):
            new_pose = Pose()
            new_pose.position.x = gauss(mu=initialpose.pose.pose.position.x, sigma=x_var)
            new_pose.position.y = gauss(mu=initialpose.pose.pose.position.y, sigma=y_var)
            new_pose.orientation = rotateQuaternion(q_orig=initialpose.pose.pose.orientation,
                                                    yaw=gauss(mu=0, sigma=rot_var))
            new_poses.poses.append(new_pose)

        for i in range(self.RANDOM_PARTICLE_COUNT):
            while True:
                new_pose = Pose()
                new_pose.position.x = rand.uniform(0, self.occupancy_map.info.width)
                new_pose.position.y = rand.uniform(0, self.occupancy_map.info.width)
                if not self.map_cell_occupied(new_pose):
                    break
            new_pose.orientation = rotateQuaternion(q_orig=initialpose.pose.pose.orientation,
                                                    yaw=rand.uniform(0, 2 * math.pi))
            new_poses.poses.append(new_pose)

        return new_poses

    def resample(self, particles, weights):
        S = []
        c = [weights[0]]
        for i in range(1, self.PARTICLE_COUNT):
            c.append(c[i - 1] + weights[i])

        u = [rand.uniform(0, 1 / self.PARTICLE_COUNT)]
        i = 0
        for j in range(0, self.PARTICLE_COUNT):
            while (u[j] > c[i]):
                i += 1
            S.append(deepcopy(particles[i]))
            u.append(u[j] + 1 / self.PARTICLE_COUNT)
        return S

    def pose_to_map_coords(self, pose):
        map_x = pose.position.x + self.occupancy_map.info.resolution * self.occupancy_map.info.width / 2
        map_y = pose.position.y + self.occupancy_map.info.resolution * self.occupancy_map.info.height / 2
        return int(math.floor(map_x)), int(math.floor(map_y))

    def map_cell_occupied(self, pose):
        x, y = self.pose_to_map_coords(pose)
        threshold = 50
        return self.occupancy_map.data[x + y * self.occupancy_map.info.width] > threshold

    def update_particle_cloud(self, scan):

        scan.ranges = np.fromiter((0.0 if np.isnan(r) else r for r in scan.ranges), float)

        # Update particlecloud, given map and laser scan
        weights = np.fromiter((self.sensor_model.get_weight(scan, pose)
                               for pose in self.particlecloud.poses), float)

        self.particlecloud.poses = self.resample(self.particlecloud.poses, weights)
        # TODO: what should these actually be?
        x_var = 0.05
        y_var = 0.05
        rot_var = 0.05

        for pose in self.particlecloud.poses:
            pose.position.x = gauss(mu=pose.position.x, sigma=x_var)
            pose.position.y = gauss(mu=pose.position.y, sigma=y_var)
            pose.orientation = rotateQuaternion(q_orig=pose.orientation,
                                                yaw=gauss(mu=0, sigma=rot_var))

    def estimate_pose(self):
        rospy.loginfo("estimate_pose")

        positions = np.array[]

        for pose in self.particlecloud.poses:
            positions.append([pose.position.x, pose.position.y])

        # whitened = whiten(means)
        centroids, labels = kmeans2(positions, 5)

        centroidGroups = np.array[]
        for i in len(centroids):
            group = np.array[]
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
