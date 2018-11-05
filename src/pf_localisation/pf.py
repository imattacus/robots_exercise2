from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random, gauss
import random as rand

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
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

        self.PARTICLE_COUNT = 20  # Number of particles

    def initialise_particle_cloud(self, initialpose):
        rospy.loginfo("initialise_particle_cloud")
        # Set particle cloud to initialpose plus noise
        new_poses = PoseArray()
        new_poses.header.frame_id = 0  # ?
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

    def resample(self, particles, weights):
        S = []
        c = [weights[0]]
        for i in range(1, self.PARTICLE_COUNT - 1):
            c[i] = c[i - 1] + weights[i]

        u = []
        u[0] = rand.uniform(0, 1 / self.PARTICLE_COUNT)
        i = 1
        for j in range(0, self.PARTICLE_COUNT - 1):
            while (u[j] > c[i]):
                i += 1
            S.append(particles[i])
            u.append(u[j] + 1 / self.PARTICLE_COUNT)
        return S

    def update_particle_cloud(self, scan):
        rospy.loginfo("update_particle_cloud")
        # Update particlecloud, given map and laser scan
        weights = np.fromiter((self.sensor_model.get_weight(scan, pose)
                               for pose in self.particlecloud.poses), float)

        self.particlecloud.poses = resample(self.particlecloud.poses, weights)
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
