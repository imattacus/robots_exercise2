from __future__ import print_function
import sys

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



def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def timeit(method):

    def timed(*args, **kw):
        ts = time()
        result = method(*args, **kw)
        te = time()

        with open("timings.csv", 'a+') as f:
	    f.write("{}\n".format(te-ts))
        return result

    return timed


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
        self.INITIAL_PARTICLE_COUNT = 180 # Number of particles around the initial pose (<= PARTICLECOUNT)
        self.RESAMPLE_PARTICLE_COUNT = 185 # Number of particles to generate on resampling (<= PARTICLECOUNT)

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

    def conv2d(self, a, f):
        s = f.shape + tuple(np.subtract(a.shape, f.shape) + 1)
        strd = np.lib.stride_tricks.as_strided
        subM = strd(a, shape=s, strides=a.strides * 2)
        return np.einsum('ij,ijkl->kl', f, subM)

    def gkern(self, l=5, sig=1.):
        ax = np.arange(-l // 2 + 1., l // 2 + 1.)
        xx, yy = np.meshgrid(ax, ax)

        kernel = np.exp(-(xx ** 2 + yy ** 2) / (2. * sig ** 2))

        return kernel / np.sum(kernel)

    @timeit
    def estimate_pose(self):
        rospy.loginfo("estimate_pose")
        kern_size = 5
        k = self.gkern(kern_size, 1)

        numBinsHorizontal = 400
        numBinsVertical = 400

        binWidth = self.occupancy_map.info.width / numBinsHorizontal
        binHeight = self.occupancy_map.info.height / numBinsVertical

        particle_bins = np.zeros((numBinsVertical, numBinsHorizontal), dtype=list)
        bins = np.zeros((numBinsVertical, numBinsHorizontal))


        for pose in self.particlecloud.poses:
            particle_x, particle_y = self.pose_to_map_coords(pose)
            bin_x = int(math.floor(particle_x / binWidth))
            bin_y = int(math.floor(particle_y / binHeight))
            bins[bin_y][bin_x] += 1
            if particle_bins[bin_y][bin_x] == 0:
                particle_bins[bin_y][bin_x] = [pose]
            else:
                particle_bins[bin_y][bin_x].append(pose)

        convolved = self.conv2d(bins, k)

        busiestBinX = 0
        busiestBinY = 0
        busyBinCount = 0
        for i in range(numBinsVertical - kern_size):
            for j in range(numBinsHorizontal - kern_size):
                if particle_bins[i][j] != 0:
                    if busyBinCount < len(convolved[i + math.floor(kern_size / 2)][j + math.floor(kern_size / 2)]):
                        busiestBinX = j + math.floor(kern_size / 2)
                        busiestBinY = i + math.floor(kern_size / 2)
                        busyBinCount = len(convolved[i][j])

        # if busiestBinX > 0:
        #     searchAreaX = (busiestBinX-1) * binWidth
        #     searchAreaWidth = binWidth * 3
        # else:
        #     searchAreaX = 0
        #     searchAreaWidth = binWidth * 2

        # if busiestBinY > 0:
        #     searchAreaY = (busiestBinY-1) * binHeight
        #     searchAreaHeight = binHeight * 3
        # else:
        #     searchAreaY = 0
        #     searchAreaHeight = binHeight * 2


        average = Pose()
        average.position.x = 0
        average.position.y = 0
        average.orientation.x = 0
        average.orientation.y = 0
        average.orientation.z = 0
        average.orientation.w = 0
        count = 0

        for i in range(busiestBinX - 1, busiestBinX + 1):
            for j in range(busiestBinY - 1, busiestBinY + 1):
                if i >= 0 and i < numBinsHorizontal - kern_size and j >= 0 and j < numBinsVertical - kern_size:
                    if particle_bins[j + math.floor(kern_size / 2)][i + math.floor(kern_size / 2)] != 0:
                        for pose in particle_bins[j + math.floor(kern_size / 2)][i + math.floor(kern_size / 2)]:
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

