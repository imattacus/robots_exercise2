import math
from random import gauss

from geometry_msgs.msg import Pose, PoseArray

from pf_base import PFLocaliserBase
from util import rotateQuaternion

PARTICLE_COUNT = 100;
NEW_PARTICLE_NOISE_MULTIPLIER = 2;


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

    def initialise_particle_cloud(self, initialpose):
        poses = PoseArray()
        for i in range(0, 100):
            dx = gauss(0, self.ODOM_TRANSLATION_NOISE) * NEW_PARTICLE_NOISE_MULTIPLIER
            dy = gauss(0, self.ODOM_DRIFT_NOISE) * NEW_PARTICLE_NOISE_MULTIPLIER
            drot = gauss(0, self.ODOM_ROTATION_NOISE) * NEW_PARTICLE_NOISE_MULTIPLIER
            extra_rot = math.atan(dy / dx)  # Caused by a combination of drift and translation
            total_rot = drot + extra_rot  # Caused by intentional noisy rotation
            dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)) # The distance supposedly travelled
            new_x = dist * math.cos(total_rot) # The grid oriented change in x and y
            new_y = dist * math.sin(total_rot)
            new_pose = Pose()
            new_pose.position.x = initialpose.position.x + new_x
            new_pose.position.y = initialpose.position.y + new_y
            new_pose.orientation = rotateQuaternion(initialpose.orientation, total_rot)
            poses.poses.append(new_pose)
        return poses

    def update_particle_cloud(self, scan):

    # Update particlecloud, given map and laser scan

    def estimate_pose(self):
# Create new estimated pose, given particle cloud
# E.g. just average the location and orientation values of each of
# the particles and return this.

# Better approximations could be made by doing some simple clustering,
# e.g. taking the average location of half the particles after
# throwing away any which are outliers
