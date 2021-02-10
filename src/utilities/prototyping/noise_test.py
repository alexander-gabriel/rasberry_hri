#!/usr/bin/env python2

from math import sqrt, sin, cos, atan2
import tf
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class Position(object):
    def __init__(self, x=0,y=0,z=0,w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __str__(self):
        return "x: {}, y: {}, z: {}, w: {}".format(self.x, self.y, self.z, self.w)

    def copy(self):
        return Position(self.x, self.y, self.z, self.w)

class Pose(object):

    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation

    def __str__(self):
        return "Position: {}\nOrientation: {}".format(self.position, self.orientation)

    def copy(self):
        return Pose(self.position.copy(), self.orientation.copy())

# 10 cm radius
MOVEMENT_NOISE_ALPHA = 0.08 # --X -0.1 - 0.1
MOVEMENT_NOISE_BETA = 0.08 # 0.16 # --X -0.1 - 0.1
MOVEMENT_NOISE_GAMMA = 0.08 # 0.15 # -- Y 0.9 - 1.1
MOVEMENT_NOISE_DELTA = 0.04 #0.1 # -- Y 0.6 - 1.35

# 5 cm radius
MOVEMENT_NOISE_ALPHA = 0.06 # --X -0.1 - 0.1
MOVEMENT_NOISE_BETA = 0.06 # 0.16 # --X -0.1 - 0.1
MOVEMENT_NOISE_GAMMA = 0.06 # 0.15 # -- Y 0.9 - 1.1
MOVEMENT_NOISE_DELTA = 0.03 #0.1 # -- Y 0.6 - 1.35


# 2.5 cm radius
MOVEMENT_NOISE_ALPHA = 0.03 # --X -0.1 - 0.1
MOVEMENT_NOISE_BETA = 0.03 # 0.16 # --X -0.1 - 0.1
MOVEMENT_NOISE_GAMMA = 0.03 # 0.15 # -- Y 0.9 - 1.1
MOVEMENT_NOISE_DELTA = 0.015 #0.1 # -- Y 0.6 - 1.35

def add_position_noise(old_pose, pose):
    # print("Old Pose: {}".format(old_pose))
    # print("New Pose: {}".format(pose))
    new_pose = pose.copy()
    if old_pose is not None:
        dx = pose.position.x - old_pose.position.x
        dy = pose.position.y - old_pose.position.y
        translation = sqrt(dx*dx + dy*dy)
        # print("Translation: {}".format(translation))
        q = [old_pose.orientation.x,
             old_pose.orientation.y,
             old_pose.orientation.z,
             old_pose.orientation.w]
        (r, p, old_theta) = tf.transformations.euler_from_quaternion(q)
        # print("Old r: {}, p: {}, theta: {}".format(r, p, old_theta))
        q = [pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w]
        (r, p, theta) = tf.transformations.euler_from_quaternion(q)
        # print("New r: {}, p: {}, theta: {}".format(r, p, theta))

        pre_movement_rotation = atan2(dy, dx) - old_theta
        # print("pre_movement_rotation: {}".format(pre_movement_rotation))
        post_movement_rotation = theta - pre_movement_rotation - old_theta
        # print("post_movement_rotation: {}".format(post_movement_rotation))
        # calculate standard deviations
        sd_pre_movement_rotation = (
            MOVEMENT_NOISE_ALPHA * abs(pre_movement_rotation)
            + MOVEMENT_NOISE_BETA * translation)
        # print("sd_pre_movement_rotation: {}".format(sd_pre_movement_rotation))
        sd_post_movement_rotation = (
            MOVEMENT_NOISE_ALPHA * abs(post_movement_rotation)
            + MOVEMENT_NOISE_BETA * translation)
        # print("sd_post_movement_rotation: {}".format(sd_post_movement_rotation))
        sd_translation = (
            MOVEMENT_NOISE_GAMMA * translation
            + MOVEMENT_NOISE_DELTA * (abs(pre_movement_rotation)
                                      + abs(post_movement_rotation)))
        # print("sd_translation: {}".format(sd_translation))

        translation += np.random.normal(
            0, sd_translation * sd_translation)
        # print("Translation: {}".format(translation))
        pre_movement_rotation += np.random.normal(
            0, sd_pre_movement_rotation * sd_pre_movement_rotation)
        # print("pre_movement_rotation: {}".format(pre_movement_rotation))
        post_movement_rotation += np.random.normal(
            0, sd_post_movement_rotation * sd_post_movement_rotation)
        # print("post_movement_rotation: {}".format(post_movement_rotation))

        new_pose.position.x = old_pose.position.x + \
            translation * cos(old_theta + pre_movement_rotation)
        new_pose.position.y = old_pose.position.y + \
            translation * sin(old_theta + pre_movement_rotation)
        new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, \
            new_pose.orientation.w = tf.transformations.quaternion_from_euler(
                0, 0, old_theta + pre_movement_rotation
                + post_movement_rotation)
    return new_pose


def add_pose_noise(old_pose, pose):
    # print("Old Pose: {}".format(old_pose))
    # print("New Pose: {}".format(pose))
    new_pose = pose.copy()
    if old_pose is not None:
        dx = pose.position.x - old_pose.position.x
        dy = pose.position.y - old_pose.position.y
        translation = sqrt(dx*dx + dy*dy)
        # print("Translation: {}".format(translation))
        q = [old_pose.orientation.x,
             old_pose.orientation.y,
             old_pose.orientation.z,
             old_pose.orientation.w]
        (r, p, old_theta) = tf.transformations.euler_from_quaternion(q)
        # print("Old r: {}, p: {}, theta: {}".format(r, p, old_theta))
        q = [pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w]
        (r, p, theta) = tf.transformations.euler_from_quaternion(q)
        # print("New r: {}, p: {}, theta: {}".format(r, p, theta))

        pre_movement_rotation = atan2(dy, dx) - old_theta
        # print("pre_movement_rotation: {}".format(pre_movement_rotation))
        post_movement_rotation = theta - pre_movement_rotation - old_theta
        # print("post_movement_rotation: {}".format(post_movement_rotation))
        # calculate standard deviations
        sd_pre_movement_rotation = (
            MOVEMENT_NOISE_ALPHA * abs(pre_movement_rotation)
            + MOVEMENT_NOISE_BETA * translation)
        # print("sd_pre_movement_rotation: {}".format(sd_pre_movement_rotation))
        sd_post_movement_rotation = (
            MOVEMENT_NOISE_ALPHA * abs(post_movement_rotation)
            + MOVEMENT_NOISE_BETA * translation)
        # print("sd_post_movement_rotation: {}".format(sd_post_movement_rotation))
        sd_translation = (
            MOVEMENT_NOISE_GAMMA * translation
            + MOVEMENT_NOISE_DELTA * (abs(pre_movement_rotation)
                                      + abs(post_movement_rotation)))
        # print("sd_translation: {}".format(sd_translation))

        translation += np.random.normal(
            0, sd_translation * sd_translation)
        # print("Translation: {}".format(translation))
        pre_movement_rotation += np.random.normal(
            0, sd_pre_movement_rotation * sd_pre_movement_rotation)
        # print("pre_movement_rotation: {}".format(pre_movement_rotation))
        post_movement_rotation += np.random.normal(
            0, sd_post_movement_rotation * sd_post_movement_rotation)
        # print("post_movement_rotation: {}".format(post_movement_rotation))

        new_pose.position.x = old_pose.position.x + \
            translation * cos(old_theta + pre_movement_rotation)
        new_pose.position.y = old_pose.position.y + \
            translation * sin(old_theta + pre_movement_rotation)
        new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, \
            new_pose.orientation.w = tf.transformations.quaternion_from_euler(
                0, 0, old_theta + pre_movement_rotation
                + post_movement_rotation)
    return new_pose


def add_pose_noise2(pose):
    # print("Old Pose: {}".format(old_pose))
    # print("New Pose: {}".format(pose))
    new_pose = pose.copy()
    new_pose.position.x += np.random.normal(0, MOVEMENT_NOISE_ALPHA)
    new_pose.position.y += np.random.normal(0, MOVEMENT_NOISE_ALPHA)
    return new_pose

if __name__ == '__main__':
    pose = Pose(Position(), Position())
    new_pose = Pose(Position(y=2.95), Position())
    fig, axs = plt.subplots(3, 4)
    index2 = 0
    for MOVEMENT_NOISE_ALPHA in [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.2, 0.3]:
        x= []
        y = []
        for index in range(0,1000):
            result_pose = add_pose_noise2(pose)
            x.append(result_pose.position.x)
            y.append(result_pose.position.y)
        print(divmod(index2, 4))
        axs[divmod(index2, 4)].scatter(x, y)
        axs[divmod(index2, 4)].set_title(str(MOVEMENT_NOISE_ALPHA))
        index2 += 1
    plt.show()
