import rospy

from numpy import arctan2, abs, mean
from rasberry_hri.msg import Classification, Pose, Criterium
from poses import pose_list
from utils import suppress
from math import pi

class MinimumDifferenceClassifier:

    def __init__(self):
        self.limit = rospy.get_param("classification_error", 20)
        self.angle_weight = rospy.get_param("angle_weight", 1.0)

    def classify(self, angles, positions):
        errors = dict()
        classification = Classification()
        classification.poses = list()
        min_error = ('Initial', 1000000)
        rospy.logdebug("CLA: -----------------")
        for name, pose in pose_list.items():
            classification.poses.append(Pose())
            classification.poses[-1].label = name
            classification.poses[-1].criteria = list()
            error_list = [self.limit]
            rospy.logdebug("CLA: checking pose: {}".format(name))
            for label, target in pose.items():
                    value = None
                # with suppress(TypeError):
                    rospy.logdebug("CLA: target: {}:{}".format(label, target))
                    angle_or_position, direction, limit = target.split(" ")
                    if angle_or_position == "a":
                        try:
                            limit = float(limit)
                        except:
                            limit = angles[limit]
                        value = angles[label]
                    else:
                        # if positions[label] is None:
                        #     error_list.append(self.limit)
                        #     break
                        try:
                            limit = float(limit)
                        except:
                            limit = positions[limit]
                        value = positions[label]
                    error = False

                    if limit is None or abs(limit - 3.14159) < 0.001:
                        if limit is None:
                            rospy.logwarn("DETECTED NONE ERROR")
                            rospy.loginfo(positions)
                        else:
                            rospy.logwarn("DETECTED PI ERROR")
                        limit = pi
                        rospy.logwarn("{}".format(label))
                        error = True
                    if value is None or abs(value - 3.14159) < 0.001:
                        if value is None:
                            rospy.logwarn("DETECTED NONE ERROR")
                        else:
                            rospy.logwarn("DETECTED PI ERROR")
                        value = pi
                        rospy.logwarn("{}".format(label))
                        error = True
                    if not error:
                        if direction == "=":
                            rospy.logdebug("CLA: error: {}-{}={}".format(limit, value, abs(limit - value)))
                            error_list.append(abs(limit - value))
                        elif direction == ">":
                            rospy.logdebug("CLA: error: max(0,{}-{})={}".format(limit, value, abs(limit - value)))
                            error_list.append(max(0, limit - value))
                        elif direction == "<":
                            rospy.logdebug("CLA: error: max(0,{}-{})={}".format(limit, value, abs(value - limit)))
                            error_list.append(max(0, value - limit))
                        else:
                            rospy.logerr("CLA: wrong direction code {}".format(direction))
                            pass
                    else:
                        error_list.append(self.limit)
                    criterium = Criterium()
                    criterium.code = "Angle: {}".format(label) if angle_or_position == "a" else "Position: {}".format(label)
                    criterium.limit = limit
                    criterium.value = value
                    criterium.error = error_list[-1]
                    classification.poses[-1].criteria.append(criterium)
            error = mean(error_list)
            if error < min_error[1]:
                min_error = (name, error)
            rospy.logdebug("CLA: {:}: {:f}".format(name, error))
        rospy.logdebug("CLA: -----------------")
        rospy.logdebug("CLA: {:}: {:f}".format(min_error[0], min_error[1]))
        return min_error, classification
