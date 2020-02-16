import rospy

from numpy import arctan2, abs, mean

from poses import pose_list
from utils import suppress

class MinimumDifferenceClassifier:

    def __init__(self):
        self.limit = rospy.get_param("classification_error", 20)
        self.angle_weight = rospy.get_param("angle_weight", 0.25)

    def classify(self, angles, positions):
        errors = dict()
        min_error = ('Initial', 1000000)
        rospy.logdebug("CLA: -----------------")
        for name, pose in pose_list.items():
            error_list = list()
            rospy.logdebug("CLA: checking pose: {}".format(name))
            for label, target in pose.items():
                # with suppress(TypeError):
                    rospy.logdebug("CLA: target: {}:{}".format(label, target))
                    angle_or_position, direction, limit = target.split(" ")
                    if angle_or_position == "a":
                        try:
                            limit = float(limit)
                        except:
                            limit = angles[limit]
                        if direction == "=":
                            rospy.logdebug("CLA: error: {}-{}={}".format(limit, angles[label], abs(limit - angles[label])))
                            error_list.append(self.angle_weight * abs(limit - angles[label]))
                        elif direction == ">":
                            rospy.logdebug("CLA: error: max(0,{}-{})={}".format(limit, angles[label], abs(limit - angles[label])))
                            error_list.append(self.angle_weight * max(0, limit - angles[label]))
                        elif direction == "<":
                            rospy.logdebug("CLA: error: max(0,{}-{})={}".format(limit, angles[label], abs(angles[label] - limit)))
                            error_list.append(self.angle_weight * max(0, angles[label] - limit))
                        else:
                            rospy.logerr("CLA: wrong direction code {}".format(direction))
                            pass
                    else:
                        if positions[label] is None:
                            error_list.append(self.limit)
                            break
                        try:
                            limit = float(limit)
                        except:
                            limit = positions[limit]
                        if direction == "=":
                            rospy.logdebug("CLA: error: {}-{}={}".format(limit, positions[label], abs(limit - positions[label])))
                            error_list.append(abs(limit - positions[label]))
                        elif direction == ">":
                            rospy.logdebug("CLA: error: max(0,{}-{})={}".format(limit, positions[label], abs(limit - positions[label])))
                            error_list.append(max(0, limit - positions[label]))
                        elif direction == "<":
                            rospy.logdebug("CLA: error: max(0,{}-{})={}".format(limit, positions[label], abs(positions[label] - limit)))
                            error_list.append(max(0, positions[label] - limit))
                        else:
                            rospy.logerr("CLA: wrong direction code {}".format(direction))
                            pass
            error = mean(error_list)
            errors[name] = error
            if error < min_error[1]:
                min_error = (name, error)
            rospy.logdebug("CLA: {:}: {:f}".format(name, error))
        rospy.logdebug("CLA: -----------------")
        rospy.logdebug("CLA: {:}: {:f}".format(min_error[0], min_error[1]))
        return min_error
