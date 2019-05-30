
from numpy import arctan2, abs, mean

from poses import pose_list


class MinimumDifferenceClassifier:

    def __init__(self):
        self.limit = 20

    def classify(self, joints):
        errors = dict()
        min_error = ('Initial', 1000000)
        for name,pose in pose_list.items():
            error_list = list()
            for label,angle in pose.items():
                error_list.append(abs(angle - joints[label]))
            error = mean(error_list)
            errors[name] = error
            if error < min_error[1]:
                min_error = (name, error)
        return min_error
