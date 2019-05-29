
from numpy import arctan2, abs, mean

from poses import pose_list


class MinimumDifferenceClassifier:

    def __init__(self):
        pass

    def classify(self, joints):
        scores = dict()
        max_score = ('Initial', 0.0)
        for name,pose in pose_list:
            score_list = list()
            for label,angle in pose.items():
                score_list.append(abs(angle - joints[label]))
            score = mean(score_list)
            scores[name] = score
            if score > max_score[1]:
                max_score = (name, score)
        return max_score
