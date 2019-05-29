
from numpy import arctan2, abs, mean

from poses import pose_list


class MinimumDifferenceClassifier:

    def __init__(self):
        pass

    def classify(self, joints):
        scores = dict()
        min_score = ('Initial', 1000000)
        for name,pose in pose_list:
            score_list = list()
            for label,angle in pose.items():
                score_list.append(abs(angle - joints[label]))
            score = mean(score_list)
            print(score)
            scores[name] = score
            if score < min_score[1]:
                min_score = (name, score)
        return min_score
