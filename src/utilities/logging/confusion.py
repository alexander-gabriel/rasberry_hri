#!/usr/bin/env python
import json
import sys

from common.utils import ardb
from contextlib import closing

def create_confusion():
    return {"gesture_cancel": 0,
             "calling": 0,
             "gesture_stop": 0,
             "picking_berries_right": 0,
             "picking_berries": 0,
             "neutral": 0,
             "walk_towards": 0,
             "walk_towards_crate": 0,
             "deposit_crate": 0,
             "walk_away": 0,
             "walk_away_crate": 0,
             "return_crate": 0,
             "deliver_crate": 0,
             "pickup_crate": 0,
             "total_count": 0,
             "gesture_backward": 0,
             "gesture_forward": 0}

def get_confusion(typ, table, klass, subject=None):
    if subject:
        try:
            return table[typ][subject][klass]
        except KeyError:
            try:
                table[typ][subject][klass] = create_confusion()
            except KeyError:
                table[typ][subject] = {}
                table[typ][subject][klass] = create_confusion()
            return table[typ][subject][klass]
    else:
        try:
            return table[typ][klass]
        except KeyError:
            table[typ][klass] = create_confusion()
            return table[typ][klass]

if __name__ == '__main__':
    count = 0
    table = {}
    table["score"] = {}
    table["rank"] = {}
    table_combined = {}
    table_combined["score"] = {}
    table_combined["rank"] = {}
    with closing(ardb.db.cursor()) as cursor:
        cursor.execute("SELECT * FROM pose_classifications")
        results = cursor.fetchall()
        for row in results:
            #(timestamp, picker, behaviour, classification)
            if not row[1] == "uninitialized":
                subject_label = row[1].split("-")
                if len(subject_label) == 3:
                    subject, mode = subject_label[1:3]
                else:
                    subject = subject_label[1]
                    mode = "standing"
                klass = row[2].split("-")[0]
                classification = json.loads(row[3])
                best = min(classification, key=classification.get).replace(" ", "_")
                # for typ in ["score", "rank"]:
                #     get_confusion(typ, table, klass)
                #     get_confusion(typ, table_combined, klass)

                confusion = get_confusion("rank", table, klass, subject)
                confusion["total_count"] += 1
                confusion[best] += 1

                confusion_combined = get_confusion("rank", table_combined, klass)
                confusion_combined["total_count"] += 1
                confusion_combined[best] += 1

                confusion = get_confusion("score", table, klass, subject)
                confusion["total_count"] += 1

                confusion_combined = get_confusion("score", table_combined, klass)
                confusion_combined["total_count"] += 1

                for key, score in classification.items():
                    key = key.replace(" ", "_")
                    confusion_combined[key] += score
                    confusion[key] += score
            else:
                count += 1
    for typ in ["score", "rank"]:
        print(typ)
        for subject, klasses in table[typ].items():
            print(subject)
            print(";".join(["actual_behavior", "gesture_call", "gesture_cancel", "gesture_stop", "neutral", "picking_berries", "picking_berries_right", "deliver_crate", "deposit_crate", "pickup_crate", "return_crate", "gesture_forward", "gesture_backward"]))
            for klass, confusion in klasses.items():
                candidates = {}
                for candidate_klass, score in confusion.items():
                    candidates[candidate_klass] = float(score)/confusion["total_count"]
            print(";".join([klass, str(candidates["calling"]), str(candidates["gesture_cancel"]), str(candidates["gesture_stop"]), str(candidates["neutral"]), str(candidates["picking_berries"]), str(candidates["picking_berries_right"]), str(candidates["deliver_crate"]), str(candidates["deposit_crate"]), str(candidates["pickup_crate"]), str(candidates["return_crate"]), str(candidates["gesture_forward"]), str(candidates["gesture_backward"])]))

    for typ in ["score", "rank"]:
        print(typ)
        print(";".join(["actual_behavior", "gesture_call", "gesture_cancel", "gesture_stop", "neutral", "picking_berries", "picking_berries_right", "deliver_crate", "deposit_crate", "pickup_crate", "return_crate", "gesture_forward", "gesture_backward"]))
        for klass, confusion in table_combined[typ].items():
            candidates = {}
            for candidate_klass, score in confusion.items():
                candidates[candidate_klass] = float(score)/confusion["total_count"]
            print(";".join([klass, str(candidates["calling"]), str(candidates["gesture_cancel"]), str(candidates["gesture_stop"]), str(candidates["neutral"]), str(candidates["picking_berries"]), str(candidates["picking_berries_right"]), str(candidates["deliver_crate"]), str(candidates["deposit_crate"]), str(candidates["pickup_crate"]), str(candidates["return_crate"]), str(candidates["gesture_forward"]), str(candidates["gesture_backward"])]))
    # for subject, klasses in table.items():
    #     for klass, confusion in klasses.items():
    #         candidates = {}
    #         for candidate_klass, score in confusion.items():
    #             candidates[candidate_klass] = float(score)/confusion["total_count"]
    #         # avg scores calculation
    #         # for candidate_klass, tuple in confusion.items():
    #         #     candidates[candidate_klass] = tuple[0]
    #         print(";".join([klass, str(candidates["calling"]), str(candidates["gesture_cancel"]), str(candidates["gesture_stop"]), str(candidates["neutral"]), str(candidates["picking_berries"]), str(candidates["picking_berries_right"]), str(candidates["deliver_crate"]), str(candidates["deposit_crate"]), str(candidates["pickup_crate"]), str(candidates["return_crate"]), str(candidates["walk_away"]), str(candidates["walk_away_crate"]), str(candidates["walk_towards"]), str(candidates["walk_towards_crate"])]))
    #         # try:
    #         #     for candidate_klass, tuple in confusion.items():
    #         #         new_avg, new_count = tuple
    #         #         old_avg, old_count = combined[klass][candidate_klass]
    #         #         combined_count = old_count + new_count
    #         #         try:
    #         #             combined_avg = (old_avg*old_count + new_avg*new_count)/combined_count
    #         #         except ZeroDivisionError:
    #         #             combined_avg = new_avg
    #         #         combined[klass][candidate_klass] = [combined_avg, combined_count]
    #         # except KeyError:
    #         #     for candidate_klass, tuple in confusion.items():
    #         #         try:
    #         #             combined[klass][candidate_klass] = [tuple[0], tuple[1]]
    #         #         except KeyError:
    #         #             combined[klass] = {}
    #         #             combined[klass][candidate_klass] = [tuple[0], tuple[1]]
    # print("combined")
    # print(";".join(["actual_behavior", "calling", "gesture_cancel", "gesture_stop", "neutral", "picking_berries", "picking_berries_right", "deliver_crate", "deposit_crate", "pickup_crate", "return_crate", "walk_away", "walk_away_crate", "walk_towards", "walk_towards_crate"]))
    # for klass, confusion in combined.items():
    #     candidates = {}
    #     for candidate_klass, score in confusion.items():
    #         candidates[candidate_klass] = float(score)/confusion["total_count"]
    #     print(";".join([klass, str(candidates["calling"]), str(candidates["gesture_cancel"]), str(candidates["gesture_stop"]), str(candidates["neutral"]), str(candidates["picking_berries"]), str(candidates["picking_berries_right"]), str(candidates["deliver_crate"]), str(candidates["deposit_crate"]), str(candidates["pickup_crate"]), str(candidates["return_crate"]), str(candidates["walk_away"]), str(candidates["walk_away_crate"]), str(candidates["walk_towards"]), str(candidates["walk_towards_crate"])]))
    # avg scores calculation
    # for klass, confusion in combined.items():
    #     candidates = {}
    #     for candidate_klass, tuple in confusion.items():
    #         candidates[candidate_klass] = tuple[0]
    #     print(";".join([klass, str(candidates["calling"]), str(candidates["gesture_cancel"]), str(candidates["gesture_stop"]), str(candidates["neutral"]), str(candidates["picking_berries"]), str(candidates["picking_berries_right"]), str(candidates["deliver_crate"]), str(candidates["deposit_crate"]), str(candidates["pickup_crate"]), str(candidates["return_crate"]), str(candidates["walk_away"]), str(candidates["walk_away_crate"]), str(candidates["walk_towards"]), str(candidates["walk_towards_crate"])]))
    # print(table)
