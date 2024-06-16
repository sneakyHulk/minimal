from scripts.sort import Sort
import json
import numpy as np
import cv2
import os
import random

from scripts.ocsort import OCSort


def main_ocsort():
    with open('detections_file.json') as file:
        j = json.loads(file.read())

    tracker = OCSort()
    for frames in j["data"]:
        if frames["src"] != "s110_w_cam_8": continue

        xs = np.array(
            [np.array([detection["left"], detection["top"], detection["right"], detection["bottom"]]) for detection in
             frames["detections"]])

        tracker.update(xs)



def main_sort():
    with open('detections_file.json') as file:
        j = json.loads(file.read())

    tracker = Sort(max_age=3, min_hits=3, iou_threshold=0.1)

    old_timestamp = -1
    for frames in j["data"]:
        if frames["src"] != "s110_w_cam_8": continue

        xs = np.array(
            [np.array([detection["left"], detection["top"], detection["right"], detection["bottom"]]) for detection in
             frames["detections"]])

        print("dt: ", (frames["timestamp"] - old_timestamp) / 1000 if old_timestamp > 0 else 0)
        tracks, unmatched_tracks, unmatched_dets = tracker.update(dets=xs, dt=(frames[
                                                                                   "timestamp"] - old_timestamp) / 1000 if old_timestamp > 0 else 0)

        img = cv2.imread("/home/lukas/src/minimal/data/camera_simulator/s110_w_cam_8/s110_w_cam_8_images/" + str(
            frames["timestamp"]) + ".jpg")

        for track in tracks:
            rng = np.random.default_rng(int(track[4]))
            random.seed(int(track[4]))
            cv2.rectangle(img, (int(track[0]), int(track[1])), (int(track[2]), int(track[3])),
                          (random.randint(0, 256), random.randint(0, 256), random.randint(0, 256)), 20)

        for track in unmatched_tracks:
            rng = np.random.default_rng(int(track[4]))
            random.seed(int(track[4]))
            cv2.rectangle(img, (int(track[0]), int(track[1])), (int(track[2]), int(track[3])),
                          (random.randint(0, 256), random.randint(0, 256), random.randint(0, 256)), 1)

        for track in unmatched_dets:
            cv2.rectangle(img, (int(track[0]), int(track[1])), (int(track[2]), int(track[3])),
                          (0, 0, 255), 20)

        for track in xs:
            cv2.rectangle(img, (int(track[0]), int(track[1])), (int(track[2]), int(track[3])),
                          (0, 0, 0), 1)

        cv2.imshow("img", img)
        cv2.waitKey(0)

        old_timestamp = frames["timestamp"]


if __name__ == '__main__':
    main_sort()
