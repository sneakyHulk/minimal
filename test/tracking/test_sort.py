from scripts.sort import Sort
import json
import numpy as np
import cv2
import os
import random

if __name__ == '__main__':
    with open('detections_file.json') as file:
        j = json.loads(file.read())

    tracker = Sort()

    for frames in j["data"]:
        if frames["src"] != "s110_w_cam_8": continue

        xs = np.array(
            [np.array([detection["left"], detection["top"], detection["right"], detection["bottom"]]) for detection in
             frames["detections"]])

        tracks = tracker.update(dets=xs)

        img = cv2.imread("/home/lukas/src/minimal/data/camera_simulator/s110_w_cam_8/s110_w_cam_8_images/" + str(
            frames["timestamp"]) + ".jpg")



        for track in tracks:
            rng = np.random.default_rng(int(track[4]))
            random.seed(track[4])
            cv2.rectangle(img, (int(track[0]), int(track[1])), (int(track[2]), int(track[3])),
                          (random.randint(0, 256), random.randint(0, 256), random.randint(0, 256)), 5)

        cv2.imshow("img", img)
        cv2.waitKey(20)
