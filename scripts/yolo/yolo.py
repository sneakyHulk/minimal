from ultralytics import YOLO
import torch
import signal
from pathlib import Path
from lib.ecal.scripts.eCALSubscriber import BinarySubscriber
import sys
import time
import ecal.core.core as ecal_core
import cv2
import numpy as np
from ultralytics.utils.plotting import Annotator

gSignalStatus = 0


def signal_handler(signum, frame):
    global gSignalStatus
    print("Got SIGINT/SIGTERM!")
    gSignalStatus = signum


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def OnImage(ret, msg, time_):
    now_ns = time.time_ns() / 1000
    print("Time taken = {} μs".format(now_ns - time_))

    data = np.asanyarray(msg)
    data = data.view(np.uint8).reshape((1200, 1920, 3))


def main():
    global gSignalStatus
    ecal_core.initialize(sys.argv, Path(__file__).stem)
    print("Initialized eCAL unit with unit_name '", Path(__file__).stem, "'!")
    print("GPU mode inference" if torch.cuda.is_available() else "CPU mode inference")
    device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')
    model = YOLO('yolov8n.pt')

    sub = BinarySubscriber("image")
    # sub.set_callback(OnImage)

    while not gSignalStatus and ecal_core.ok():
        msg = sub.receive()

        if msg[0]:
            data = np.frombuffer(msg[1])
            data = data.view(np.uint8).reshape((1200, 1920, 3))

            results = model(data, device=device, classes=[0, 1, 2, 3, 5, 7])

            for r in results:
                annotator = Annotator(data)

                for box in r.boxes:
                    b = box.xyxy[0]
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])

                now_ns = time.time_ns() / 1000
                print("Time taken = {} μs".format(now_ns - msg[2]))

                cv2.imshow('display', annotator.result())

            cv2.waitKey(1) & 0xFF

    cv2.destroyAllWindows()
    cv2.waitKey(1)
    print("Finalized eCAL unit!")
    ecal_core.finalize()


if __name__ == "__main__":
    main()
