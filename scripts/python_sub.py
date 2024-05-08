import sys
import time
import signal

import ecal.core.core as ecal_core
from eCALSubscriber import MsgPackSubscriber
import msgpack

gSignalStatus = 0


def signal_handler(signum, frame):
    global gSignalStatus
    print("Got SIGINT/SIGTERM! Will finalize eCAL.")
    gSignalStatus = signum


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def OnTimestamp(ret, msg, time_):
    now_ns = time.time_ns()
    print("Time taken = {} ns".format(now_ns - msg[0]))
    print(msg[1])


def main():
    global gSignalStatus
    ecal_core.initialize(sys.argv, "python_sub")

    sub = MsgPackSubscriber("timestamp")
    sub.set_callback(OnTimestamp)

    while not gSignalStatus and ecal_core.ok():
        pass

    ecal_core.finalize()


if __name__ == "__main__":
    main()
