import sys
import time
import signal

sys.path.append('../python_sub//')

import ecal.core.core as ecal_core
from ecal.core.subscriber import MessageSubscriber
import msgpack


class MsgPackSubscriber(MessageSubscriber):
    """Spezialized publisher subscribes to msgpack data
    """

    def __init__(self, name):
        topic_type = "mpack"
        super(MsgPackSubscriber, self).__init__(name, topic_type)
        self.callback = None

    def receive(self, timeout=0):
        """ receive subscriber content with timeout

        :param timeout: receive timeout in ms

        """
        ret, msg, time_ = self.c_subscriber.receive(timeout)
        return ret, msgpack.unpackb(msg) if ret > 0 else None, time_

    def set_callback(self, callback):
        """ set callback function for incoming messages

        :param callback: python callback function (f(topic_name, msg, time))

        """
        self.callback = callback
        self.c_subscriber.set_callback(self._on_receive)

    def rem_callback(self, callback):
        """ remove callback function for incoming messages

        :param callback: python callback function (f(topic_name, msg, time))

        """
        self.c_subscriber.rem_callback(self._on_receive)
        self.callback = None

    def _on_receive(self, topic_name, msg, time_):
        self.callback(topic_name, msgpack.unpackb(msg), time_)


class BinarySubscriber(MessageSubscriber):
    """Spezialized publisher subscribes to binary data
    """

    def __init__(self, name):
        topic_type = "binary"
        super(BinarySubscriber, self).__init__(name, topic_type)
        self.callback = None

    def receive(self, timeout=0):
        """ receive subscriber content with timeout

        :param timeout: receive timeout in ms

        """
        ret, msg, time_ = self.c_subscriber.receive(timeout)
        return ret, msg if ret > 0 else None, time_

    def set_callback(self, callback):
        """ set callback function for incoming messages

        :param callback: python callback function (f(topic_name, msg, time))

        """
        self.callback = callback
        self.c_subscriber.set_callback(self._on_receive)

    def rem_callback(self, callback):
        """ remove callback function for incoming messages

        :param callback: python callback function (f(topic_name, msg, time))

        """
        self.c_subscriber.rem_callback(self._on_receive)
        self.callback = None

    def _on_receive(self, topic_name, msg, time_):
        self.callback(topic_name, msgpack.unpackb(msg), time_)


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
