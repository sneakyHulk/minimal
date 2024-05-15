from ecal.core.subscriber import MessageSubscriber
import msgpack


class MsgPackSubscriber(MessageSubscriber):
    """Specialized publisher subscribes to msgpack data
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
    """Specialized publisher subscribes to binary data
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
        self.callback(topic_name, msg, time_)
