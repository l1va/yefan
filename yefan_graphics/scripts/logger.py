#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
from importlib import import_module
from datetime import datetime


class Config:
    def __init__(self, topic):
        self.topic = topic
        self.fields = []
        self.file = "log.csv"


def parse_config(s):
    x = s.split()
    cfg = Config(x[0])
    fname = "log.csv"
    if len(x) > 1:
        fname = x[1]
        if fname.endswith(".csv"):
            fname = fname[:-4]
    cfg.file = fname + datetime.now().strftime('_%Y-%m-%d_%H:%M:%S') + ".csv"
    if len(x) > 2:
        cfg.fields = x[2].split(",")
    return cfg


class Listener:
    def __init__(self, cfg):
        self.cfg = cfg
        self._binary_sub = rospy.Subscriber(cfg.topic, rospy.AnyMsg, self.binary_callback)
        f = open(cfg.file, "w+")
        f.close()
        with open(cfg.file[:-4]+"_desc.csv", 'w+') as f:
            print(cfg.fields, "of topic",cfg.topic, file=f)

    def binary_callback(self, data):
        self._binary_sub.unregister()
        assert sys.version_info >= (2, 7)  # import_module's syntax needs 2.7
        connection_header = data._connection_header['type'].split('/')
        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        print('Message type detected as ' + msg_type)
        msg_class = getattr(import_module(ros_pkg), msg_type)
        print(" rosPKG: ", ros_pkg)
        self._deserialized_sub = rospy.Subscriber(self.cfg.topic, msg_class, self.deserialized_callback)

    def deserialized_callback(self, data):
        s = str(rospy.get_time()) + ", "
        if len(self.cfg.fields) == 0:
            s += str(data).replace("\n", ", ")
        else:
            for field in self.cfg.fields:
                inners = field.split(".")
                v = data
                for inner in inners:
                    v = getattr(v, inner)
                sv = str(v)
                if isinstance(v, list): # remove brackets if list
                    sv = sv[1:-1]
                s += sv.replace("\n", ", ") + ", "
            s = s[:-2] # drop last comma
        with open(self.cfg.file, 'a') as f:
            print(s, file=f)


def logger():
    rospy.init_node('logger')
    cfgs = []
    i = 1
    while rospy.has_param('~topic' + str(i)):
        val = rospy.get_param('~topic' + str(i))
        i += 1
        cfg = parse_config(val)
        cfgs.append(cfg)

    for cfg in cfgs:
        Listener(cfg)
        print(cfg.topic)

    rospy.spin()


if __name__ == "__main__":
    logger()
