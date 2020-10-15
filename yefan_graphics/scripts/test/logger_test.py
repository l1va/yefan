import unittest
import rospy
from yefan_graphics.scripts.logger import Listener, Config
from roscore import Roscore


class DataObj:
    def __init__(self, header, vel, pos):
        self.header = header
        self.vel = vel
        self.pos = pos

    def __str__(self):
        return str(self.header) + ", " + str(self.vel) + ", " + str(self.pos)


class Pos:
    def __init__(self, joint1, joint2):
        self.joint1 = joint1
        self.joint2 = joint2

    def __str__(self):
        return str(self.joint1) + ", " + str(self.joint2)


LOGGER_CASES = [
    dict(
        fields=[],
        data=[DataObj("test_header", 5.123, -322.0),
              DataObj("test_header2", 6.789, 12.0333333)
              ],
        res=["test_header, 5.123, -322.0",
             "test_header2, 6.789, 12.0333333",
             ]
    ),
    dict(
        fields=["header"],
        data=[DataObj("test_header", 5.123, -322.0),
              DataObj("test_header2", 6.789, 12.0333333)
              ],
        res=["test_header",
             "test_header2",
             ]
    ),
    dict(
        fields=["header", "pos"],
        data=[DataObj("test_header", 5.123, -322.0),
              DataObj("test_header2", 6.789, 12.0333333)
              ],
        res=["test_header, -322.0",
             "test_header2, 12.0333333",
             ]
    ),
    dict(
        fields=["header", "pos"],
        data=[DataObj("test_header", 5.123, Pos(13, 456.7)),
              DataObj("test_header2", 6.789, Pos(76, 432.1))
              ],
        res=["test_header, 13, 456.7",
             "test_header2, 76, 432.1",
             ]
    ),
    dict(
        fields=["vel", "pos.joint2"],
        data=[DataObj("test_header", 5.123, Pos(13, 456.7)),
              DataObj("test_header2", 6.789, Pos(76, 432.1))
              ],
        res=["5.123, 456.7",
             "6.789, 432.1",
             ]
    ),
    dict(
        fields=["vel", "pos.joint2.joint1"],
        data=[DataObj("test_header", 5.123, Pos(1, Pos(2, 3))),
              DataObj("test_header2", 6.789, Pos(7, Pos(8, 9))),
              ],
        res=["5.123, 2",
             "6.789, 8",
             ]
    ),
    dict(
        fields=["header", "pos.joint2"],
        data=[DataObj("test_header", 5.123, Pos(1, [1, 2, 3])),
              DataObj("test_header2", 6.789, Pos(7, [4.5, 6.7, 8.9])),
              ],
        res=["test_header, 1, 2, 3",
             "test_header2, 4.5, 6.7, 8.9",
             ]
    )
]


class TestLogger(unittest.TestCase):
    def setUp(self):
        self.roscore = Roscore()
        self.roscore.run()
        rospy.init_node('logger_test')

    def tearDown(self):
        self.roscore.terminate()

    def test_callback(self):
        file = "test_file.csv"

        for i in range(len(LOGGER_CASES)):

            c = LOGGER_CASES[i]

            cfg = Config("/test_topic")
            cfg.fields = c["fields"]
            cfg.file = file

            l = Listener(cfg)
            for dat in c["data"]:
                l.deserialized_callback(dat)

            with open(file, 'r') as f:
                contents = f.read().rstrip().split("\n")
                res = c["res"]
                self.assertEqual(len(res), len(contents), "len is different:" + str(res) + " and " + str(contents))
                for j in range(len(contents)):
                    cont = contents[j]
                    cont = cont[cont.find(",") + 2:]  # drop rostime
                    self.assertEqual(res[j], cont)


if __name__ == '__main__':
    unittest.main()
