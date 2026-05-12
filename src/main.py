


import rospy

import rospy
from geometry_msgs.msg import TransformStamped


class ViconTracker:
    def __init__(self):
        rospy.init_node('vicon_tracker', anonymous=True)

        self.bomb = None
        self.chair = None
        self.table = None

        rospy.Subscriber('/vicon/bomb_1/bomb_1', TransformStamped, self.bomb_cb)
        rospy.Subscriber('/vicon/chair_1/chair_1', TransformStamped, self.chair_cb)
        rospy.Subscriber('/vicon/table_1/table_1', TransformStamped, self.table_cb)

    def bomb_cb(self, msg):
        t = msg.transform
        self.bomb = {
            'position': (t.translation.x, t.translation.y, t.translation.z),
            'orientation': (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        }

    def chair_cb(self, msg):
        t = msg.transform
        self.chair = {
            'position': (t.translation.x, t.translation.y, t.translation.z),
            'orientation': (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        }

    def table_cb(self, msg):
        t = msg.transform
        self.table = {
            'position': (t.translation.x, t.translation.y, t.translation.z),
            'orientation': (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        }


if __name__ == '__main__':
    tracker = ViconTracker()
    rospy.spin()



