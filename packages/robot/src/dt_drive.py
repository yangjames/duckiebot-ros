#!/usr/bin/python3

import os
import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, BoolStamped
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import BoolStamped
from duckietown.dtros import DTROS, NodeType, TopicType

class DtDrive(DTROS):
    def __init__(self, node_name):
        super(DtDrive, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        joy_node = f'/{os.environ["VEHICLE_NAME"]}/joy'
        rospy.loginfo(f'Publishing {joy_node}')

        self.joy_pub = rospy.Publisher(
            f'/{os.environ["VEHICLE_NAME"]}/joy',
            Joy,
            queue_size=1
        )

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            print("SPIN")
            joy_msg =  Joy(
                axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            )
            joy_msg.axes[1] = 1.0
            self.joy_pub.publish(joy_msg)
            rate.sleep()


if __name__ == '__main__':
    node = DtDrive(node_name='dt_drive')
    node.run()
    rospy.spin()
