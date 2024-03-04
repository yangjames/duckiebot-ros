#!/usr/bin/env python3
from pynput import keyboard
from duckietown.dtros import DTROS, NodeType
import rospy
from sensor_msgs.msg import Joy
import contextlib
import time

import sys
import termios

@contextlib.contextmanager
def echo_disabled():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~termios.ECHO          # lflags
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, new)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class KeyboardPublisher(DTROS):
    def __init__(self, node_name):
        super(KeyboardPublisher, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self.joy_publisher = rospy.Publisher('~joy', Joy, queue_size=1)
        self.current_key = None
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()  # start to listen on a separate thread

    def on_press(self, key):
        k = None
        if isinstance(key, keyboard._xorg.KeyCode):
            k = key.char
        else:
            k = key.name
        self.current_key = k

    def on_release(self, key):
        self.current_key = None

    def on_shutdown(self):
        self.listener.stop()
        joy_msg = Joy(
            axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )
        joy_msg.header.stamp = rospy.Time.from_sec(time.time())
        self.joy_publisher.publish(joy_msg)
        
    def run(self):
        desired_rate = 10
        rate = rospy.Rate(desired_rate)
        start = time.time()
        it = 0
        # with echo_disabled():
        while self.listener.running and not rospy.is_shutdown():
            # Stamp and measure everything.
            now = time.time()
            it += 1
            hz_estimate = it / (now - start)
        
            if self.current_key in ['q', 'esc']:
                rospy.loginfo('Received quit signal [\'ESC\' or \'q\']. Shutting down...')
                rospy.signal_shutdown()
            joy_msg = Joy(
                axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            )
            joy_msg.header.stamp = rospy.Time.from_sec(now)
            if self.current_key == 'up':
                joy_msg.axes[1] = 1.0
            elif self.current_key == 'down':
                joy_msg.axes[1] = -1.0
            elif self.current_key == 'right':
                joy_msg.axes[3] = -1.0
            elif self.current_key == 'left':
                joy_msg.axes[3] = 1.0
            elif self.current_key == 'e':
                joy_msg.buttons[3] = 1
            self.joy_publisher.publish(joy_msg)
            # rospy.loginfo(f'Desired rate: {desired_rate}, Actual rate: {hz_estimate:.3f}, Key: {self.current_key}')
            rospy.loginfo(joy_msg)
            rate.sleep()

if __name__ == '__main__':
    node = KeyboardPublisher('keyboard_publisher_node')
    node.run()
