#!/usr/bin/env python
import os
import rospy
import gclib
import string
import argparse
import threading
import collections
import numpy as np
from criutils import read_parameter
from galilmc import GalilHardwareInterface
# Messages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class GalilDriver(object):
  def __init__(self):
    self.interface = GalilHardwareInterface()
    # Connect to the controller
    if not rospy.has_param('~address'):
      rospy.logerr('Please specify the controller address')
      return
    if not rospy.has_param('~encoder_ppr'):
      rospy.logerr('Please specify encoder pulses per revolution')
      return
    address = read_parameter('~address', None)
    direct = read_parameter('~direct', False)
    subscribe = read_parameter('~subscribe', 'NONE')
    # Connect to and configure the controller
    cmd = self.interface.connect(address, subscribe, direct)
    if not self.interface.is_connected:
      rospy.logerr('Failed to connect to the galilmc: {}'.format(cmd))
      return
    self.encoder_ppr = read_parameter('~encoder_ppr', None)
    self.interface.turn_off()
    config = read_parameter('~config', {})
    for key, value in config.items():
      success = self.interface.send_command(key, value)
      if not success:
        rospy.logwarn('Failed to send command: {0}={1}'.format(key,value))
    # Shutdown hook
    rospy.on_shutdown(self.on_shutdown)
    # Setup subscribers
    self.mutex = threading.Lock()
    self.enabled = dict()
    for i in range(self.interface.num_axes):
      axis = string.ascii_uppercase[i]
      self.enabled[axis] = False
      topic = 'axis_{}/jog'.format(axis.lower())
      rospy.Subscriber(topic, Float64, self.jog_cb, callback_args=axis)
      # Add missing encoder ppr
      if axis not in self.encoder_ppr:
        self.encoder_ppr[axis] = 1.0
    # Setup state publisher
    self.state_pub = rospy.Publisher('state', JointState, queue_size=1)
    self.rate = np.clip(read_parameter('~rate', 100), 1, 1000)

  def jog_cb(self, msg, axis):
    if not self.interface.is_valid_axis(axis):
      rospy.logwarn('Received command for invalid axis: {}'.format(axis))
      return
    counts_per_sec = msg.data * self.encoder_ppr[axis] / 60.
    with self.mutex:
      self.interface.jog(axis, counts_per_sec)

  def on_shutdown(self):
    self.interface.turn_off()
    self.interface.disconnect()

  def start(self):
    if not hasattr(self, 'rate'):
      return
    # Spin
    state = JointState()
    axes_str = string.ascii_uppercase[:self.interface.num_axes]
    state.name = [letter for letter in axes_str]
    state.position = [0. for _ in range(self.interface.num_axes)]
    state.velocity = [0. for _ in range(self.interface.num_axes)]
    state.effort = [0. for _ in range(self.interface.num_axes)]
    r = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      with self.mutex:
        for i,axis in enumerate(axes_str):
          ppr = float(self.encoder_ppr[axis])
          position = self.interface.get_position(axis)
          velocity = self.interface.get_velocity(axis)
          if (position is None) or (velocity is None):
            continue
          state.position[i] = position / ppr * 2*np.pi
          state.velocity[i] = 60 * velocity / ppr
      self.state_pub.publish(state)
      r.sleep()

def parse_args():
  # Remove extra IPython notebook args
  clean_argv = rospy.myargv()[1:]
  if '-f' in clean_argv:
    clean_argv = clean_argv[2:]
  # Parse
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='ROS driver for galil motion controller')
  parser.add_argument('--debug', action='store_true',
    help='If set, will show debugging messages')
  args = parser.parse_args(clean_argv)
  return args


if __name__ == '__main__':
  # Configure the node
  args = parse_args()
  log_level= rospy.DEBUG if args.debug else rospy.INFO
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name, log_level=log_level)
  # Start the driver
  driver = GalilDriver()
  driver.start()
