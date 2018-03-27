#!/usr/bin/env python
import enum
import gclib
import string
import collections
import numpy as np

_EPS = np.finfo(float).eps

class ControlMode(enum.Enum):
  NONE = 1
  JOG = 2
  POSITION_TRACKING = 3

class GalilHardwareInterface(object):
  def __init__(self):
    # Default values
    self._reset()
    # Instance of the gclib python class
    self.g = gclib.py()

  def _reset(self):
    self.num_axes = -1
    self.connected = False
    self.mode = dict()

  def connect(self, address, subscribe, direct=True):
    """
    Connect to a Galil Controller.

    Parameters
    ----------
    address: str
      Simple address to hardware. It can be IP address, PCI, COM port
    subscribe: str or :class:`SubscribeType`
      Subscribe to messages, data records, and/or interrupts
    direct: bool, optional
      Connect directly to hardware instead of via gcaps

    Returns
    -------
    cmd: str
      The string sent to the controller using `GOpen`
    """
    cmd = address
    if direct:
      cmd += ' --direct'
    if isinstance(subscribe, str):
      cmd += ' -s {}'.format(subscribe)
    elif isinstance(subscribe, SubscribeType):
      cmd += ' -s {}'.format(subscribe.name)
    try:
      self.g.GOpen(cmd)
      # Report we managed to connect and get the number of axes
      self.connected = True
      self.num_axes = len(self.g.GCommand('TP*=?').split(','))
      for i in range(self.num_axes):
        axis = string.ascii_uppercase[i]
        self.mode[axis] = ControlMode.NONE
    except gclib.GclibError as e:
      self._reset()
    return cmd

  def disconnect(self):
    self._reset()
    try:
      self.g.GClose()
      success = True
    except gclib.GclibError:
      success = False
    return success

  def get_position(self, axis):
    if not self.connected:
      return None
    axis = axis.upper()
    try:
      position = float(self.g.GCommand('TP{}'.format(axis)))
    except (gclib.GclibError, ValueError):
      position = None
    return position

  def get_position_error(self, axis):
    if not self.connected:
      return None
    axis = axis.upper()
    try:
      position_error = float(self.g.GCommand('TE{}'.format(axis)))
    except (gclib.GclibError, ValueError):
      position_error = None
    return position_error

  def get_velocity(self, axis):
    if not self.connected:
      return None
    axis = axis.upper()
    try:
      velocity = float(self.g.GCommand('TV{}'.format(axis)))
    except (gclib.GclibError, ValueError):
      velocity = None
    return velocity

  def get_torque(self, axis):
    if not self.connected:
      return None
    axis = axis.upper()
    try:
      torque = float(self.g.GCommand('TT{}'.format(axis)))
    except (gclib.GclibError, ValueError):
      torque = None
    return torque

  def jog(self, axis, counts_per_sec):
    """
    Parameters
    ----------
    axis: str
      Letter that identifies the axis (A, B, C, etc)
    counts_per_sec: int
      The target jog speed of the axis. The units of this are [counts/second].
    """
    axis = axis.upper()
    if axis not in self.mode.keys():
      raise ValueError('Jog command received invalid axis')
    # Check the control mode is valid
    if self.mode[axis] == ControlMode.NONE:
      self.mode[axis] = ControlMode.JOG
    elif self.mode[axis] != ControlMode.JOG:
      raise TypeError('Cannot process jog command. Invalid control mode')
    # Process the command
    if self.is_moving(axis):
      self.send_command('JG{}'.format(axis), int(counts_per_sec))
    else:
      self.send_command('SH{}'.format(axis))
      self.send_command('JG{}'.format(axis), int(counts_per_sec))
      self.send_command('BG{}'.format(axis))

  def is_connected(self):
    return self.connected

  def is_moving(self, axis):
    axis = axis.upper()
    try:
      res = float(self.g.GCommand('MG _BG{}'.format(axis)))
    except gclib.GclibError:
      res = 0.
    return res > _EPS

  def is_valid_axis(self, axis):
    is_valid = False
    axis = axis.upper()
    if axis in string.ascii_uppercase:
      is_valid = string.ascii_uppercase.index(axis) < self.num_axes
    return is_valid

  def position_tracking(self, axis, counts, counts_per_sec):
    """
    Parameters
    ----------
    axis: str
      Letter that identifies the axis (A, B, C, etc)
    counts: int
      The absolute position target. The units of this are [counts].
    counts_per_sec: int
      The target speed of the axis. The units of this are [counts/second].
    """
    axis = axis.upper()
    if axis not in self.mode.keys():
      raise ValueError('Position command received invalid axis')
    # Check the control mode is valid
    if self.mode[axis] == ControlMode.NONE:
      self.mode[axis] = ControlMode.POSITION_TRACKING
      # Change the coordinate system and enable the servo control
      self.send_command('SH{}'.format(axis))
      self.send_command('BG{}'.format(axis))
      # Enable position tracking
      self.send_command('DP{}'.format(axis), 0) # Zero the absolute position
      self.send_command('PT{}'.format(axis), 1) # Start PT mode
    elif self.mode[axis] != ControlMode.POSITION_TRACKING:
      raise TypeError('Cannot process position command. Invalid control mode')
    # Process the command
    self.send_command('SP{}'.format(axis), int(counts_per_sec))
    self.send_command('PA{}'.format(axis), int(counts))

  def send_command(self, key, value=None):
    cmd = key
    if isinstance(value, (int, float, basestring)):
      cmd += ' = ' + str(value)
    elif isinstance(value, collections.Sequence):
      cmd += ' = ' + ', '.join(map(str,value))
    elif value is not None:
      raise TypeError('Unsupported value type: {}'.format(type(value)))
    try:
      self.g.GCommand(cmd)
      success = True
    except gclib.GclibError:
      success = False
    return success

  def stop(self, axis=None):
    success = False
    if self.connected:
      if axis is None:
        success = self.send_command('ST')
      elif self.is_valid_axis(axis):
        axis = axis.upper()
        success = self.send_command('ST{}'.format(axis))
    # Reset control mode
    if success:
      if axis is None:
        axes = self.mode.keys()
      elif self.is_valid_axis(axis):
        axes = [axis]
      for a in axes:
        self.mode[a.upper()] = ControlMode.NONE
    return success

  def turn_off(self, axis=None):
    """
    Turn off the specified motor. If `axis=None`, turn off all the motors

    Parameters
    ----------
    axis: str
      Uppercase letter that identifies the axis (A, B, C, etc)

    Returns
    -------
    success: bool
      True if succeeded, False otherwise.

    Notes
    -----
    This command attempts to stop the axes before turning then off. This is
    because a `MO` command will fail if the axis is moving
    """
    success = False
    if self.connected:
      self.stop(axis)
      if axis is None:
        success = self.send_command('MO')
      elif self.is_valid_axis(axis):
        axis = axis.upper()
        success = self.send_command('MO{}'.format(axis))
    return success


class SubscribeType(enum.Enum):
  """
  Flag to indicate whether to subscribe to messages, data records, and/or
  interrupts
  """
  NONE = 1
  MG = 2
  DR = 3
  EI = 4
  ALL = 5


if __name__ == '__main__':
  # Simple usage example
  from galilmc import GalilHardwareInterface
  interface = GalilHardwareInterface()
  cmd = interface.connect('192.168.0.41', subscribe='ALL', direct=False)
  if not interface.is_connected():
    raise Exception('Failed to connect to the galilmc: {}'.format(cmd))
  # Setup
  interface.send_command('AUA', 1)
  interface.send_command('AGA', 2)
  interface.send_command('TLA', 7)
  interface.send_command('CEA', 3)
  interface.send_command('MTA', -1)
  interface.send_command('BRA', 0)
  interface.send_command('ACA', 15432704)
  interface.send_command('DCA', 15432704)
  interface.send_command('ERA', 2500)
  interface.send_command('OEA', 0)
  interface.send_command('KPA', 6)    # Low gain to avoid overshooting
  interface.send_command('KDA', 64)
  # Test the position tracking
  rpm = 1400
  encoder_ppr = 2500
  depth = 8e-3
  pitch = 0.8e-3
  counts = (depth / pitch) * encoder_ppr
  counts_per_sec = rpm * encoder_ppr / 60.
  interface.position_tracking('A', counts, counts_per_sec)
  # Clean-up
  interface.turn_off()
  interface.disconnect()
