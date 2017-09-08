#!/usr/bin/env python
import sys
import smbus
import struct
import time

class Romi:
  def __init__(self):
    self.bus = smbus.SMBus(1)
    self.python_version = sys.version_info.major

  def read_unpack(self, address, size, format):
    # Ideally we could do this:
    #    byte_list = self.bus.read_i2c_block_data(20, address, size)
    # But the AVR's TWI module can't handle a quick write->read transition,
    # since the STOP interrupt will occasionally happen after the START
    # condition, and the TWI module is disabled until the interrupt can
    # be processed.
    #
    # A delay of 0.0001 (100 us) after each write is enough to account
    # for the worst-case situation in our example code.
    self.bus.write_byte(20, address)
    time.sleep(0.0001)
    byte_list = [self.bus.read_byte(20) for _ in range(size)]
    if self.python_version == 3:
        # Python version 3
        return struct.unpack(format, bytes(byte_list))
    else:
        # Python version 2
        return struct.unpack(format, bytes(bytearray(byte_list)))

  def write_pack(self, address, format, *data):
    if self.python_version == 3:
        # Python version 3
        data_array = list(struct.pack(format, *data))
    else:
        # Python version 2
        data_array = [ord(char) for char in  list(struct.pack(format, *data))]
    self.bus.write_i2c_block_data(20, address, data_array)
    time.sleep(0.0001)

  def leds(self, red, yellow, green):
    self.write_pack(0, "???", red, yellow, green)

  def read_buttons(self):
    return self.read_unpack(3, 3, "???")

  def velocity_command(self, v_x_command, v_theta_command):
    self.write_pack(6, 'ff', v_x_command, v_theta_command)

  def read_odometry(self):
    return self.read_unpack(14, 20, 'fffff')

  def reset_odometry(self):
    self.write_pack(34, '?', True)

  def read_battery_millivolts(self):
    return self.read_unpack(35, 2, "H")

  def play_notes(self, notes):
    self.write_pack(37, 'B15s', 1, notes.encode("ascii"))

  def test_read8(self):
    self.read_unpack(0, 8, 'cccccccc')

  def test_write8(self):
    self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
    time.sleep(0.0001)
