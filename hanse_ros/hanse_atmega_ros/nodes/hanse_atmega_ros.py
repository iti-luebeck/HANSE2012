#!/usr/bin/env python
import roslib
roslib.load_manifest('hanse_atmega_ros')
import rospy
import serial
import struct
import sys
import exceptions
from time import sleep
from threading import Thread
from hanse_msgs.msg import pressure
from hanse_msgs.msg import temperature
from hanse_msgs.msg import sollSpeed
from std_msgs.msg import Empty
from threading import Lock
from threading import Event
class Comm:
    def __init__(self, device):
        self._device = device
        self._baud = 57600
        self._port = self._open_port()
        self._write_lock = Lock()
        self._read_lock = Lock()
        self._reinit_lock = Lock()

    def _open_port(self):
        return serial.Serial(self._device, self._baud)

    def _oc_add(self, a, b):
        s = a + b
        if s < 256:
            return s
        else:
            return s - 255

    def _checksum(self, data):
            a, b = 0, 0
            for byte in data:
                a = (a + ord(byte)) & 0xFF #self._oc_add(a, ord(byte))
                b = (b + a) & 0xFF #self._oc_add(b, a)
            return a, b

    def _make_packet(self, data):
        data = chr(len(data)) + data
        ck_a, ck_b = self._checksum(data)
        return '\x13\x37' + data + struct.pack('2B', ck_a, ck_b)

    def _write_packet(self, data):
        with self._write_lock:
            try:
                self._port.write(self._make_packet(data))
                self._port.flush()
            except (serial.serialutil.SerialException, exceptions.AttributeError):
                with self._reinit_lock:
                    try:
                        self._port = None
                        sleep(0.1)
                        port = self._open_port()
                        self._port = port
                    except:
                        pass

    def _read_packet(self):
        b = self._port.read(1)
        while True:
            if b == '':
                return None
            if b == '\x13':
                b = self._port.read(1)
                if b == '\x37':
                    break
            else:
                b = self._port.read(1)
        length_byte = self._port.read(1)
        length = ord(length_byte)
        payload = self._port.read(length)
        ck = tuple(map(ord, self._port.read(2)))
        if self._checksum(length_byte + payload) != ck:
            print "chk", ck, self._checksum(length_byte + payload)
            return None
        return payload

    def set_motor_speeds(self, left, right, front, back):
        self._write_packet(struct.pack('2B4b', 0x01, 8, left, right, front, back))

    def stop_motors(self):
        self._write_packet(struct.pack('1B', 0x02))


    _list = [
        "no_error",
        "unknown_error",
        "checksum_error",
        "serial_abort",
        "serial_receive_error",
        "serial_receive_overflow",
        "i2c_abort",
        "i2c_arb_lost",
        "i2c_write_nack",
        "i2c_read_nack",
        "i2c_receive_overflow"
        ]

    def get_message(self):
        with self._read_lock:
            try:
                packet = self._read_packet()
                if packet == None:
                    return 'error', 'checksum error'
                cmd = ord(packet[0])
                if cmd == 0x03:
                    pressure, temp = struct.unpack('<2H', packet[1:])
                    return 'pressuretemp', pressure, temp
                elif cmd == 0x04:
                    a, b = struct.unpack('<2B', packet[1:])
                    if a == 0x01:
                        return 'error', 'wdt reset'
                    elif a == 0x02 and b < len(self._list):
                        return 'error', self._list[b]
                    else:
                        return 'error', 'unknown error reported'
                else:
                    return 'error', 'unknown command received'
            except (serial.serialutil.SerialException, exceptions.AttributeError):
                with self._reinit_lock:
                    try:
                        self._port = None
                        sleep(0.1)
                        port = self._open_port()
                        self._port = port
                    except:
                        pass
                    return 'error', 'serial port error'
            except:
                return 'error', 'python error: ' + str(sys.exc_info()[0])

class Node:
    def __init__(self):
        rospy.init_node("hanse_atmega_ros", disable_signals=True)
        self._comm = Comm(rospy.get_param('~serial_port'))
        self._depth = rospy.Publisher('pressure/depth', pressure)
        self._temp = rospy.Publisher('pressure/temp', temperature)
        self._left = rospy.Subscriber('motors/left', sollSpeed,
                                      callback=self._motor, callback_args=['left'], queue_size = 1)
        self._right = rospy.Subscriber('motors/right', sollSpeed,
                                      callback=self._motor, callback_args=['right'], queue_size = 1)
        self._up = rospy.Subscriber('motors/up', sollSpeed,
                                    callback=self._motor, callback_args=['front', 'back'], queue_size = 1)
        self._empty = rospy.Subscriber('motors/stop', Empty,
                                       callback=self._stop, queue_size = 1)
        self._state_lock = Lock()
        self._event = Event()
        self._stop_state()

    def _stop_state(self):
        self._state = {'stopped': True, 'left': 0, 'right': 0, 'front': 0, 'back': 0}

    def start(self):
        try:
            self._receive_thread = Thread(target=self._receive)
            self._receive_thread.daemon = True
            self._receive_thread.start()
            self._transmit()
        except:
            self._receive_thread.kill_received = True
            raise

    def _motor(self, msg, which):
        with self._state_lock:
            self._state['stopped'] = False
            for i in which:
                self._state[i] = msg.data
            self._event.set()

    def _stop(self, msg):
        with self._state_lock:
            self._stop_state()
            self._event.set()

    def _receive(self):
        while True:
            msg = self._comm.get_message()
            if msg[0] == 'pressuretemp':
                pressure_val, temp_val = msg[1:]
                t = rospy.Time.now()
                msg = pressure(data=pressure_val)
                msg.header.stamp = t
                self._depth.publish(msg)
                msg = temperature(data=temp_val)
                msg.header.stamp = t
                self._temp.publish(msg)
            else:
                rospy.logerr(repr(msg))

    def _transmit(self):
        self._comm.stop_motors()
        sleep(0.5)
        self._comm.stop_motors()
        while True:
            self._event.wait(0.1)
            with self._state_lock:
                self._event.clear()
                if self._state['stopped']:
                    self._comm.stop_motors()
                else:
                    self._comm.set_motor_speeds(
                        self._state['left'],
                        self._state['right'],
                        self._state['front'],
                        self._state['back'])

if __name__ == '__main__':
    try:
        node = Node()
        node.start()
    except KeyboardInterrupt:
        rospy.rospy.signal_shutdown("keyboard interrupt")
        exit(0)
