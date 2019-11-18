import time
import os
import sys
import serial
import logging

class usbSerial():

    def __init__(self, port):
        self.device = None
        self.port = port
        self._state = 'disconnected'
        self.log = logging.getLogger(__name__)

    def connect(self):
        try:
            self.device = serial.Serial(
                port=self.port,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.5
            )
            # TODO huh what was this for self.is_connected()
        except serial.serialutil.SerialException as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)
            return False
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)
            return False

        return True

    def close(self):
        self.device.close()

    def is_connected(self):
        # if self.device is None:
        #    raise Exception("usbSerial device not connected: None")
        self._state = 'connected' if self.device.isOpen() else 'disconnected'
        return self._state

    def readlines(self):
        try:
            if self.device.inWaiting() > 0:
                return [ line.strip().decode("utf-8")  for line in self.device.readlines() ]
            else:
                return []
        except serial.serialutil.SerialException as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)
            #if not self.device.isOpen():
            self.device.connect()
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)

    def sendline(self, data):
        if not self.is_connected():
            raise Exception("Connection not open")

        try:
            self.log.debug("Sending: {}".format(data.strip()))
            self.device.write(data.encode())
            #time.sleep(1)
        except Exception as e:
            self.log.error(e)
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)
            return None
