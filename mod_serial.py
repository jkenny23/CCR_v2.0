import os
import sys
import serial
import logging


class usbSerial:

    def __init__(self, port):
        self.serial = None
        self.port = port
        self._state = 'disconnected'
        self.log = logging.getLogger(__name__)

    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.5
            )
        except serial.serialutil.SerialException:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)
            return False
        except Exception:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)
            return False

        return True

    def close(self):
        self.serial.close()

    def is_connected(self):
        self._state = 'connected' if self.serial.isOpen() else 'disconnected'
        return self._state == "connected"

    def readlines(self):
        try:
            if self.serial.inWaiting() > 0:
                return [line.strip().decode("utf-8") for line in self.serial.readlines()]
            else:
                return []
        except serial.serialutil.SerialException:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)
            self.serial.connect()
        except Exception:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)

    def sendline(self, data):
        if not self.is_connected():
            raise Exception("Connection not open")

        try:
            self.log.debug("Sending: {}".format(data.strip()))
            self.serial.write(data.encode())
        except Exception as e:
            self.log.error(e)
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.log.error(exc_type, fname, exc_tb.tb_lineno)
            return None
