import sys
import os
import time
from mod_serial import usbSerial
import sqlite3

usb_dev = 'COMxx'
test_line = "y1 i500 k2500\n"

class Cycler():
    def __init__(self):
        self.device = None

        # TODO total_cyclers hard coded
        total_cyclers = 2

    def init(self):
        while True:
            # Connect
            #________
            self.device = usbSerial(usb_dev)
            try:
                self.device.connect()
            except Exception as e:
                if not self.device:
                    print("Closing serial")
                    self.device.close()
                print("Serial connect failed: {}".format(e))
                time.sleep(5)
            else:
                print("Connected to serial")
                break

        while True:
            # Initialize
            #___________
            try:
                while not self.sync():
                    time.sleep(0.1)
                print("Arduino sync completed")
            except Exception as e:
                time.sleep(5)
                print("Arduino sync failure: {}".format(e))
            else:
                print("Synced with arduino")
                break

    def connect(self):
        print('Connecting')

        while not self.device.connect():
            time.sleep(1)
        print('Connected')

        return True

    def disconnect(self):
        self.device.sendline("n1\n")
        self.device.sendline("n2\n")
        self.device.close()

    # Communicate to arduino, ensure response
    #________________________________________
    def sync(self):
        if not self.device:
            raise("Device not connected, device is: {}".format(self.device))

        print("Sending NL to get a prompt")
        self.device.sendline("\n")

        print("Fetching received lines")
        data = self.device.readlines()
        for line in data:
            print("Received: {}".format(line))

        print("Sending ? to get menu")
        self.device.sendline("?\n")
        time.sleep(0.1)

        data = self.device.readlines()
        for line in data:
            print("Received: {}".format(line))
        for line in data:
            print("checking: {}".format(line))
            if '> Select Mode:'  in line:
                print('Initialized')
                return True

        return False

    # Run
    #_______________
    def start(self):
        # Initialize serial device
        self.init()
        print('Cycler process running')
        #
        # self.conn = sqlite3.connect('cell_database.db')
        # self.cur = self.conn.cursor()

        self.device.sendline(test_line)

        while True:
            if self.device is None:
                print("Re-initializing lost comms")
                self.init()

            #
            # check for serial data
            #_______________
            for line in self.device.readlines():
                if line:
                    self.process_cycle_data(line)

    def create_tables(self):

        self.conn = sqlite3.connect('cell_database.db')
        self.cur = self.conn.cursor()

        # Create table
        # try:
        #     self.cur.execute('''CREATE TABLE cells
        #                          (cell_id text, cell_model text)''')
        # except Exception as e:
        #     print(e)

        try:
            self.cur.execute('''CREATE TABLE cell_data
                        (msg_type integer, millivolt integer, milliamp integer, millamphour real, 
                         millwatthour real, temp real, state text, time integer)''')
            # self.cur.execute('''CREATE TABLE cell_data
            #             (f_cell_id text, amphour real, current real, temp real,
            #             voltage real, watthour real, stage text, time integer)''')
        except Exception as e:
            print(e)
        # try:
        #     self.cur.execute('''CREATE TABLE cell_capacity
        #                     msg_type real, ohm real, cap_ah real, cap_wh real, cap_wh real, stage text, time integer)''')
        #     # self.cur.execute('''CREATE TABLE cell_capacity
        #     #             (f_cell_id text, ohm real, cap_ah real, cap_wh real, cap_wh real, stage text, time integer)''')
        # except:
        #     pass

    def record_db(self, value_list):
        if len(value_list) == 0:
            return

        if value_list[0] in [ '0', '1', '2', '5', '6', '7' ]:
            try:
                value_list.append(str(int(time.time())))
                print('Recording to db: ', value_list)
                self.cur.execute("INSERT INTO cell_data VALUES (?,?,?,?,?,?,?,?)", value_list)
            except Exception as e:
                self.disconnect()
                exc_type, exc_value, exc_traceback = sys.exc_info()
                print(traceback.format_exception(exc_type, exc_value, exc_traceback))
        #else:
        #    print("#ERROR# Unexpected value_list length: {}".format(len(value_list)))

        # Save (commit) the changes
        self.conn.commit()

    def process_cycle_data(self, line):
        # Ignore remaining menu lines
        if line[0] == '>':
            return

        value_list = line.split(',')

        self.record_db(value_list)

def main():
    try:
        c = Cycler()

        # Create tables if not don't exist, delete the .db file to create with new fields but you lose te data
        c.create_tables()

        c.start()
    except KeyboardInterrupt:
        c.disconnect()

        print()
        print("Shutdown completed")


if __name__ == "__main__":
        main()
