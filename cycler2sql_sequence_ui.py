import argparse
import sqlite3
import os
import sys
import time
import traceback
import configparser
from colorama import init

from mod_serial import usbSerial

usb_dev = 'COM4'
test_line = "d1 i1067 v2500"

charge_cmd = 'c1 i6400 e1080'
wait_sec = 240
get_status_cmd = 's'
wait_cmd = 'c1 i1600 o400 e3600'
discharge_cmd = 'd1 i1600 v2500'
db_name = "cell_database"


class StageCompleted(BaseException):
    pass


class Cycler():
    def __init__(self):
        self.device = None

        # TODO total_cyclers hard coded
        self.total_cyclers = 2

    def init(self, device=None):
        print("COM port used:", device)
        init()
        while True:
            # Connect
            # ________
            self.device = usbSerial(device)
            try:
                self.device.connect()
            except Exception as e:
                if not self.device:
                    print("Closing serial")
                    self.device.close()
                print("Serial connect failed: {}".format(e))
                sys.exit()
            else:
                print("Connected to serial")
                break

        while True:
            # Initialize
            # ___________
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
    # ________________________________________
    def sync(self):
        if not self.device.serial:
            raise Exception("Device not connected, device is: {}".format(self.device))

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
            if '> Select Mode:' in line:
                print('Initialized')
                return True

        return False

    # Run
    # _______________
    def start_cycle(self, args):
        try:
            # Create tables if not don't exist, delete the .db file to create with new fields but you lose te data

            self.create_tables(args.database)

            # Initialize serial device
            self.init(args.device)
            print('Cycler cycle mode running')

            self.device.sendline(args.testline + "\n")

            while True:
                if self.device is None:
                    print("Re-initializing lost comms")
                    self.init(args.device)

                #
                # check for serial data
                # _______________
                for line in self.device.readlines():
                    if line:
                        self.process_cycle_data(line)

        except KeyboardInterrupt:
            self.disconnect()

            print()
            print("Shutdown completed")

    def create_tables(self, db_name):

        if os.path.exists("{}.db".format(db_name)) is not True:
            print("Creating new DB file: {}.db".format(db_name))

        self.conn = sqlite3.connect(db_name + '.db')
        self.cur = self.conn.cursor()


        try:
            self.cur.execute('''CREATE TABLE cell_data
                        (msg_type integer, millivolt integer, milliamp integer, millamphour real, 
                         millwatthour real, temp real, state text, time integer)''')
        except Exception as e:
            print(e)

    def record_db(self, value_list):
        if len(value_list) == 0:
            return

        if value_list[0] in ['0', '1', '2', '5', '6', '7', '9']:
            try:
                value_list.append(str(int(time.time())))
                #print('Recording to db: ', value_list)
                self.cur.execute("INSERT INTO cell_data VALUES (?,?,?,?,?,?,?,?)", value_list)
            except Exception:
                self.disconnect()
                exc_type, exc_value, exc_traceback = sys.exc_info()
                print(traceback.format_exception(exc_type, exc_value, exc_traceback))

        # Save (commit) the changes
        self.conn.commit()

    def process_cycle_data(self, line):
        # Ignore empty lines
        if len(line) == 0:
            return [-1]
        # Ignore remaining menu lines
        if line[0] == '>':
            return [-1]

        value_list = line.split(',')

        self.record_db(value_list)

        return value_list

    def process_sequence_data(self, line):
        pass

    def start_profile(self, args):
        config = configparser.ConfigParser()

        # Profile checks
        if not os.path.isfile('profiles.ini'):
            print("profile.ini does not exist, please create one with profiles set")
            sys.exit(1)

        config.read('profiles.ini')
        if args.profile_name not in config.sections():
            print(config.sections())
            print("Profile '{}' does not exist in profiles.ini".format(args.profile_name))
            sys.exit()

        # Validate commands in profile
        for stage, command in config[args.profile_name].items():
            print("Stage: {} Command: {}".format(stage, command))
            if command.split(" ")[0] not in ['c1', 'c2', 'd1', 'd2', 'r1', 'r2', 'w1', 'w2']:
                print("'{}' is not a support profile command, only c[12], d[12] or r[12] can be used".format(
                    command.split(" ")[0]
                ))
                sys.exit(1)

        # Run commands in profile
        try:
            self.create_tables(args.database)

            # Initialize serial device
            self.init(args.device)
            print('Cycler cycle mode running')

            for stage, command in config[args.profile_name].items():
                print("Stage: {} Command: {}".format(stage, command))

                self.device.sendline(command.strip() + "\n")

                try:
                    while self.device.is_connected():
                        if self.device.serial is None:
                            print("Re-initializing lost comms")
                            self.init(args.device)

                        #
                        # check for serial data
                        # _______________
                        for line in self.device.readlines():
                            if line:
                                values = self.process_cycle_data(line)
                                if values[0] in ['0']:
                                    print("\033[1AStatus - Stage: {} V1: {}mV, I1: {}mA".format(stage, values[1], values[2]),' ','')
                                #print("Stage: {}, Values: {}".format(stage, values))
                                if values[0] in ['1', '6', '2', '7']:
                                    raise StageCompleted(stage)  # to next stage
                        # Hard coded 0.5 second pause between stages
                        time.sleep(0.5)
                except StageCompleted as e:
                    print("Stage: '{}' Completed -----------------".format(e))
        except KeyboardInterrupt:
            self.disconnect()

            print()
            print("Shutdown completed")
            sys.exit()

        print("Profile command run completed for profile: {}".format(args.profile_name))
        sys.exit()

    def start_sequence(self, args):
        # Initialize serial device
        self.init(args.device)
        print('Cycler sequence mode running')

        try:
            self.create_tables(args.database)
            for cycle_index in range(0, args.repeat):
                # Create tables if not don't exist, delete the .db file to create with new fields but you lose te data

                if self.device is None:
                    print("Re-initializing lost comms")
                    self.init(args.device)

                # Send Charge mode
                self.device.sendline(charge_cmd + "\n")

                # Process readouts until mode 2 received
                waiting = True
                while waiting:
                    for line in self.device.readlines():
                        if line and self.process_cycle_data(line)[0] == "2":
                            waiting = False

                # Wait for wait_sec, poll for data
                last_time = int(time.time()) - 1
                wait_count = 0
                while wait_count < wait_sec:
                    # Sleep less because of processing delay
                    if int(time.time()) >= last_time + 1:
                        last_time = int(time.time())
                        wait_count = wait_count + 1
                        self.device.sendline(get_status_cmd + "\n")
                        for line in self.device.readlines():
                            self.process_cycle_data(line)

                # Send next charge cmd
                self.device.sendline(wait_cmd + "\n")

                # Process readouts until mode 2 received
                waiting = True
                while waiting:
                    for line in self.device.readlines():
                        if line and self.process_cycle_data(line)[0] == "2":
                            waiting = False

                # Send discharge cmd
                # self.device.sendline(discharge_cmd + "\n")

                # Process readouts until mode 1 received
                # waiting = True
                # while waiting:
                #    for line in self.device.readlines():
                #        if line and self.process_cycle_data(line)[0] == "1":
                #            waiting = False

        except KeyboardInterrupt:
            self.disconnect()
            print()
            print("Shutdown completed")


def setup_cmd_line():
    parser = argparse.ArgumentParser(description='Cycler2SQL - Cell Tools v0.0.1', add_help=False)

    subparser = parser.add_subparsers()

    # Normal Cycle
    single = subparser.add_parser('single', help='Normal cycler system')
    single.add_argument('-d', '--device', default=usb_dev, required=False, help='Com port of serial device')
    single.add_argument('-t', '--testline', default=test_line, required=False, help='Com port of serial device')
    single.add_argument('-n', '--database', default=db_name, required=False, help='Database filename')
    single.set_defaults(func=Cycler().start_cycle)

    # Sequence Cycle
    sequence = subparser.add_parser('sequence', help='Runs a sequence of charges and discharges')
    # sequence.add_argument('sequence', help='Specify the service for which logs should be fetched')
    sequence.add_argument('-d', '--device', default=usb_dev, required=False, help='Com port of serial device')
    sequence.add_argument('-r', '--repeat', default=1, required=False, help='Number of charge/discharge cycles to run')
    sequence.add_argument('-n', '--database', default=db_name, required=False, help='Database filename')
    sequence.add_argument('-w', '--wait', default=wait_sec, required=False,
                          help='Number of seconds to sleep between charge and discharge cycle')

    sequence.set_defaults(func=Cycler().start_sequence)

    # Normal Cycle
    profile = subparser.add_parser('profile', help='Profile sequence command set')
    profile.add_argument('-d', '--device', default=usb_dev, required=False, help='Com port of serial device')
    profile.add_argument('-n', '--database', default=db_name, required=False, help='Database filename')
    profile.add_argument('-p', '--profile', dest='profile_name', default=None, required=True,
                         help='Profile name defined in profiles.ini')

    profile.set_defaults(func=Cycler().start_profile)

    if len(sys.argv[1:]) == 0:
        parser.print_help()
        parser.exit()
    return parser.parse_args()


if __name__ == "__main__":
    args = setup_cmd_line()
    args.func(args)
    print("Stopping..")
