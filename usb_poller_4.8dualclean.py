import serial
import logging
import time
import requests
import os
import sys
import traceback
from urllib.parse import urlencode
from influxdb import InfluxDBClient # we could go without this using arduino UDP method
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


## ARE WE DEBUGGING
testing = False
windows = True
# 0 cell1 status
# 1 cell1 summary/result
# 5 cell2 status
# 6 cell2 summary/result


col_green = "\033[32m"
col_red = "\033[31m"
col_yellow = "\033[33m"
col_blue = "\033[36m"
col_norm = "\033[0m"


#
# Grafana settings
#
username='yourusername'
password='yourpassword'
database='your_db'
measurement='cellcharger' if not testing else "cellcharger_test"

#---------------------------------
# Hold off time before sending new data
if testing:
  senddata_ttl = 20
  read_delay = 0.5
  fr = open("cycler_orig.csv","r")
  client = InfluxDBClient(host='127.0.0.1', username=username, password=password, database=database, port=443, ssl=True, timeout=1)
else:
  senddata_ttl = 20
  csv = open('cycler_orig.csv','a')
  client = InfluxDBClient(host='127.0.0.1', username=username, password=password, database=database, port=443, ssl=True, timeout=1)


#
# Thingspeak settings
#

# API url
api_url = 'https://api.thingspeak.com/update'

# API key for each data type
api_key = {}
api_key[0] = 'APIKEYPERIODICST' # key 0 for type 0
api_key[5] = 'APIKEYPERIODICST' # key 0 for type 5
api_key[1] = 'APIKEYENDOFCYCLE' # key 1 for type 1
api_key[6] = 'APIKEYENDOFCYCLE' # key 1 for type 6

#---------------------------------
# Serial configuration
usb_dev = 'COMxx'
usb_speed = 115200

last_sent = {}

csv_path = '.'

states = {  '1': 'discharge',
            '2': 'batt_disconnected',
            '3': 'charge',
            '4': 'batt_disconnect',
            '5': 'wait',
            '6': 'measure_ir',
            '7': 'measure_ir',
            '8': 'parking'
         }

finished_states = {  2, 1, 3 }

def format_payload(data, method):
    try:
        if method == 'http':
            if data[0] == "0":
                #['0', 'mV'     , 'mA'   , 'mAH'  , 'mWH'  , '5', '55']
                #['0', '4142.28', '-0.19', '-2.51', '28.87', '5', '55']
                return {
                    "field1": data[1],
                    "field2": "{}".format(data[2]),
                    "field3": "{}".format(data[3]),
                    "field4": "{}".format(data[5])}
            elif data[0] == "1":
                #1,mV     ,mOhms     ,mAH  ,mWH  ,C    ,6,61
                #1,4143.69,-169965.33,-1.43,-6.02,23.76,6,61
                return {
                    "field1": "{}".format(data[2]),
                    "field2": "{}".format(data[3]),
                    "field3": "{}".format(data[4]),
                    "field4": "{}".format(data[5])}
            elif data[0] == "6":
                #1,mV     ,mOhms     ,mAH  ,mWH  ,C    ,6,61
                #1,4143.69,-169965.33,-1.43,-6.02,23.76,6,61
                return {
                    "field5": "{}".format(data[2]),
                    "field6": "{}".format(data[3]),
                    "field7": "{}".format(data[4]),
                    "field8": "{}".format(data[5])}
        elif method == 'file':
            return [ val for val in data if val is not "" and val[-1].isdigit() ]
        elif method == 'grafana':
            if data[0] == "4":
                # Buffer pack data
                payload = [{ "measurement": measurement,
                    "time": int(time.time()*1000000000),
                    "fields": {
                        "voltage_buffer": float(data[1])
                        }
                    }]
                return payload
            if data[0] == "0" or data[0] == "5":
                cell_id = (int(data[0]) % 4) + 1
                # Store what stage we're in
                stage_id = data[6][0]
                # Cell data type 1
                payload = [{ "measurement": measurement,
                    "time": int(time.time()*1000000000),
                    "tags": { "stage": states[stage_id],
                              "cell_id": cell_id },
                    "fields": {
                        "voltage": float(data[1]),
                        "current": float(data[2]),
                        "amphour": float(data[3]),
                        "watthour": float(data[4]),
                        "temp": float(data[5])
                        }
                    }]

                return payload
            elif data[0] == "1" or data[0] == "6":
                cell_id = (int(data[0]) % 4) + 0
                # Store what stage we're in
                stage_id = data[6][0]
                # Cell data type 2                
                payload = [{ "measurement": measurement,
                    "time": int(time.time()*1000000000),
                    "tags": { "stage": states[stage_id],
                              "cell_id": cell_id },
                    "fields": {
                        "voltage": float(data[1]),
                        "ohms": float(data[2]),
                        "amphour": float(data[3]),
                        "watthour": float(data[4]),
                        "temp": float(data[5])
                        }
                    }]
                return payload
            return {}
    except:
        print(data)
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        logging.error("Payload Error: '{}'".format(traceback.print_exc(file=sys.stdout)))

        raise RuntimeError("Type '{}' sent but not configured in format_payload()".format(data[0]))
        
def format_payload_full(data, method):
    try:
        if method == 'http':
            if data[0] == "0":
        #Serial.print("0,");
        #Serial.print(vbat_1_1);
        #Serial.print(",");
        #Serial.print(ibat_1_1);
        #Serial.print(",");
        #Serial.print(mah1);
        #Serial.print(",");
        #Serial.print(mwh1);
        #Serial.print(",");
        #Serial.print(temp1);
        #Serial.print(",");
        #Serial.println(state1);
                #['0', '4142.28', 'mV', '-0.19', 'mA', '-2.51', 'mAH', '28.87', 'mWH', '5', '55']
                #['0', '4142.28', '-0.19', '-2.51', '28.87', '5', '55']
                return {
                    "field1": data[1],
                    "field2": "{}".format(data[2]),
                    "field3": "{}".format(data[3]),
                    "field4": "{}".format(data[5]),
                    "field5": "{}".format(data[8]),
                    "field6": "{}".format(data[9]),
                    "field7": "{}".format(data[10]),
                    "field8": "{}".format(data[12])}
            elif data[0] == "1" and len(data) > 8:
        # Serial.print("1,");
        # Serial.print(vbat_1_1);
        # Serial.print(",");
        # Serial.print(ir1);
        # Serial.print(",");
        # Serial.print(mah1);
        # Serial.print(",");
        # Serial.print(mwh1);
        # Serial.print(",");
        # Serial.print(temp1);
        # Serial.print(",");
        # Serial.println(state1);
                #['0', '4142.28', 'mV', '-0.19', 'mA', '-2.51', 'mAH', '28.87', 'mWH', '5', '55']
                #['0', '4142.28', '-0.19', '-2.51', '28.87', '5', '55']
                return {
                    "field1": data[2],
                    "field2": "{}".format(data[3]),
                    "field3": "{}".format(data[4]),
                    "field4": "{}".format(data[5]),
                    "field5": "{}".format(data[9]),
                    "field6": "{}".format(data[10]),
                    "field7": "{}".format(data[11]),
                    "field8": "{}".format(data[12])}
            elif data[0] == "1":
        # Serial.print("1,");
        # Serial.print(vbat_1_1);
        # Serial.print(",");
        # Serial.print(ir1);
        # Serial.print(",");
        # Serial.print(mah1);
        # Serial.print(",");
        # Serial.print(mwh1);
        # Serial.print(",");
        # Serial.print(temp1);
        # Serial.print(",");
        # Serial.println(state1);
                #['0', '4142.28', 'mV', '-0.19', 'mA', '-2.51', 'mAH', '28.87', 'mWH', '5', '55']
                #['0', '4142.28', '-0.19', '-2.51', '28.87', '5', '55']
                return {
                    "field1": data[2],
                    "field2": "{}".format(data[3]),
                    "field3": "{}".format(data[4]),
                    "field4": "{}".format(data[5])}
            elif data[0] == "6":
        # Serial.print("1,");
        # Serial.print(vbat_1_1);
        # Serial.print(",");
        # Serial.print(ir1);
        # Serial.print(",");
        # Serial.print(mah1);
        # Serial.print(",");
        # Serial.print(mwh1);
        # Serial.print(",");
        # Serial.print(temp1);
        # Serial.print(",");
        # Serial.println(state1);
                #['0', '4142.28', 'mV', '-0.19', 'mA', '-2.51', 'mAH', '28.87', 'mWH', '5', '55']
                #['0', '4142.28', '-0.19', '-2.51', '28.87', '5', '55']
                return {
                    "field5": data[2],
                    "field6": "{}".format(data[3]),
                    "field7": "{}".format(data[4]),
                    "field8": "{}".format(data[5])}
        elif method == 'file':
            return [ val for val in data if val is not "" and val[-1].isdigit() ]
        elif method == 'grafana':
            if data[0] == "4":
                # Buffer pack data
                payload = [{ "measurement": measurement,
                    "time": int(time.time()*1000000000),
                    "tags": { "buffer": 'buffer' },
                    "fields": {
                        "voltage": float(data[1])
                        }
                    }]
            if data[0] == "0" or data[0] == "5":
                cell_id = (data[0] % 4) + 1
                # Store what stage we're in
                stage_id = data[6]
                # Cell data type 1
                payload = [{ "measurement": measurement,
                    "time": int(time.time()*1000000000),
                    "tags": { "stage": states[stage_id] },
                    "fields": {
                        "voltage": float(data[1]),
                        "current": float(data[2]),
                        "amphour": float(data[3]),
                        "watthour": float(data[4]),
                        "cell_id": cell_id
                        }
                    }]

                return payload
            elif data[0] == "1" or data[0] == "6":
                cell_id = (data[0] % 4) + 1
                # Store what stage we're in
                stage_id = data[6]
                # Cell data type 2                
                payload = [{ "measurement": measurement,
                    "time": int(time.time()*1000000000),
                    "tags": { "stage": states[stage_id] },
                    "fields": {
                        "voltage": float(data[1]),
                        "ohms": float(data[2]),
                        "amphour": float(data[3]),
                        "watthour": float(data[4]),
                        "cell_id": cell_id
                        }
                    }]
                return payload
            return {}
    except:
        print(data)
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        logging.error("Payload Error: '{}'".format(traceback.print_exc(file=sys.stdout)))

        raise RuntimeError("Type '{}' sent but not configured in format_payload()".format(data[0]))


def ready_to_send(dtype):
    # Init for first time seen
    if dtype not in last_sent:
        last_sent[dtype] = 0
    # If we've past the expiry time, ok to send new data
    if time.time() - last_sent[dtype] >= senddata_ttl:
#        print("updated last_sent!!!!!!!!!!!!!")
#        last_sent[dtype] = time.time()
        return True

    # otherwise drop the data
    return False


def read_serial():
    # Msg type 1 (Discharged): 1,3650.24,mV,-1524.76,mA,361.03,mAH,,3,750
    # (message type normal status,cell voltage,cell voltage units, cell current, cell current units, cell mah, cell mah units, state machine state, settle time)
    if not testing:
        # USB data
        with serial.Serial(usb_dev, usb_speed, timeout = 30) as ser:
            while True:
                readline = ser.readline().decode('ascii').rstrip("\r\n")
                if not testing:
                  csv.write(str(readline)+'\n')
                yield readline                
    else:
        line = fr.readline().rstrip("\r\n")
        yield line
        time.sleep(read_delay)



def send_grafana(data_type, payload):
    return client.write_points(payload)

def send_thingspeak(data_type, payload):
    if data_type not in api_key:
        raise RuntimeError("api_key not configured for data type: {}".format(data_type))

    headers = {'Content-type': 'application/x-www-form-urlencoded',
               'X-THINGSPEAKAPIKEY': api_key[data_type]}

    if not testing:
      http = requests.post(api_url, data=payload, headers=headers)
    else:
      #logging.debug("Testing, won't post to thingspeak")
      http = requests.post(api_url, data=payload, headers=headers)
      #http = {}

    return http

def write_to_file(stage, cellno, data, datestamp, timestamp):
    if testing == True:
        return
    # CSV data format
    linedate = time.strftime("%d%m%Y")
    linetime = time.strftime("%H%M%S")
    dataformat = "{},{},{}\n".format(','.join(data[1:]), linedate, linetime)

    # Filename format
    filename = csv_path+'/cell{cellno}_{stage}_{datestamp}_{timestamp}.csv'.format(cellno=cellno,
                                                                                   stage=stage,
                                                                                   datestamp=datestamp,
                                                                                   timestamp=timestamp)

    fw = open(filename, 'a+')
    fw.write(dataformat)


# main
#
#logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(levelname)-8s] %(message)s')
#logging.debug("Starting up")
root = logging.getLogger()
root.setLevel(logging.DEBUG)
ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
root.addHandler(ch)
logging.debug("Starting up")

fw = None
os.system('color')

stage = 'charge'
state_now = 0
full_list1 = []
cell1_cycle = []
cell1_pending = 0
cell2_cycle = []
cell2_pending = 0
cellno = 1 # TODO hardcoded for now
datestamp = time.strftime("%d%m%Y")
timestamp = time.strftime("%H%M%S")
write_to_file(stage, cellno, ['','Cell Voltage mV', 'Cell Current mA', 'Cell Capacity mAH', 'Cell Capacity mWH', 'Temperature (C)', 'State', 'Runtime', 'Date', 'Time'], datestamp, timestamp)

while True:
    logging.info("Looping main")
    try:
        for line in read_serial():
            if not line:
                # empty line skipped
                continue
            value_list = line.split(',')
                
            state_last = state_now
            #logging.debug("state:")
            #logging.debug(str(state_now))
            state_now = int(value_list[0])

            # Cell Status
            if state_now == 0:
                full_list1 = value_list
                logging.debug("state = 0")
            elif state_now == 5:
                full_list1 = full_list1 + value_list
                logging.debug("state = 0+5")

            # Cell cycle summary
            if state_now == 1:
                cell1_cycle = value_list
                cell1_pending = 1
                logging.debug("state = 1, cell1_pending = 1")
            if state_now == 6:
                cell2_cycle = value_list
                cell2_pending = 1
                logging.debug("state = 6, cell2_pending = 1")
                
            """
            # Send to Grafana
            payload = format_payload(value_list, 'grafana')

            if payload:
                send_grafana(state_now, payload)
                #logging.debug("{}Sending grafana: {}{}".format(col_green, payload, col_norm))
            else:
                logging.debug("payload empty for grafana, skipping")
            """
            # Check we're past the 15sec
            if state_now in api_key:
                #if state_now == 1 and len(full_list2) > 8:
                if state_now == 1 and ready_to_send(1) and ready_to_send(6):
                    logging.debug("{}received cell 1 cycle, sending to thingspeak{}".format(col_yellow, col_norm))
                    # Format the payload base on the type of data received
                    payload = format_payload(cell1_cycle, 'http')
                    cell1_cycle = []
                    cell1_pending = 0
                    logging.debug("state = 1, ready to send, cell1_pending = 0")
                    logging.debug("{}Sending cell1_cycle to http: {}{}".format(col_green, payload, col_norm))
                    # Send all the thingspeak
                    send_thingspeak(state_now, payload)
                    last_sent[state_now] = time.time()
                if state_now == 6 and ready_to_send(1) and ready_to_send(6):
                    logging.debug("{}received cell 2 cycle, sending to thingspeak{}".format(col_yellow, col_norm))
                    # Format the payload base on the type of data received
                    payload = format_payload(cell2_cycle, 'http')
                    cell2_cycle = []
                    cell2_pending = 0
                    logging.debug("state = 6, ready to send, cell2_pending = 0")
                    logging.debug("{}Sending cell2_cycle to http: {}{}".format(col_green, payload, col_norm))
                    # Send all the thingspeak
                    send_thingspeak(state_now, payload)
                    last_sent[state_now] = time.time()
                #elif (state_now == 1 or state_now == 6):
                if (state_now == 0 or state_now == 5) and ready_to_send(0) and ready_to_send(5) and len(full_list1) > 8:
                    logging.debug("{}received full 0+5, sending to thingspeak{}".format(col_yellow, col_norm))
                    # Format the payload base on the type of data received
                    payload = format_payload_full(full_list1, 'http')
                    logging.debug("{}Sending full_list1 to http: {}{}".format(col_green, payload, col_norm))
                    # Send all the thingspeakl
                    send_thingspeak(state_now, payload)
                    last_sent[state_now] = time.time()
                    if not testing:
                      csv.flush()
                #logging.debug("Cell1_pending = {}, Cell2_pending = {}".format(cell1_pending, cell2_pending))
        
                if cell1_pending == 1:
                  if ready_to_send(1) and ready_to_send(6):
                      logging.debug("{}pending cell 1 cycle, sending to thingspeak{}".format(col_yellow, col_norm))
                      # Format the payload base on the type of data received
                      payload = format_payload(cell1_cycle, 'http')
                      cell1_cycle = []
                      cell1_pending = 0
                      logging.debug("ready to send (outside loop), cell1_pending = 0")
                      logging.debug("{}Sending cell1_cycle to http: {}{}".format(col_green, payload, col_norm))
                      # Send all the thingspeak
                      send_thingspeak(1, payload)
                      last_sent[1] = time.time()
                if cell2_pending == 1:
                  if ready_to_send(1) and ready_to_send(6):
                      logging.debug("{}pending cell 2 cycle, sending to thingspeak{}".format(col_yellow, col_norm))
                      # Format the payload base on the type of data received
                      payload = format_payload(cell2_cycle, 'http')
                      cell2_cycle = []
                      cell2_pending = 0
                      logging.debug("ready to send (outside loop), cell2_pending = 0")
                      logging.debug("{}Sending cell2_cycle to http: {}{}".format(col_green, payload, col_norm))
                      # Send all the thingspeak
                      send_thingspeak(6, payload)
                      last_sent[6] = time.time()
                #else:
                    #logging.debug("{}Nothing Happened{}".format(col_blue, col_norm))
            else:                
                logging.debug("{}Dropping data: {}{}".format(col_red, value_list, col_norm))

            # Save to csv
            if state_last in finished_states: #!= state_now:
                write_to_file(stage, cellno, format_payload(value_list, 'file'), datestamp, timestamp)
                if state_now == 0:
                    state = 'charge'
                elif state_now == 1:
                    state = 'disch'
                datestamp = time.strftime("%d%m%Y")
                timestamp = time.strftime("%H%M%S")
                write_to_file(stage, cellno, ['','Cell Voltage mV', 'Cell Current mA', 'Cell Capacity mAH', 'Cell Capacity mWH', 'Temperature (C)', 'State', 'Runtime', 'Date', 'Time'], datestamp, timestamp)
            else:
                write_to_file(stage, cellno, format_payload(value_list, 'file'), datestamp, timestamp)
                
    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        logging.error("Restarting. Error: '{}'".format(traceback.print_exc(limit=2, file=sys.stdout)))
        #logging.error("Restarting. Error: '{}' {} on line {}".format(e, exc_type, exc_tb.tb_lineno))
        time.sleep(1)

