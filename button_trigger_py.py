#!/usr/bin/env python3

"""
Hardware: NAVQ plus and RDDRON-FMUK66
Connection: NAVQ UART3 (/dev/ttymxc2)

The aim of this program is to evaluate and process mavlink data sets in order to enable the saving of localization data and photos in 2 different cases. 
In the context of the Hovergame 3 

The NAVQ+ and FMU are connected via UART. There are two buttons on the remote controle. Button associated with channel 9 should trigger an event. 
Therefore the output of the RC channel 9 is forwareded to the FMU PWM Pin5 in the remote control settings of the FMU. 
There are MAVLink Messges for the RC-raw values and the PWM-raw output. Bothe could be analysed for triggering the event. 
The trigger should only be set if the drone is armed. The Raw Servo output would only change value in armed state but the 
rc channel outputs the incoming data directly. therefore the easy way is using the servo channel. 

In case the Drone flys outonomous, there is a possiblity to check either for the flightplan status to trigger an event or set a 
pwm signal as part of a flightplan. Thus this is not implemented yet.

If the Event is triggered, the last received Location is saved and a photo is taken via the Google Coral Cam. Every picture is saved with a timecode 
which is the same as in the logfile. 

Author Joscha Siewert 
"""

# import module
from pymavlink import mavutil
from datetime import datetime
import cv2
import os
import pwd
import csv


def get_username():
    return pwd.getpwuid(os.getuid())[0]


user = get_username()
basepath = f'/home/{user}/hovergames/images/'
# check if basepath is exisiting else create it
if not os.path.exists(basepath):
    os.makedirs(basepath)

fields = ['Timestamp', 'Photoname', 'time_usec', 'fix_type', 'lat',
          'lon', 'alt', 'eph', 'epv', 'vel', 'cog', 'satellites_visible']
# check if  photolog exists:
logfilename = f'{basepath}photolog.csv'
if not os.path.isfile(logfilename):
    with open(logfilename, 'w') as csvfile:
        filewriter = csv.writer(csvfile)
        filewriter.writerow(fields)


# open Video Capture device (google Coral Camera)
cap = cv2.VideoCapture(
    'v4l2src device=/dev/video3 ! video/x-raw,framerate=30/1,width=640,height=480 ! appsink', cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise Exception("Could not open video device")


# get starting Time of Script
current_datetime = datetime.now()
str_current_datetime = str(current_datetime)
print(f"Hello there: {str_current_datetime}")

button_pressed = False		# debounce input
# dict entry element which is read
trigger_button = "servo5_raw"  # "chan9_raw" #"servo5_raw"
time_key = "time_usec"  # "time_boot_ms" #"time_usec"
# MAVLInk Message type to be analysed
trigger_msg_type = "SERVO_OUTPUT_RAW"  # 'RC_CHANNELS'#'SERVO_OUTPUT_RAW':
# dummy heartbeat dict
last_heartbet = {'mavpackettype': 'HEARTBEAT', 'type': 2, 'autopilot': 12,
                 'base_mode': 81, 'custom_mode': 131072, 'system_status': 3, 'mavlink_version': 3}

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection("/dev/ttymxc2", baud=921600)
# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages
while True:
    msg = the_connection.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'HEARTBEAT':
        new_heartbeat = msg.to_dict()
        # check if armed state has changed
        if new_heartbeat["system_status"] != last_heartbet["system_status"]:
            if new_heartbeat["system_status"] == 4:
                print("armed")
            else:
                print("disarmed")
        last_heartbet = new_heartbeat

# receive trigger Message Mavlink
    if msg.get_type() == trigger_msg_type:
        msg_dict = msg.to_dict()
        # check if trigger condition is reached for the first time
        if msg_dict[trigger_button] > 1900 and not button_pressed:
            time1 = msg_dict[time_key]  # get mavlink timestamp from Mavlink
            print(f"{time1}: button pressed!! \nat: {last_gps_raw_int}")
            button_pressed = True
        # get timestamp for filename:
            current_datetime = datetime.now()
            str_current_datetime = str(current_datetime)
            file_name = str_current_datetime+".txt"
        # take picture:
            result, image = cap.read()
            if result:
                path = f"/home/user/hovergames/images/{str_current_datetime}.png"
                status = cv2.imwrite(path, image)
                print("Image written to file-system : ", status)
                elements = [str_current_datetime, f'{str_current_datetime}.png', last_gps_raw_int['time_usec'], last_gps_raw_int['fix_type'], last_gps_raw_int['lat'], last_gps_raw_int['lon'],
                            last_gps_raw_int['alt'], last_gps_raw_int['eph'], last_gps_raw_int['epv'], last_gps_raw_int['vel'], last_gps_raw_int['cog'], last_gps_raw_int['satellites_visible']]
                with open(logfilename, 'a+', newline='') as csvfile:
                    filewriter = csv.writer(csvfile)
                    filewriter.writerow(elements)
            else:
                print("No image detected. Please! try again")
                elements = [str_current_datetime, f'NO_IMAGE', last_gps_raw_int['time_usec'], last_gps_raw_int['fix_type'], last_gps_raw_int['lat'], last_gps_raw_int['lon'],
                            last_gps_raw_int['alt'], last_gps_raw_int['eph'], last_gps_raw_int['epv'], last_gps_raw_int['vel'], last_gps_raw_int['cog'], last_gps_raw_int['satellites_visible']]
                with open(logfilename, 'a+', newline='') as csvfile:
                    filewriter = csv.writer(csvfile)
                    filewriter.writerow(elements)
        if msg_dict[trigger_button] <= 1900 and button_pressed:
            time2 = msg_dict[time_key]
            button_pressed = False
            diff = float(time2)-float(time1)
            print(f"{time2}: button released after: {diff}")

    if msg.get_type() == 'GPS_RAW_INT':
        last_gps_raw_int = msg.to_dict()
  