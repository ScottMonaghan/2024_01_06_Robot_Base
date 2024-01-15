import board
from digitalio import DigitalInOut, Direction, Pull
from time import monotonic, sleep
from analogio import AnalogIn
import adafruit_bno055
import busio
import adafruit_minimqtt.adafruit_minimqtt as MQTT

i2c = busio.I2C(board.GP13,board.GP12)
bno055 = adafruit_bno055.BNO055_I2C(i2c)

import wifi
import socketpool
import ipaddress
import os
import ssl
import json

# import adafruit_requests

throttle_pot = AnalogIn(board.A0)
MAX_RAW_THROTTLE = 64000
MIN_RAW_THROTTLE = 500
RATE_IN_HERTZ = 60
MAX_RPM = 100
MIN_RPM = 20

btn_up= DigitalInOut(board.GP3)
btn_down = DigitalInOut(board.GP2)

btn_up.direction = Direction.INPUT
btn_down.direction = Direction.INPUT

btn_up.pull = Pull.DOWN
btn_down.pull = Pull.DOWN

# #  set static IP address
client_ipv4 =  ipaddress.IPv4Address(os.getenv('ROBOT_BASE_CONTROLLER_IPV4'))

netmask =  ipaddress.IPv4Address(os.getenv('CIRCUITPY_NETMASK'))
gateway =  ipaddress.IPv4Address(os.getenv('CIRCUITPY_GATEWAY'))
wifi.radio.set_ipv4_address(ipv4=client_ipv4,netmask=netmask,gateway=gateway)
#  connect to your SSID
print(os.getenv('CIRCUITPY_WIFI_SSID'))
wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))
ssl_context = ssl.create_default_context()

print("Connected to WiFi")
pool = socketpool.SocketPool(wifi.radio)
    
HOST = os.getenv('ROBOT_BASE_IPV4')
PORT = os.getenv('ROBOT_BASE_PORT')
TIMEOUT = 5
INTERVAL = 0.05
MAXBUF = 256

print("Self IP", wifi.radio.ipv4_address)
server_ipv4 = ipaddress.ip_address(pool.getaddrinfo(HOST, PORT)[0][4][0])
print("Server ping", server_ipv4, wifi.radio.ping(server_ipv4), "ms")

buf = bytearray(MAXBUF)
print("Create UDP Client socket")
s = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
s.settimeout(TIMEOUT)
imu_calibrated = False
while True:
    loop_start = monotonic()
    target_loop_duration = 1.0/RATE_IN_HERTZ

    if not imu_calibrated:
        imu_calibrated = bno055.calibration_status[3] == 3

    #calculate target rpm
    raw_throttle = throttle_pot.value
    target_rpm = 0 
    if raw_throttle > MAX_RAW_THROTTLE: target_rpm = MAX_RPM
    elif raw_throttle < MIN_RAW_THROTTLE: target_rpm = 0
    else:
        normalized_throttle = (raw_throttle - MIN_RAW_THROTTLE) / (MAX_RAW_THROTTLE - MIN_RAW_THROTTLE)
        target_rpm = int((MAX_RPM - MIN_RPM) * normalized_throttle + MIN_RPM)
    command = "STOP"
    if btn_up.value: command = "FORWARD"
    elif btn_down.value: command = "BACK"

    
    publish_args = {
        "heading":-1.0 if not imu_calibrated else int(bno055.euler[0]),
        "target_rpm":target_rpm,
        "command":command,
    }

    payload = json.dumps(publish_args)
    size = s.sendto(payload, (HOST, PORT))
    #print(payload)

    loop_duration = monotonic() - loop_start
    if loop_duration < target_loop_duration:
        sleep(target_loop_duration - loop_duration)
    else:
        print (f'Loop overtime: {loop_duration}')
