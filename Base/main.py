import board
import pwmio
from time import (
    sleep,
    monotonic,
    monotonic_ns, 
    )
from digitalio import(
    DigitalInOut,
    Direction,
    Pull,
)
import countio
import asyncio
import wifi
import socketpool
import ipaddress
import os
import ssl
import json
import adafruit_bno055
from select import select
import busio
from errno import ETIMEDOUT
from random import (randrange, random)

DEMO = False

RATE = 10 #target rate in hz
SCALE_TIME_NS = 1e-9
SCALE_TIME_MS = 1e-3
SCALE_PWM = 1e-1
SCALE_ERROR = 10
SCALE_HEADING_CORRECTION = 0.5
PULSES_PER_ROTATION = 45
MIN_PULSES_FOR_CALCULATION = 3
RPM_MONITOR_ZERO_TIMEOUT = 0.5
MIN_NONZERO_THROTTLE = 50
DIFFERENTIAL_MIN_TICKS = 2
PID_KP = 1.0
PID_KI = 2e-5
PID_KD = 0 

#modified from https://www.kevsrobots.com/resources/how_it_works/pid-controllers.html
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.last_time = 0

    def update(self, error, dt):
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else: 
            derivative = 0
        self.integral += error * dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

class PulseRecord:
    def __init__(self, pulse_count, timestamp):
        self.pulse_count = pulse_count
        self.timestamp = timestamp

class ZS_X11H_BLDCWheel:
    def __init__(
            self,
            pwm_out:pwmio.PWMOut,
            pulse_counter:countio.Counter,
            pulses_per_rotation:int,
            pid:PID,
            reverse_switch:DigitalInOut,
            reverse_value: bool
            ):
        self.pwm_out = pwm_out
        self.pulse_counter = pulse_counter
        self.pid = pid
        self.rmp_monitor_enabled = True
        self._current_rpm = 0.0
        self._target_rpm = 0
        self._last_pulse_record = PulseRecord(self.pulse_counter.count,monotonic_ns() * SCALE_TIME_NS)
        self._reverse_value = reverse_value
        self._reverse_switch = reverse_switch
        self._reverse_switch.value = not self._reverse_value
        self._lock_target_rpm = False

    @property
    def current_rpm(self):
        return self._current_rpm
    
    @current_rpm.setter
    def current_rpm(self, value):
        self._current_rpm = value

    @property
    def target_rpm(self):
        return self._target_rpm
    
    async def set_target_rpm(self, value):
        if (
            (value < 0 and self._reverse_switch.value != self._reverse_value)
            or (value > 0 and self._reverse_switch.value == self._reverse_value)
            ):
            # # we need to change direction
            self._target_rpm = 0
            self.throttle = 0
            await asyncio.sleep(0.05)
            self._reverse_switch.value = not self._reverse_switch.value
            #now reset target value
            self._target_rpm = value
        self._target_rpm = value

    @property
    def throttle(self):
        return self.pwm_out.duty_cycle
    
    @throttle.setter
    def throttle(self, value):
        if value < MIN_NONZERO_THROTTLE: 
            value = 0
        self.pwm_out.duty_cycle = clamp(abs(value),0,65535)
    
    async def rpm_monitor(self, min_ticks, timeout):
        while True:
            self.pulse_counter.reset()
            start_time = monotonic()
            while self.pulse_counter.count < min_ticks and monotonic()-start_time < timeout and self.rmp_monitor_enabled:
                await asyncio.sleep(0)
            if self.rmp_monitor_enabled:
                if monotonic()-start_time > timeout:
                    #print(monotonic()-start_time)
                    self.current_rpm = 0
                else:
                    pulses_per_second = self.pulse_counter.count / (monotonic() - start_time)
                    self.current_rpm = pulses_per_second / PULSES_PER_ROTATION * 60
            await asyncio.sleep(0)
 
async def differential_synced_rpm_monitor(left_wheel:ZS_X11H_BLDCWheel, right_wheel:ZS_X11H_BLDCWheel, min_ticks:int, timeout:float):
    while True:
        left_wheel.pulse_counter.reset()
        right_wheel.pulse_counter.reset()
        start_time = monotonic()
        while (
            (left_wheel.pulse_counter.count < min_ticks
            or right_wheel.pulse_counter.count < min_ticks)
            and monotonic()-start_time < timeout
            ):
            await asyncio.sleep(0)

        left_wheel.current_rpm = 0 if left_wheel.pulse_counter.count < min_ticks else calc_rpm(left_wheel,start_time)
        right_wheel.current_rpm = 0 if right_wheel.pulse_counter.count < min_ticks else calc_rpm(right_wheel,start_time)

        ##lets see what happens if we do pid here
        current_time = monotonic_ns() * SCALE_TIME_NS
        dt = current_time - left_wheel.pid.last_time
        left_wheel.pid.last_time = current_time
        right_wheel.pid.last_time = current_time

        # calculate error for PID
        left_error = (abs(left_wheel.target_rpm) - left_wheel.current_rpm) 
        right_error = (abs(right_wheel.target_rpm) - right_wheel.current_rpm)
 
        # #calc PID adjustent
        left_adjustment = left_wheel.pid.update(left_error, dt)
        right_adjustment = right_wheel.pid.update(right_error,dt)

        left_wheel.throttle = 0 if left_wheel.target_rpm == 0 else left_wheel.throttle + int(left_adjustment / SCALE_PWM)
        right_wheel.throttle = 0 if right_wheel.target_rpm == 0 else right_wheel.throttle + int(right_adjustment / SCALE_PWM)

        # print(
        #     left_wheel.target_rpm,
        #     left_wheel.current_rpm,
        #     left_error,
        #     left_adjustment,
        #     left_wheel.throttle

        # )
        await asyncio.sleep(0)


def calc_rpm(wheel:ZS_X11H_BLDCWheel, start_time:float)->float:
    duration = monotonic() - start_time
    if duration > 0:
        pulses_per_second = wheel.pulse_counter.count / duration
        return pulses_per_second / PULSES_PER_ROTATION * 60
    else:
        return wheel.current_rpm 

class Robot:
    def __init__(self):
        self.target_heading = 0

def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    elif value > max_value: 
        return max_value
    else:
        return value
    

async def udp_monitor(right_wheel:ZS_X11H_BLDCWheel, left_wheel:ZS_X11H_BLDCWheel, bno055:adafruit_bno055.BNO055_I2C):
        #set up udp
    client_ipv4 =  ipaddress.IPv4Address(os.getenv('ROBOT_BASE_IPV4'))
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
    print("Create UDP Client socket")
    sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
    sock.settimeout(TIMEOUT)
    sock.bind((HOST,PORT))
    buf = bytearray(MAXBUF)
    last_command_received = monotonic()
    SAFETY_TIMEOUT = 0.5
    target_heading = 0
    while True:
        try:
            dataready, _, _ = select([sock],[],[],0)
            if len(dataready)> 0:
                size, addr = sock.recvfrom_into(buf)
                command = buf[:size].decode()
                #print('Command: ' + command)
                received_args = json.loads(command)
                correct_heading = False
                if received_args['command'] == 'STOP':
                    await right_wheel.set_target_rpm(0)
                    await left_wheel.set_target_rpm(0)
                    target_heading = bno055.euler[0]
                    #print(f'target heading:{target_heading}')
                else:
                    if received_args['command'] == 'FORWARD':
                        rw_target_rpm = received_args['target_rpm']
                        lw_target_rpm = received_args['target_rpm']
                        left_heading_multiplier = -1
                        right_heading_multiplier = 1
                        correct_heading = True
                    #     print('FORWARD')
                    elif received_args['command'] == 'BACK':
                        rw_target_rpm = -1 * received_args['target_rpm']
                        lw_target_rpm = -1 * received_args['target_rpm']
                        left_heading_multiplier = 1
                        right_heading_multiplier = -1
                        correct_heading = True
                    elif received_args['command'] == 'RIGHT':
                        lw_target_rpm = 1 * received_args['target_rpm']
                        rw_target_rpm = -1 * received_args['target_rpm']
                    elif received_args['command'] == 'LEFT':
                        lw_target_rpm = -1 * received_args['target_rpm']
                        rw_target_rpm = 1 * received_args['target_rpm']

                    if correct_heading:
                        heading_correction = ((bno055.euler[0] - target_heading)/180)
                        if abs(heading_correction) > 180:
                            heading_correction = 180 - heading_correction
                    else: heading_correction = 0

                    await left_wheel.set_target_rpm(int(lw_target_rpm + left_heading_multiplier * lw_target_rpm * heading_correction)) 
                    await right_wheel.set_target_rpm(int(rw_target_rpm + right_heading_multiplier * rw_target_rpm * heading_correction))
                last_command_received = monotonic()
            if monotonic() - last_command_received > SAFETY_TIMEOUT:
                await right_wheel.set_target_rpm(0)
                await left_wheel.set_target_rpm(0)
                print("SAFETY TIMEOUT")
        except OSError as e:
            if e.errno == ETIMEDOUT:
                print("Timeout: emergency stop")
                #stop()
            else: raise e
        await asyncio.sleep(0)

async def demo_mode(right_wheel:ZS_X11H_BLDCWheel, left_wheel:ZS_X11H_BLDCWheel, bno055:adafruit_bno055.BNO055_I2C):
    min_rpm = -100
    max_rpm = 100
    rpm_step = 20
    step_delay = 5
    target_rpm = 0
    await left_wheel.set_target_rpm(target_rpm)
    await right_wheel.set_target_rpm(target_rpm)
    step_multiplier = 1
    while True:
        new_abs_target_rpm = randrange(15,40)
        multiplier = 1 if random() * 2 -1 > 0 else -1
        await left_wheel.set_target_rpm(new_abs_target_rpm * multiplier)
        await right_wheel.set_target_rpm(new_abs_target_rpm * multiplier)
        await asyncio.sleep(step_delay)

async def main():
    i2c = busio.I2C(board.GP13,board.GP12)
    bno055 = adafruit_bno055.BNO055_I2C(i2c)
    right_wheel_pwm = pwmio.PWMOut(board.GP1, frequency=20000)
    right_wheel_pulse_counter = countio.Counter(board.GP3, edge=countio.Edge.RISE)
    right_wheel_pid = PID(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD)
    right_wheel_reverse_switch = DigitalInOut(board.GP0)
    right_wheel_reverse_switch.direction = Direction.OUTPUT
    right_wheel_reverse_switch.value = False 

    left_wheel_pwm = pwmio.PWMOut(board.GP4, frequency=20000)
    left_wheel_pulse_counter = countio.Counter(board.GP7, edge=countio.Edge.RISE)
    left_wheel_pid = PID(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD)
    left_wheel_reverse_switch = DigitalInOut(board.GP16)
    left_wheel_reverse_switch.direction = Direction.OUTPUT
    left_wheel_reverse_switch.value = True 

    right_wheel = ZS_X11H_BLDCWheel(right_wheel_pwm, right_wheel_pulse_counter, PULSES_PER_ROTATION, right_wheel_pid, right_wheel_reverse_switch,True)
    left_wheel = ZS_X11H_BLDCWheel(left_wheel_pwm,left_wheel_pulse_counter, PULSES_PER_ROTATION, left_wheel_pid, left_wheel_reverse_switch, False)
    
    #right_wheel_rpm_monitor_task = asyncio.create_task(right_wheel.rpm_monitor(2,RPM_MONITOR_ZERO_TIMEOUT))
    #left_wheel_rpm_monitor_task = asyncio.create_task(left_wheel.rpm_monitor(2,RPM_MONITOR_ZERO_TIMEOUT))
    differential_synced_rpm_monitor_task = asyncio.create_task(
            differential_synced_rpm_monitor(
                left_wheel,
                right_wheel,
                min_ticks = DIFFERENTIAL_MIN_TICKS,
                timeout=RPM_MONITOR_ZERO_TIMEOUT
            )
        )
    if DEMO:
        demo_mode_task = asyncio.create_task(demo_mode(right_wheel, left_wheel, bno055))
    else: 
        udp_input_monitor_task = asyncio.create_task(udp_monitor(right_wheel, left_wheel, bno055))

    imu_calibrated = False
    await right_wheel.set_target_rpm(0)
    await left_wheel.set_target_rpm(0)
    last_time = monotonic_ns() * SCALE_TIME_NS
    while True:
        loop_start = monotonic_ns() * SCALE_TIME_NS
        
        # print(
        #     left_wheel.target_rpm,
        #     left_wheel.current_rpm,
        #     right_wheel.current_rpm

        # )
        # if not imu_calibrated:
        #     imu_calibrated = bno055.calibration_status[3] == 3

        # #update delta time for PID
        # dt = loop_start - last_time
        # last_time = loop_start

        # #calculate error for PID
        # left_error = (abs(left_wheel.target_rpm) - left_wheel.current_rpm) 
        # right_error = (abs(right_wheel.target_rpm) - right_wheel.current_rpm)
        # #print(f'left error:{left_error}\t\tright error:{right_error}')

        # #calc PID adjustent
        # left_adjustment = left_wheel.pid.update(left_error, dt)
        # right_adjustment = right_wheel.pid.update(right_error,dt)
        # #print(
        #     #  f'left rpm/adjustment:{int(left_wheel.current_rpm)}/{int(left_adjustment)}',
        #     #  f'\t\tright rpm/adjustment:{int(right_wheel.current_rpm)}/{int(right_adjustment)}'
        #     #  )

        # #update motor output
        # left_wheel.throttle = left_wheel.throttle + int(left_adjustment / SCALE_PWM)
        # right_wheel.throttle = right_wheel.throttle + int(right_adjustment / SCALE_PWM)

        # print(left_wheel.target_rpm,
        #     '\t',
        #     int(left_wheel.current_rpm),
        #     '\t\t',
        #     right_wheel.target_rpm,
        #     '\t',
        #     int(right_wheel.current_rpm),
        #       )

        # throttle loop based on desired rate
        loop_end = monotonic_ns() * SCALE_TIME_NS
        loop_duration = loop_end - loop_start
        if loop_duration < 1.0/RATE:
            await asyncio.sleep(1.0/RATE - loop_duration)
            #print( monotonic_ns() * SCALE_TIME_NS - loop_start)
        else:
            print(f'WARNING LOOP TIME OVERFLOW: {loop_duration}')
        await asyncio.sleep(0)

asyncio.run(main())

