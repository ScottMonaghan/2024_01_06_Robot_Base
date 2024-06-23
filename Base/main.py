import board
import pwmio
from time import (
    monotonic,
    monotonic_ns, 
    )
from digitalio import(
    DigitalInOut,
    Direction
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
from random import (randrange, uniform, random)

DEMO = False

#physical robot dimensions. Need to be adjusted to specific robot implementation
WHEEL_CIRCUMFERENCE = 0.47878 #m
BASE_TURN_CIRCUMFERENCE = 1.6757 #m
WHEEL_ROTATIONS_PER_TURN = 3.5
RPM_DIFFERENTIAL_TO_RAD_PER_SEC = 0.01495883 #rad/sec
RPM_LINEAR_TO_METERS_PER_SEC = 0.0079797 #m/sec
MAX_ABS_RPM = 150 #safety
MIN_ABS_RPM = 15 
MAX_THROTTLE = 10000 #safety

RATE = 100 #target rate in hz
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
PID_KI = 1.0
PID_KD = 0

#helper functions
def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    elif value > max_value: 
        return max_value
    else:
        return value

#modified from https://www.kevsrobots.com/resources/how_it_works/pid-controllers.html
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.last_time = 0
    def reset(self):
        self.last_error = 0
        self.integral = 0
    def update(self, error, dt):
        if dt > 0:
             derivative = (error - self.last_error) / dt
        else: 
             derivative = 0
        #if error crosses 0, then zero out integral
        if self.last_error * error < 0:
            self.integral = 0
        else:
            self.integral += error * dt
        output = ( 
            self.Kp * error  
            + self.Ki * self.integral 
            + self.Kd * derivative
        )
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
            reverse_value: bool,
            brake:DigitalInOut
            ):
        self.pwm_out = pwm_out
        self.pulse_counter = pulse_counter
        self.pulses_per_rotation = pulses_per_rotation 
        self.pid = pid
        self._current_rpm = 0.0
        self._target_rpm = 0
        self._last_pulse_record = PulseRecord(self.pulse_counter.count,monotonic_ns() * SCALE_TIME_NS)
        self._reverse_value = reverse_value
        self._reverse_switch = reverse_switch
        self._reverse_switch.value = not self._reverse_value
        self._lock_target_rpm = False
        self.brake = brake

    @property
    def current_abs_rpm(self):
        return self._current_rpm
    
    @current_abs_rpm.setter
    def current_abs_rpm(self, value):
        self._current_rpm = value

    @property
    def current_signed_rpm(self):
        if self._reverse_value == self._reverse_switch.value:
            return -1 * self._current_rpm
        else:
            return self._current_rpm  

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
        if value != 0 and abs(value) < MIN_ABS_RPM:
            if value > 0: value = MIN_ABS_RPM
            else: value = -1 * MIN_ABS_RPM
        self._target_rpm = clamp(value,-1*MAX_ABS_RPM,MAX_ABS_RPM)

    @property
    def throttle(self):
        return self.pwm_out.duty_cycle
    
    @throttle.setter
    def throttle(self, value):
        if value < MIN_NONZERO_THROTTLE: 
            value = 0
        self.pwm_out.duty_cycle = clamp(abs(value),0,MAX_THROTTLE)
    
    def calc_rpm(self, start_time:float)->float:
        duration = monotonic() - start_time
        if duration > 0:
            pulses_per_second = self.pulse_counter.count / duration * 1.0
            return pulses_per_second / self.pulses_per_rotation * 60.0
        else:
            return self.current_abs_rpm 

class Robot:
    def __init__(self, left_wheel:ZS_X11H_BLDCWheel, right_wheel: ZS_X11H_BLDCWheel, bno055: adafruit_bno055.BNO055_I2C):
        self.target_heading = 0
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.bno055 = bno055
        self._target_linear_velocity = 0 #m/sec
        self._target_angular_velocity = 0 #rad/sec

    async def _update_target_rpm(self):
        #first set the base target rpm for linear velocity for both wheels
        lv_target_rpm = self._target_linear_velocity / RPM_LINEAR_TO_METERS_PER_SEC
        #next angular velocity differential
        av_differential = self._target_angular_velocity / RPM_DIFFERENTIAL_TO_RAD_PER_SEC
        #adjust individual wheel target rpm (w/reminder av is counterclockwise)
        await self.right_wheel.set_target_rpm(lv_target_rpm + av_differential/2)
        await self.left_wheel.set_target_rpm(lv_target_rpm - av_differential/2)
    
    @property
    def target_linear_velocity(self):
        return self._target_linear_velocity
    
    async def set_target_linear_and_angular_velocity(self, lv, av):
        self._target_linear_velocity = lv
        self._target_angular_velocity = av
        await self._update_target_rpm()

    @property
    def target_angular_velocity(self):
        return self._target_angular_velocity
    
    def get_current_linear_velocity(self):
        return (self.left_wheel.current_signed_rpm+self.right_wheel.current_signed_rpm)/2 * RPM_LINEAR_TO_METERS_PER_SEC
    def get_current_angular_velocity(self):
        #positive angular velocity is counterclockwise
        return (self.right_wheel.current_signed_rpm-self.left_wheel.current_signed_rpm) * RPM_DIFFERENTIAL_TO_RAD_PER_SEC
    

async def differential_synced_rpm_monitor(robot:Robot, min_ticks:int, timeout:float):
    left_wheel = robot.left_wheel
    right_wheel = robot.right_wheel
    while True:
        left_wheel.pulse_counter.reset()
        right_wheel.pulse_counter.reset()
        start_time = monotonic()
        if left_wheel.target_rpm == 0 and right_wheel.target_rpm == 0:
            left_wheel.throttle = 0
            right_wheel.throttle = 0
            left_wheel.current_abs_rpm = 0
            right_wheel.current_abs_rpm = 0
            current_time = monotonic_ns() * SCALE_TIME_NS
            left_wheel.pid.last_time = current_time
            right_wheel.pid.last_time = current_time
        else:
            if left_wheel.throttle != 0 and right_wheel.throttle !=0:
                while (
                    (left_wheel.pulse_counter.count < min_ticks
                    or right_wheel.pulse_counter.count < min_ticks)
                    and monotonic()-start_time < timeout
                    ):
                    await asyncio.sleep(0)

            left_wheel.current_abs_rpm = 0 if left_wheel.pulse_counter.count < min_ticks else left_wheel.calc_rpm(start_time)
            right_wheel.current_abs_rpm = 0 if right_wheel.pulse_counter.count < min_ticks else right_wheel.calc_rpm(start_time)

            ##lets see what happens if we do pid here
            current_time = monotonic_ns() * SCALE_TIME_NS
            dt = current_time - left_wheel.pid.last_time
            left_wheel.pid.last_time = current_time
            right_wheel.pid.last_time = current_time

            # calculate error for PID
            left_error = (abs(left_wheel.target_rpm) - left_wheel.current_abs_rpm) 
            right_error = (abs(right_wheel.target_rpm) - right_wheel.current_abs_rpm)
    
            # #calc PID adjustent
            left_adjustment = left_wheel.pid.update(left_error, dt)
            right_adjustment = right_wheel.pid.update(right_error,dt)

            left_wheel.throttle = 0 if left_wheel.target_rpm == 0 else left_wheel.throttle + int(left_adjustment / SCALE_PWM)
            right_wheel.throttle = 0 if right_wheel.target_rpm == 0 else right_wheel.throttle + int(right_adjustment / SCALE_PWM)
        await asyncio.sleep(0)

async def udp_monitor(robot:Robot):
    #set up udp
    left_wheel = robot.left_wheel
    right_wheel = robot.right_wheel
    bno055 = robot.bno055
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
                    left_wheel.pid.integral = 0
                    right_wheel.pid.integral = 0
                    robot.target_heading = bno055.euler[0]
                    left_wheel.brake.value = True
                    right_wheel.brake.value = True
                    #print(f'target heading:{target_heading}')
                else:
                    if received_args['command'] == 'FORWARD':
                        rw_target_rpm = received_args['target_rpm']
                        lw_target_rpm = received_args['target_rpm']
                        left_heading_multiplier = -1
                        right_heading_multiplier = 1
                        correct_heading = True
                        left_wheel.brake.value = False
                        right_wheel.brake.value = False
                    #     print('FORWARD')
                    elif received_args['command'] == 'BACK':
                        rw_target_rpm = -1 * received_args['target_rpm']
                        lw_target_rpm = -1 * received_args['target_rpm']
                        left_heading_multiplier = 1
                        right_heading_multiplier = -1
                        correct_heading = True
                        left_wheel.brake.value = False
                        right_wheel.brake.value = False
                    elif received_args['command'] == 'RIGHT':
                        lw_target_rpm = 1 * received_args['target_rpm']
                        rw_target_rpm = -1 * received_args['target_rpm']
                        left_wheel.brake.value = False
                        right_wheel.brake.value = False
                    elif received_args['command'] == 'LEFT':
                        lw_target_rpm = -1 * received_args['target_rpm']
                        rw_target_rpm = 1 * received_args['target_rpm']
                        left_wheel.brake.value = False
                        right_wheel.brake.value = False
                    if correct_heading:
                        heading_delta = bno055.euler[0] - robot.target_heading
                        if abs(heading_delta) > 180:
                            if heading_delta < 0:
                                sign_multiplier = -1
                            else:
                                sign_multiplier = 1
                            heading_delta = (360 - abs(heading_delta)) * sign_multiplier *-1
                        heading_correction = heading_delta / 180
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

async def demo_mode(robot:Robot):
    right_wheel = robot.right_wheel
    left_wheel = robot.left_wheel
    step_delay = 8
    target_rpm = 0
    await left_wheel.set_target_rpm(0)
    await right_wheel.set_target_rpm(0)
    step_multiplier = 1
    while True:
        lv_multiplier = 1 if random() > 1/2 else -1
        av_multiplier = 1 if random() > 1/2 else -1
        new_target_linear_velocity = 0.2
        new_target_angular_velocity = uniform(0,2) * av_multiplier
        await robot.set_target_linear_and_angular_velocity(new_target_linear_velocity, new_target_angular_velocity)
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
    right_wheel_brake = DigitalInOut(board.GP5)
    right_wheel_brake.direction = Direction.OUTPUT
    right_wheel_brake.value = False

    left_wheel_pwm = pwmio.PWMOut(board.GP4, frequency=20000)
    left_wheel_pulse_counter = countio.Counter(board.GP7, edge=countio.Edge.RISE)
    left_wheel_pid = PID(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD)
    left_wheel_reverse_switch = DigitalInOut(board.GP16)
    left_wheel_reverse_switch.direction = Direction.OUTPUT
    left_wheel_reverse_switch.value = True 
    left_wheel_brake = DigitalInOut(board.GP15)
    left_wheel_brake.direction = Direction.OUTPUT
    left_wheel_brake.value = False

    right_wheel = ZS_X11H_BLDCWheel(right_wheel_pwm, right_wheel_pulse_counter, PULSES_PER_ROTATION, right_wheel_pid, right_wheel_reverse_switch,True, right_wheel_brake)
    left_wheel = ZS_X11H_BLDCWheel(left_wheel_pwm,left_wheel_pulse_counter, PULSES_PER_ROTATION, left_wheel_pid, left_wheel_reverse_switch, False,left_wheel_brake)
    
    robot = Robot(left_wheel,right_wheel, bno055)

    differential_synced_rpm_monitor_task = asyncio.create_task(
            differential_synced_rpm_monitor(
                robot = robot,
                min_ticks = DIFFERENTIAL_MIN_TICKS,
                timeout=RPM_MONITOR_ZERO_TIMEOUT
            )
        )
    if DEMO:
        demo_mode_task = asyncio.create_task(demo_mode(robot))
    else: 
        udp_input_monitor_task = asyncio.create_task(udp_monitor(robot))

    imu_calibrated = False
    await right_wheel.set_target_rpm(0)
    await left_wheel.set_target_rpm(0)
    last_time = monotonic_ns() * SCALE_TIME_NS
    while True:
        loop_start = monotonic_ns() * SCALE_TIME_NS
        
        print(
            #f'ltrpm:{left_wheel.target_rpm:.2f}\t',
            # f'lcrpm:{left_wheel.current_signed_rpm:.2f}\t',
            #f'rtrpm:{right_wheel.target_rpm:.2f}\t',
            # f'rcrpm:{right_wheel.current_signed_rpm:.2f}\t',
            f'target lv:{robot.target_linear_velocity:.2f} m\s\t',
            f'actual lv:{robot.get_current_linear_velocity():.2f} m\s\t',
            f'target av:{robot.target_angular_velocity:.2f} rad\s\t',
            f'actual av:{robot.get_current_angular_velocity():.2f} rad\s\t',
            '\r'
        )
       
        loop_end = monotonic_ns() * SCALE_TIME_NS
        loop_duration = loop_end - loop_start
        if loop_duration < 1.0/RATE:
            await asyncio.sleep(1.0/RATE - loop_duration)
        else:
            print(f'WARNING LOOP TIME OVERFLOW: {loop_duration}')
        await asyncio.sleep(0)

asyncio.run(main())

