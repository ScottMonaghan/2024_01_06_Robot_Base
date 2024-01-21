import board
import pwmio
from time import (
    sleep,
    monotonic,
    monotonic_ns, 
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

RATE = 10 #target rate in hz
SCALE_TIME_NS = 1e-9
SCALE_TIME_MS = 1e-3
SCALE_PWM = 1e-1
SCALE_ERROR = 10
SCALE_HEADING_CORRECTION = 0.5
RPM_TIMEOUT = 0.5
PULSES_PER_ROTATION = 45
MIN_PULSES_FOR_CALCULATION = 3

#modified from https://www.kevsrobots.com/resources/how_it_works/pid-controllers.html
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

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
            ):
        self.pwm_out = pwm_out
        self.pulse_counter = pulse_counter
        self.pid = pid
        self.current_rpm = 0.0
        self.target_rpm = 0
        self._last_pulse_record = PulseRecord(self.pulse_counter.count,monotonic_ns() * SCALE_TIME_NS)

    async def rpm_monitor(self, min_ticks, timeout):
        while True:
            self.pulse_counter.reset()
            start_time = monotonic()
            while self.pulse_counter.count < min_ticks and monotonic()-start_time < timeout:
                await asyncio.sleep(0)
            if monotonic()-start_time > timeout:
                 #print(monotonic()-start_time)
                 self.current_rpm = 0
            else:
                pulses_per_second = self.pulse_counter.count / (monotonic() - start_time)
                self.current_rpm = pulses_per_second / PULSES_PER_ROTATION * 60
            await asyncio.sleep(0)

class Robot:
    def __init__(self):
        self.target_heading = 0
    # def update_current_rpm(self):
    #     current_pulse_record = PulseRecord(self.pulse_counter.count,monotonic_ns() * SCALE_TIME_NS)
    #     pulses_since_last =  current_pulse_record.pulse_count - self._last_pulse_record.pulse_count
    #     seconds_since_last = current_pulse_record.timestamp - self._last_pulse_record.timestamp
    #     if pulses_since_last > MIN_PULSES_FOR_CALCULATION:
    #         #print(seconds_since_last)
    #         pulses_per_second = pulses_since_last/seconds_since_last
    #         #print(pulses_per_second)
    #         #convert to RPM
    #         updated_rpm = (pulses_per_second * 60) / PULSES_PER_ROTATION
    #         self.current_rpm = updated_rpm
    #         self._last_pulse_record = current_pulse_record
    #     elif seconds_since_last > RPM_TIMEOUT:
    #         self.current_rpm = 0.0
            #print('RPM TIMEOUT')

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
                if received_args['command'] == 'STOP':
                    right_wheel.target_rpm = 0
                    left_wheel.target_rpm = 0
                    target_heading = bno055.euler[0]
                    print(f'target heading:{target_heading}')
                elif received_args['command'] == 'FORWARD':
                    rw_target_rpm = received_args['target_rpm']
                    lw_target_rpm = received_args['target_rpm']

                    heading_correction = (bno055.euler[0] - target_heading) * SCALE_HEADING_CORRECTION
                    if abs(heading_correction) > 180:
                        heading_correction = heading_correction - 180
                    print(heading_correction)

                    left_wheel.target_rpm = lw_target_rpm - heading_correction
                    right_wheel.target_rpm = rw_target_rpm + heading_correction
                last_command_received = monotonic()
            if monotonic() - last_command_received > SAFETY_TIMEOUT:
                right_wheel.target_rpm = 0
                left_wheel.target_rpm = 0
                print("SAFETY TIMEOUT")
        except OSError as e:
            if e.errno == ETIMEDOUT:
                print("Timeout: emergency stop")
                #stop()
            else: raise e
        await asyncio.sleep(0)

async def main():
    i2c = busio.I2C(board.GP13,board.GP12)
    bno055 = adafruit_bno055.BNO055_I2C(i2c)
    right_wheel_pwm = pwmio.PWMOut(board.GP1, frequency=20000)
    right_wheel_pulse_counter = countio.Counter(board.GP3, edge=countio.Edge.RISE)
    right_wheel_pid = PID(Kp=1.0, Ki=0.04, Kd=0)

    left_wheel_pwm = pwmio.PWMOut(board.GP4, frequency=20000)
    left_wheel_pulse_counter = countio.Counter(board.GP7, edge=countio.Edge.RISE)
    left_wheel_pid = PID(Kp=1.0, Ki=0.04, Kd=0)

    right_wheel = ZS_X11H_BLDCWheel(right_wheel_pwm, right_wheel_pulse_counter, PULSES_PER_ROTATION, right_wheel_pid)
    left_wheel = ZS_X11H_BLDCWheel(left_wheel_pwm,left_wheel_pulse_counter, PULSES_PER_ROTATION, left_wheel_pid)

    #right_wheel.pwm_out.duty_cycle = 2000
    #left_wheel.pwm_out.duty_cycle = 2000
    
    right_wheel_rpm_monitor_task = asyncio.create_task(right_wheel.rpm_monitor(2,0.5))
    left_wheel_rpm_monitor_task = asyncio.create_task(left_wheel.rpm_monitor(2,0.5))
    udp_input_monitor_task = asyncio.create_task(udp_monitor(right_wheel, left_wheel, bno055))

    imu_calibrated = False
    right_wheel.target_rpm = 0
    left_wheel.target_rpm = 0
    last_time = monotonic_ns() * SCALE_TIME_NS
    while True:
        loop_start = monotonic_ns() * SCALE_TIME_NS
        
        if not imu_calibrated:
            imu_calibrated = bno055.calibration_status[3] == 3

        #update delta time for PID
        dt = loop_start - last_time
        last_time = loop_start

        #check for updated udp commands
        
        #update rpm of wheels
        #left_wheel.update_current_rpm()
        #right_wheel.update_current_rpm()
        #print(f'left rpm:{left_wheel.current_rpm}\t\tright rpm:{right_wheel.current_rpm}')

        #calculate error for PID
        left_error = (left_wheel.target_rpm - left_wheel.current_rpm) 
        right_error = (right_wheel.target_rpm - right_wheel.current_rpm)
        #print(f'left error:{left_error}\t\tright error:{right_error}')

        #calc PID adjustent
        left_adjustment = left_wheel.pid.update(left_error, dt)
        right_adjustment = right_wheel.pid.update(right_error,dt)
        #print(
            #  f'left rpm/adjustment:{int(left_wheel.current_rpm)}/{int(left_adjustment)}',
            #  f'\t\tright rpm/adjustment:{int(right_wheel.current_rpm)}/{int(right_adjustment)}'
            #  )

        #update motor output
        left_motor_output = clamp(left_wheel.pwm_out.duty_cycle + int(left_adjustment / SCALE_PWM),0,65535) 
        right_motor_output = clamp(right_wheel.pwm_out.duty_cycle + int(right_adjustment / SCALE_PWM),0,65535) 
        
        left_wheel.pwm_out.duty_cycle = left_motor_output
        right_wheel.pwm_out.duty_cycle = right_motor_output

        

        # throttle loop based on desired rate
        loop_end = monotonic_ns() * SCALE_TIME_NS
        loop_duration = loop_end - loop_start
        if loop_duration < 1.0/RATE:
            await asyncio.sleep(1.0/RATE - loop_duration)
            #print( monotonic_ns() * SCALE_TIME_NS - loop_start)
        else:
            print(f'WARNING LOOP TIME OVERFLOW: {loop_duration}')

        # dt = (monotonic_ns() / 1e+11) - last_time
        # rw_throttle = 0
        # lw_throttle = 0
        # if right_wheel.target_rpm > 0:
        #     rw_error = (right_wheel.target_rpm-right_wheel.current_rpm) * 10
        #     rw_throttle = right_wheel.pwm_out.duty_cycle
        #     rw_throttle += int(right_wheel.pid.update(rw_error,dt))
        #     rw_throttle = ZS_X11H_BLDCWheel.clamp(rw_throttle,0,65535)
        # if left_wheel.target_rpm > 0:
        #     lw_error = (left_wheel.target_rpm-left_wheel.current_rpm) * 10
        #     lw_throttle = left_wheel.pwm_out.duty_cycle
        #     lw_throttle += int(left_wheel.pid.update(lw_error,dt))
        #     lw_throttle = ZS_X11H_BLDCWheel.clamp(lw_throttle,0,65535)
        # right_wheel.pwm_out.duty_cycle = rw_throttle
        # left_wheel.pwm_out.duty_cycle = lw_throttle
        # #print (f'right: {int(right_wheel.current_rpm)}\t\tleft:  {int(left_wheel.current_rpm)}')
        # # if monotonic() - speed_change_counter > 3:
        # #     if target_rpm >= 170: sign = -1
        # #     elif target_rpm <= 20: sign = 1
        # #     target_rpm += 50 * sign
        # #     speed_change_counter = monotonic()
        # await asyncio.sleep(0.1)

    #await asyncio.gather(right_wheel_rpm_monitor_task ,left_wheel_rpm_monitor_task)
asyncio.run(main())

