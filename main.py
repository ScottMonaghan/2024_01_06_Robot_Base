import board
import pwmio
from time import (
    sleep,
    monotonic,
    monotonic_ns, 
    )
import countio
import asyncio

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
        print(self.integral)
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output


PULSES_PER_ROTATION = 45
MINIUM_SAMPLE_DURATION = 0.05

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
        self.pulses_per_rotation = pulses_per_rotation
        self.pid = pid
        self.current_rpm = 0.0

    @staticmethod
    def clamp(value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value: 
            return max_value
        else:
            return value
        
    async def rpm_monitor(self, min_ticks, timeout):
        while True:
            self.pulse_counter.reset()
            start_time = monotonic()
            while self.pulse_counter.count < min_ticks and monotonic()-start_time < timeout:
                await asyncio.sleep(0)
            if monotonic()-start_time > timeout:
                 print(monotonic()-start_time)
                 self.current_rpm = 0
            else:
                pulses_per_second = self.pulse_counter.count / (monotonic() - start_time)
                self.current_rpm = pulses_per_second / PULSES_PER_ROTATION * 60
            await asyncio.sleep(0)

async def main():  
    right_wheel_pwm = pwmio.PWMOut(board.GP1, frequency=20000)
    right_wheel_pulse_counter = countio.Counter(board.GP3, edge=countio.Edge.RISE)
    right_wheel_pid = PID(Kp=1.0, Ki=0.04, Kd=0)

    left_wheel_pwm = pwmio.PWMOut(board.GP4, frequency=20000)
    left_wheel_pulse_counter = countio.Counter(board.GP7)
    left_wheel_pid = PID(Kp=1.0, Ki=0.04, Kd=0)

    right_wheel = ZS_X11H_BLDCWheel(right_wheel_pwm, right_wheel_pulse_counter, PULSES_PER_ROTATION, right_wheel_pid)
    left_wheel = ZS_X11H_BLDCWheel(left_wheel_pwm,left_wheel_pulse_counter, PULSES_PER_ROTATION, left_wheel_pid)

    right_wheel_rpm_monitor_task = asyncio.create_task(right_wheel.rpm_monitor(2,0.5))
    left_wheel_rpm_monitor_task = asyncio.create_task(left_wheel.rpm_monitor(2,0.5))

    right_wheel.pwm_out.duty_cycle = 0
    left_wheel.pwm_out.duty_cycle = 0
    
    
    last_time = monotonic_ns() / 1e+11
    target_rpm = 40
    speed_change_counter = monotonic()
    while True:
        dt = (monotonic_ns() / 1e+11) - last_time
        rw_error = (target_rpm-right_wheel.current_rpm) * 10
        rw_throttle = right_wheel.pwm_out.duty_cycle
        rw_throttle += int(right_wheel.pid.update(rw_error,dt))
        rw_throttle = ZS_X11H_BLDCWheel.clamp(rw_throttle,100,65535)
        lw_error = (target_rpm-left_wheel.current_rpm) * 10
        lw_throttle = left_wheel.pwm_out.duty_cycle
        lw_throttle += int(left_wheel.pid.update(lw_error,dt))
        lw_throttle = ZS_X11H_BLDCWheel.clamp(lw_throttle,100,65535)
        right_wheel.pwm_out.duty_cycle = rw_throttle
        left_wheel.pwm_out.duty_cycle = lw_throttle
        print (f'right: {int(right_wheel.current_rpm)}\t\tleft:  {int(left_wheel.current_rpm)}')
        # if monotonic() - speed_change_counter > 3:
        #     if target_rpm >= 170: sign = -1
        #     elif target_rpm <= 20: sign = 1
        #     target_rpm += 50 * sign
        #     speed_change_counter = monotonic()
        await asyncio.sleep(0.1)

    await asyncio.gather(right_wheel_rpm_monitor_task ,left_wheel_rpm_monitor_task)
asyncio.run(main())

