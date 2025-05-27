from machine import Pin
import time


class Motor:
    stepms = 10

    # Do be defined by subclasses
    maxpos = 0
    states = []

    def __init__(self, p1, p2, p3, p4, stepms=None):
        self.pins = [p1, p2, p3, p4]

        if stepms is not None:
            self.stepms = stepms

        self._state = 0
        self._pos = 0

    def __repr__(self):
        return '<{} @ {}>'.format(
            self.__class__.__name__,
            self.pos,
        )

    @property
    def pos(self):
        return self._pos

    @classmethod
    def frompins(cls, *pins, **kwargs):
        return cls(*[Pin(pin, Pin.OUT) for pin in pins],
                   **kwargs)

    def reset(self):
        self._pos = 0

    def _step(self, dir):
        state = self.states[self._state]

        for i, val in enumerate(state):
            self.pins[i].value(val)

        self._state = (self._state + dir) % len(self.states)
        self._pos = (self._pos + dir) % self.maxpos

    def step(self, steps):
        dir = 1 if steps >= 0 else -1
        steps = abs(steps)

        for _ in range(steps):
            t_start = time.ticks_ms()

            self._step(dir)

            t_end = time.ticks_ms()
            t_delta = time.ticks_diff(t_end, t_start)
            time.sleep_ms(self.stepms - t_delta)

    def step_until(self, target, dir=None):
        if target < 0 or target > self.maxpos:
            raise ValueError(target)

        if dir is None:
            dir = 1 if target > self._pos else -1
            if abs(target - self._pos) > self.maxpos/2:
                dir = -dir

        while True:
            if self._pos == target:
                break
            self.step(dir)

    def step_until_angle(self, angle, dir=None):
        if angle < 0 or angle > 360:
            raise ValueError(angle)

        target = int(angle / 360 * self.maxpos)
        self.step_until(target, dir=dir)

    def step_degrees(self, degrees):
        if degrees < 0 or degrees > 360:
            raise ValueError("Degrees should be between 0 and 360")

        steps_to_take = int(degrees / 360 * self.maxpos)

        self.zero()  # Ignore the current position, start from zero
        self.step(steps_to_take)


class HalfStepMotor(Motor):
    stepms = 3
    maxpos = 4096
    states = [
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1],
        [1, 0, 0, 1],
    ]


STEPPERI_IN1 = 10
STEPPERI_IN2 = 11
STEPPERI_IN3 = 12
STEPPERI_IN4 = 13

# Initialize the stepper motor
stepper_motor = HalfStepMotor.frompins(
    STEPPERI_IN1, STEPPERI_IN2, STEPPERI_IN3, STEPPERI_IN4)

# Set the current position as position 0
stepper_motor.reset()

try:
    # Move counterclockwise full step count to calibrate end stop position
    stepper_motor.step(-4200)
    time.sleep(1.5)  # stop for a while

    # Step clockwise back to middle position
    stepper_motor.step(1930)
    time.sleep(0.5)  # stop for a while

    # Move 400 steps in counterclockwise direction
    stepper_motor.step(-400)
    time.sleep(0.5)  # stop for a while

    while True:
        # Move 800 steps in clockwise direction
        stepper_motor.step(800)
        time.sleep(0.5)  # stop for a while

        # Move 800 steps in counterclockwise direction
        stepper_motor.step(-800)
        time.sleep(0.5)  # stop for a while
except KeyboardInterrupt:
    print('Keyboard interrupt')
