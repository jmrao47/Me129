import pigpio
import time
import random 
from numpy import inf

class Ultrasound:
    ECHO_TO_INDEX = {16: 0, 20: 1, 21: 2}

    # Ultrasound sensors
    ULTRASOUND_1_ECHO = 16
    ULTRASOUND_2_ECHO = 20
    ULTRASOUND_3_ECHO = 21

    ULTRASOUND_1_TRIGGER = 13
    ULTRASOUND_2_TRIGGER = 19
    ULTRASOUND_3_TRIGGER = 26

    ULTRASOUND_CHANNELS = [(ULTRASOUND_1_ECHO, ULTRASOUND_1_TRIGGER), 
                            (ULTRASOUND_2_ECHO, ULTRASOUND_2_TRIGGER), 
                            (ULTRASOUND_3_ECHO, ULTRASOUND_3_TRIGGER)]

    EXPECT_RISE = 0
    EXPECT_FALL = 1
    DEFAULT_STATE = 2

    def __init__(self, io):
        self.io = io
        self.stop_flag = True

        # Set up three ultrasound channels
        self.cb_rises = []
        self.cb_falls = []
        self.last_rises = []
        self.last_falls = []
        self.distances = []
        self.states = []

        # Rising callback
        def rising(gpio, level, tick):
            index = Ultrasound.ECHO_TO_INDEX[gpio]
            #print(f'RISING, echo: {gpio}, tick: {tick}, state: {self.states[index]}')

            if self.states[index] == Ultrasound.EXPECT_RISE:
                self.last_rises[index] = tick
                self.states[index] = Ultrasound.EXPECT_FALL

        # Falling callback
        def falling(gpio, level, tick):
            index = Ultrasound.ECHO_TO_INDEX[gpio]
            #print(f'FALLING, echo: {gpio}, tick: {tick}, state: {self.states[index]}')

            if self.states[index] == Ultrasound.EXPECT_FALL:
                time_elapsed = tick - self.last_rises[index]

                self.distances[index] = 343 / 2 * time_elapsed * (10 ** -6)
                self.states[index] = Ultrasound.DEFAULT_STATE

        for echo, trigger in Ultrasound.ULTRASOUND_CHANNELS:
            io.set_mode(echo, pigpio.INPUT)
            io.set_mode(trigger, pigpio.OUTPUT)

            self.cb_rises.append(self.io.callback(echo, pigpio.RISING_EDGE, rising))
            self.cb_falls.append(self.io.callback(echo, pigpio.FALLING_EDGE, falling))

            self.last_rises.append(0)
            self.last_falls.append(0)
            self.distances.append(inf)
            self.states.append(Ultrasound.DEFAULT_STATE)

    def trigger(self):
        for i in range(len(Ultrasound.ULTRASOUND_CHANNELS)):
            _, trigger = Ultrasound.ULTRASOUND_CHANNELS[i]
            self.states[i] = Ultrasound.EXPECT_RISE

            self.io.write(trigger, 1)
            time.sleep(0.00001)
            self.io.write(trigger, 0)

    def stop_continual(self):
        self.stop_flag = True

    def run_continual(self):
        self.stop_flag = False
        while not self.stop_flag:
            self.trigger()
            #print(f'distances: {self.get_distances()}')
            time.sleep(0.08 + 0.04 * random.random())

    def get_distances(self, index=None):
        if index is None:
            return self.distances

        return self.distances[index]

    def shutdown(self):
        for cb in self.cb_rises:
            cb.cancel()
        
        for cb in self.cb_falls:
            cb.cancel()


        