import pigpio

class Infrared:
    # IR sensors
    LEFT_IR = 14
    MIDDLE_IR = 15
    RIGHT_IR = 18

    IR_CHANNELS = [LEFT_IR, MIDDLE_IR, RIGHT_IR]

    # IR states
    COMPLETELY_OFF = (0, 0, 0)
    OFF_LEFT = (0, 0, 1)
    SLIGHT_OFF_LEFT = (0, 1, 1)
    OFF_RIGHT = (1, 0, 0)
    SLIGHT_OFF_RIGHT = (1, 1, 0)
    CENTERED = (0, 1, 0)
    CENTER_OFF = (1, 0, 1)
    COMPLETELY_ON = (1, 1, 1)

    def __init__(self, io):
        self.io = io 

        # Set up the three IR sensors as inputs
        for ir_pin in Infrared.IR_CHANNELS:
            self.io.set_mode(ir_pin, pigpio.INPUT)

    def get_ir_states(self):
        return tuple([self.io.read(ir_pin) for ir_pin in Infrared.IR_CHANNELS])
