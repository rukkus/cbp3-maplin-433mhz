import time


from modules import cbpi
from modules.core.hardware import ActorBase, SensorPassive, SensorActive
from modules.core.props import Property
import wiringpi

wiringpi.wiringPiSetup()

class RemoteSocket(object):
    channel_codes = [
        [0x33353335, 0x33533335, 0x35333335, 0x53333335],
        [0x33353353, 0x33533353, 0x35333353, 0x53333353],
        [0x33353533, 0x33533533, 0x35333533, 0x53333533],
        [0x33355333, 0x33535333, 0x35335333, 0x53335333]
    ]

    on_code = 0x3333
    off_code = 0x5333

    default_pulse_width = 450 * 1e-6 # Measured from Maplin transmitters
    preamble = [0] * 26
    sync = [1]
    postamble = [0] * 2

    def __init__(self):
        wiringpi.pinMode(0, wiringpi.OUTPUT)

    # converts the lowest bit_count bits to a list of ints
    def int_to_bit_list(self, i, bit_count):
        result = []
        shifted = i
        for i in range(0, bit_count):
            result.append(shifted & 0x01)
            shifted >>= 1
        return result

    # encodes 0 as a 1 count state change, 1 as a 3 count state change, starting
    # with a change to low
    def encode_as_state_list(self, bit_list):
        result = []
        state = 0
        for bit in bit_list:
            result.extend([state] if bit == 0 else [state, state, state])
            state = 1 - state
        return result

    def encode_packet(self, bit_list):
        return self.preamble + self.sync + self.encode_as_state_list(bit_list) + self.postamble

    def command_as_bit_list(self, channel, button, on):
        return self.int_to_bit_list(
            self.channel_codes[channel - 1][button - 1], 32) + \
            self.int_to_bit_list(self.on_code if on else self.off_code, 16)

    def busy_wait_until(self, end_time):
        while time.time() <= end_time: pass

    def send(self, state_list, pulse_width):
         end_time = time.time()
         for state in state_list:
             end_time = end_time + pulse_width
             wiringpi.digitalWrite(0, state)
             self.busy_wait_until(end_time)


    def send_command(self, channel, button, on):
        self.send(self.encode_packet(self.command_as_bit_list(channel, button, on)), self.default_pulse_width)

    def switchOn(self, group, socket):
        for i in range(1, 6):
            self.send_command(group, socket, True)

    def switchOff(self, group, socket):
        for i in range(1, 6):
            self.send_command(group, socket, False)



@cbpi.actor
class Maplin433MHzSocket(ActorBase):
    group = Property.Select("group", options=[1, 2, 3, 4])
    socket = Property.Select("socket", options=[1, 2, 3, 4])

    @classmethod
    def init_global(cls):
        cls.device = RemoteSocket()

    def on(self, power=100):
        self.device.switchOn(int(self.group), int(self.socket))

    def off(self):
        self.device.switchOff(int(self.group), int(self.socket))
