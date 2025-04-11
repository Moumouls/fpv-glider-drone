import hid
import time
import pigpio
import threading

class X:

   GAP=300
   WAVES=3

   def __init__(self, pi, gpio, channels=8, frame_ms=27):
      self.pi = pi
      self.gpio = gpio

      self.frame_ms = frame_ms

      self._frame_us = int(frame_ms * 1000)
      self._frame_secs = frame_ms / 1000.0

      if channels < 1:
         channels = 1
      elif channels > (frame_ms // 2):
         channels = int(frame_ms // 2)
      self.channels = channels

      self._widths = [1000] * channels # set each channel to minimum pulse width

      self._wid = [None]*self.WAVES
      self._next_wid = 0

      pi.write(gpio, pigpio.LOW)

      self._update_time = time.time()

   def _update(self):
    wf =[]
    micros = 0
    for i in self._widths:
        wf.append(pigpio.pulse(1<<self.gpio, 0, self.GAP))
        wf.append(pigpio.pulse(0, 1<<self.gpio, i-self.GAP))
        micros += i
    # off for the remaining frame period
    wf.append(pigpio.pulse(1<<self.gpio, 0, self.GAP))
    micros += self.GAP
    wf.append(pigpio.pulse(0, 1<<self.gpio, self._frame_us-micros))

    self.pi.wave_add_generic(wf)
    wid = self.pi.wave_create()
    self.pi.wave_send_using_mode(wid, pigpio.WAVE_MODE_REPEAT_SYNC)
    self._wid[self._next_wid] = wid

    self._next_wid += 1
    if self._next_wid >= self.WAVES:
        self._next_wid = 0

    remaining = self._update_time + self._frame_secs - time.time()
    if remaining > 0:
        time.sleep(remaining)
    self._update_time = time.time()

    wid = self._wid[self._next_wid]
    if wid is not None:
        try:
            self.pi.wave_delete(wid)
        except pigpio.error as e:
            print(f"Failed to delete wave id {wid}: {e}")
        self._wid[self._next_wid] = None

   def update_channel(self, channel, width):
      self._widths[channel] = width
      self._update()

   def update_channels(self, widths):
      self._widths[0:len(widths)] = widths[0:self.channels]
      self._update()

   def cancel(self):
      self.pi.wave_tx_stop()
      for i in self._wid:
         if i is not None:
            self.pi.wave_delete(i)

MAX_STICK_VALUE = 65535
MIDDLE_STICK_VALUE = 32767

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Failed to connect to pigpio daemon")

# PPM configuration
PPM_GPIO = 4  # GPIO pin for PPM output
FRAME_MS = 21  # Frame duration in milliseconds
CHANNEL_COUNT = 8  # Increased to 8 channels

# Initialize the PPM generator
ppm = X(pi, PPM_GPIO, channels=CHANNEL_COUNT, frame_ms=FRAME_MS)

# Find devices
devices = hid.enumerate()
flight_stick = next((dev for dev in devices if dev['product_string'] == "VelocityOne Flightstick"), None)
flight_rudder = next((dev for dev in devices if dev['product_string'] == "VelocityOne Rudder"), None)

if not flight_stick:
    raise RuntimeError('Flight stick not found')

if not flight_rudder:
    raise RuntimeError('Flight rudder not found')

print('All commands found')

# Open HID devices
stick_hid = hid.device()
stick_hid.open(flight_stick['vendor_id'], flight_stick['product_id'])

rudder_hid = hid.device()
rudder_hid.open(flight_rudder['vendor_id'], flight_rudder['product_id'])

control_state = {
    'motor': 0,
    'flap': 0,
    'elevator': MIDDLE_STICK_VALUE,
    'aileron': MIDDLE_STICK_VALUE,
    'rudder': MIDDLE_STICK_VALUE,
    'channel7': MIDDLE_STICK_VALUE,
    'channel8': MIDDLE_STICK_VALUE,
}

def map_range(value, from_min, from_max, to_min, to_max):
    return int((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min)

# New PPM input configuration
PPM_IN_GPIO = 18  # GPIO pin for PPM input
start_of_frame = False
channel = 0
last_tick = None

def ppm_input_callback(gpio, level, tick):
    global start_of_frame, channel, last_tick, control_state
    if last_tick is not None:
        diff = pigpio.tickDiff(last_tick, tick)
        if diff > 2990:  # start of frame
            start_of_frame = True
            channel = 0
        else:
            if start_of_frame:
                if channel == 6:  # Channel 7 (0-indexed)
                    control_state['channel7'] = map_range(diff, 1000, 2000, 0, MAX_STICK_VALUE)
                elif channel == 7:  # Channel 8 (0-indexed)
                    control_state['channel8'] = map_range(diff, 1000, 2000, 0, MAX_STICK_VALUE)
                channel += 1
    last_tick = tick

# Set up PPM input callback
pi.set_mode(PPM_IN_GPIO, pigpio.INPUT)
ppm_input_cb = pi.callback(PPM_IN_GPIO, pigpio.RISING_EDGE, ppm_input_callback)

def update_ppm():
    channels = [
        map_range(control_state['aileron'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['elevator'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['motor'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['rudder'], 0, MAX_STICK_VALUE, 1000, 2000),
        1000,  # Channel 5 Arm switch managed on the TX by default to false for security reason
        map_range(control_state['flap'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['channel7'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['channel8'], 0, MAX_STICK_VALUE, 1000, 2000),
    ]
    ppm.update_channels(channels)

latest_stick_data = None
latest_rudder_data = None
input_lock = threading.Lock()

# Thread function for reading stick data
def read_stick_data(stick_hid):
    global latest_stick_data
    while True:
        try:
            data = stick_hid.read(64, timeout_ms=10)
            if data:
                with input_lock:
                    latest_stick_data = data
        except:
            break

# Thread function for reading rudder data
def read_rudder_data(rudder_hid):
    global latest_rudder_data
    while True:
        try:
            data = rudder_hid.read(64, timeout_ms=10)
            if data:
                with input_lock:
                    latest_rudder_data = data
        except:
            break

stick_thread = threading.Thread(target=read_stick_data, args=(stick_hid,), daemon=True)
rudder_thread = threading.Thread(target=read_rudder_data, args=(rudder_hid,), daemon=True)

stick_thread.start()
rudder_thread.start()

print('Listening for data and generating PPM signal')
update_ppm()

try:
    while True:
        # Process latest input data
        with input_lock:
            if latest_stick_data:
                control_state['elevator'] = int.from_bytes(latest_stick_data[3:5], byteorder='little')
                control_state['aileron'] = int.from_bytes(latest_stick_data[1:3], byteorder='little')
                control_state['motor'] = int.from_bytes(latest_stick_data[11:13], byteorder='little')
                control_state['flap'] = int.from_bytes(latest_stick_data[13:15], byteorder='little')
                latest_stick_data = None  # Clear processed data

            if latest_rudder_data:
                control_state['rudder'] = int.from_bytes(latest_rudder_data[5:7], byteorder='little')
                latest_rudder_data = None  # Clear processed data

        # Update PPM signal
        update_ppm()
        time.sleep(0.001)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    # Signal threads to stop
    stick_hid.close()
    rudder_hid.close()
    
    # Wait for threads to finish
    stick_thread.join(timeout=1)
    rudder_thread.join(timeout=1)
    
    ppm.cancel()
    ppm_input_cb.cancel()
    pi.stop()