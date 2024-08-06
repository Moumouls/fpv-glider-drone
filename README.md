# FPV Glider Drone

A little homemade project to learn how to build a FPV glider drone, with head tracking, return to home and autonomous flight capabilities like loitering.
Using my software skills, i managed to build decent true control system with a flightstick and a rudder, for a 100% immersive experience.

![DSC00611](https://github.com/user-attachments/assets/97567429-8ae6-410e-bb4f-380805c984bf)

## Personal experience in RC flight

I have been flying RC since i was young (small copters and planes). I joined a club in early 2024 and practiced flying (LoS) on an Aero scout, built a balsa wood plane, and a depron pusher prop plane. I did not have experience with soldering. I'm a developer so the programming/soft configuration part was not an issue for me.

## Airframe

As a first FPV plane, i wanted something slow to fly, cheap, and i know i wanted a glider to play with thermals and winds. I know that the camera system, avionics and the flight controller are quite heavy, and take some space in the cockpit, so i needed a relatively large airframe.
I found that my best option was the Volantex ASW-28 V2 with a wing span of 2.6m.

- [Volantex ASW-28 V2 from HK](https://hobbyking.com/fr_fr/volantex-759-1-asw28-v2-electric-sailplane-epo-plastic-2540mm-pnf.html)
  - Price: 140-180€

## FPV System

I wanted a good FPV system, i was ready to spend money on it. Obviously i opted for the DJI O3 Air Unit with DJI Goggle V2.

- [DJI O3 Air Unit](https://www.dji.com/fr/o3-air-unit)
  - Price: 249€
- [DJI Goggle V2](https://www.dji.com/fr/goggles-2)
  - Price: 599€

I was also ready to go for a true immersive FPV system, so i ordered a Gimbal for the camera and a prebuilt system for the head tracking system. Motion Sic headtracker is "just" a prebuilt Arduino based on the open source project [HeadTracker](https://headtracker.gitbook.io/head-tracker-v2.2).

- [Gimbal Motion Sic](https://fpvdogfight.com/products/motionsic-b-a-g-badass-gimbal)
  - Price: 83€
- [Head Tracking System Motion Sic](https://fpvdogfight.com/products/tally-ho-2-prebuilt-head-tracker)
  - Price: 101€

## Flight Controller

I chose the Matek H743 WLITE because it's compatible with INAV and also Ardupilot. This flight controller is dedicated to fixed wing drones with extensive connectors.
This card have also a 9V/2A programmable switch needed for the O3 Air Unit, because the O3 Air Unit will burn if powered on on the ground without flying, more en that later.

- [Matek H743 WLITE](https://www.mateksys.com/?portfolio=h743-wlite)
  - Price: 119,90€

![H743-WLITE-Wiring](https://github.com/user-attachments/assets/1e830b71-74ff-4799-8e77-50bc51f598a9)

## Sensors

The barometer, accelerometer and gyroscope are included in the Matek H743 WLITE, i added a GPS with Compass and a pitot tube for air speed.

- [GPS with Compass M10Q-5883](https://www.mateksys.com/?portfolio=m10q-5883)
  - Price: 44,90€
- [Pitot ASPD 4525](https://www.mateksys.com/?portfolio=aspd-4525)
  - Price: 55,90€

## Battery

Recommended battery for the Volantex is a 3S 2200mAh battery, without thermal you can fly almost 15-20min. Space is critical in the cockpit, so i choose a compact and light battery. Like the OVONIC Air 3S 2200mAh

- [OVONIC Air 3S 2200mAh XT60](https://www.ovonicshop.com/products/ovonic-3s-2200mah-35c-11-1v-short-lipo-battery-pack-for-air-heli-drone-xt60-plug?_pos=4&_sid=42bfc1070&_ss=r)

## Aircraft Beacon System

I fly in France, and the drone will weight more than 800g, so the aircraft need to be registered and have a beacon to transmit the position of the drone and his altitude. Dronavia is a relatively cheap option, it means that i've 2 GPS onboarded on the drone, but i'm not sure if its possible to use INAV/Ardupilot with a WIFI beacon to transmit the position without needing a dedicated beacon.

- [Dronavio Zephyr Beacon AM](https://www.dronavia.com/product/zephyr-beacon-am/)
  - Price: 40€

## Transmitter/Receiver

I love open source projects, so i opted out for ExpressLRS technology with RadioMaster hardware (TX16S transmitter and ER6 PWM receiver). PWM receiver is quite useless since all PWM outputs will be on the Matek, but let's go for it.

- [RadioMaster TX16S](https://www.radiomasterrc.com/products/tx16s-mark-ii-radio-controller)
- [RadioMaster ER6](https://www.radiomasterrc.com/products/er6-2-4ghz-elrs-pwm-receiver)

## FlightStick and Rudder Controller

Building an FPV aircraft is nice, with a head tracking system it was a step forward into the immersive experience, but i wanted the "ultimate" experience with a flightstick and a rudder. The solution was to use a Raspberry Pi 3 (or 4) with a soldered input/ouput jack 3.5mm, and some dedicated code to read the joystick, merge an input PPM signal (from the headtracker) and send a PPM signal to the TX16S transmitter.
Note: I don't bought the flightstick and rudder specifically for this project, i bought them some year ago for flying simulations.

- [Raspberry Pi 3 or 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)
  - Price: 35€-50€
- [Velocity One Flightstick](https://fr.turtlebeach.com/products/velocity-one-flight-stick)
  - Price: 129€
- [Velocity One Rudder](https://fr.turtlebeach.com/products/velocity-one-rudder)
  - Price: 299€

## Software

### INAV (Flight Controller Software)

After some research for Fixed Wing drones or fixed wing assisted flight INAV is the best option.

- [INAV](https://github.com/iNavFlight/inav)

### ExpressLRS (Radio Software)

Open source and super performant, in FPV you need minimum latency and long range communication, ELRS is perfect for that.

- [ExpressLRS](https://www.expresslrs.org/)

### HeadTracker (Head Tracking Software)

Open source with great documentation to add head tracking to the drone.

- [HeadTracker](https://headtracker.gitbook.io/head-tracker-v2.2)

### PIGPIO

An open source library to control precisely the GPIO pins of the Raspberry Pi.
Needed to read and generate an accurate PPM signal.

- [PIGPIO](https://abyz.me.uk/rpi/pigpio/)

## Build

### Airframe

The Volantex ASW-28 is easy to build but need some modifications to be even better.

- Be carful on the servo pushrod of the elevator, connect the pushrod before connecting the full elevator to the tail.
- You need a cutter or small drill to enlarge the hole of wing servos in the fuselage, they are too small and connecting/disconnecting them is a nightmare without this modification.
- The V2 introduced a landing gear with a internal foam protection, this system is not aerodynamic, it's a little bit heavy, and takes up space at a critical point where the centre of gravity is located. So i removed the landing gear and added a depron piece to fill the landing gear hole. I removed the useless internal foam protection, because on my setup the battery will be located where the landing gear was.
- I don't liked the red wing tips, red nose, and black propeller so i painted all of them in white.
- The canopy have a clip system too fragile, i added small piece of a pvc white tape to avoid to break the clip system.

![20240710_232059](https://github.com/user-attachments/assets/e37ea1f3-94ea-47e6-bea7-c5a28ff3f0a8)

### FPV Camera

The FPV camera have some constraint, the camera need to be mounted OUTSIDE of the airframe because the canopy is not transparent enough. It's also required because the CPU unit of the O3 need to be located on the fuselage to avoid overheating during flight (i tested to use a small fan, the airflow is not sufficient to cool the O3 unit). The solution was to cut a hole into the canopy and mount the camera at the back of the canopy, with the O3 unit scratched on the fuselage just behind the camera. This kind of mounting is quite nice in flight, the view on top of the canopy is very good.

![20240712_192606](https://github.com/user-attachments/assets/cab5191d-76d1-4d9a-bf03-2d38f589382f)
![DSC00612](https://github.com/user-attachments/assets/6b1a8ddc-62fa-46d1-bf54-18279160396d)

### Flight Controller Assembly

Matek manual is clear, you just need to be patient, and solder correctly. Soldering the Airspeed sensor was not needed thanks to the JST connectors. But it needed to solder the:

- Servo Pins (Ailerons, Elevator, Rudder, Pitch Camera, Pan Camera, Flaps, Beacon)
- Receiver cables
- Battery connector (XT60) (with the capacitor)
- O3 Air Unit cables
- ESC cables

You have around 30 points to solder. You need to be patient and careful, because a pin failure or short circuit can be the end of the drone in flight.
(It's my biggest fear)

The FC is placed under the canopy, scratched on some soft foam to reduce gyro noise.

![DSC00614](https://github.com/user-attachments/assets/83b4b19f-0415-44f4-bbfb-7e342774742c)

### Sensors placement

- The GPS+Compass is placed far from the electric motor to avoid magnetic interference.
- The Pitot ASPD is placed on the canopy, i made the trade to have wrong data on the airspeed during motor use, it's okay for me because this aircraft is not meant to fly autonomously and it's a glider so i only the motor during climb at full power.
- The Beacon is placed next to the FC, it just need to have a clear sky view.
- The receiver is placed at the rear of the canopy so the receiver antenna is far enough from the FPV antenna.

### Flightstick and Rudder Controller

- Solder the male jack on a 3.3V pin
- Solder the female jack on a 3.3V pin
- Install python3 on the Raspberry Pi
- Install the PIGPIO library
- Run the python3 script and pigpio deamon automatically at startup

Here the the code used to read the joystick and rudder in separate threads to avoid blocking the main thread, and avoid huge latency of controls.

```python
import hid
import time
import pigpio

class X:

   GAP=300
   WAVES=3

   def __init__(self, pi, gpio, channels=8, frame_ms=27):
      self.pi = pi
      self.gpio = gpio

      if frame_ms < 5:
         frame_ms = 5
         channels = 2
      elif frame_ms > 100:
         frame_ms = 100

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

# Constants
MAX_STICK_VALUE = 65535
MIDDLE_STICK_VALUE = 32767

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Failed to connect to pigpio daemon")

# PPM configuration
PPM_GPIO = 4  # GPIO pin for PPM output
FRAME_MS = 20  # Frame duration in milliseconds
CHANNEL_COUNT = 6  # Number of PPM channels

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
}

def map_range(value, from_min, from_max, to_min, to_max):
    return int((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min)

def update_ppm():
    channels = [
        map_range(control_state['aileron'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['elevator'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['motor'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['rudder'], 0, MAX_STICK_VALUE, 1000, 2000),
        map_range(control_state['flap'], 0, MAX_STICK_VALUE, 1000, 2000),
    ]

    ppm.update_channels(channels)

print('Listening for data and generating PPM signal')
update_ppm()

try:
    while True:
        # Read from stick
        stick_data = stick_hid.read(64, timeout_ms=10)
        if stick_data:
            control_state['elevator'] = int.from_bytes(stick_data[3:5], byteorder='little')
            control_state['aileron'] = int.from_bytes(stick_data[1:3], byteorder='little')
            control_state['motor'] = int.from_bytes(stick_data[11:13], byteorder='little')
            control_state['flap'] = int.from_bytes(stick_data[13:15], byteorder='little')

        # Read from rudder
        rudder_data = rudder_hid.read(64, timeout_ms=10)
        if rudder_data:
            control_state['rudder'] = int.from_bytes(rudder_data[5:7], byteorder='little')

        # Update PPM signal
        update_ppm()

        # Small delay to prevent excessive CPU usage
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ppm.cancel()
    pi.stop()
    stick_hid.close()
    rudder_hid.close()
```

```sh
# Short script to run at Pi startup
sudo pigpiod
sleep 5
sudo python3 /home/{{username}}/project/script.py
```

![20240717_203719](https://github.com/user-attachments/assets/3e0302c3-cd5a-4b19-8e92-2675afe27b29)

## INAV

Here the INAV configuration file, it's important on the Matek H743 WLITE, to set `vbat_scale` to `2100` and `current_meter_scale` to `150` for correct battery voltage and current reading.

```sh

# diff all

# version

# INAV/MATEKH743 7.1.2 Jun 1 2024 / 00:04:49 (4e1e59eb)

# GCC-10.3.1 20210824 (release)

# start the command batch

batch start

# reset configuration to default settings

defaults noreboot

# resources

# Timer overrides

# Outputs [servo]

# safehome

# Fixed Wing Approach

# features

feature -TX_PROF_SEL
feature -BLACKBOX
feature GPS
feature PWM_OUTPUT_ENABLE
feature FW_AUTOTRIM

# beeper

# blackbox

blackbox -NAV_ACC
blackbox NAV_POS
blackbox NAV_PID
blackbox MAG
blackbox ACC
blackbox ATTI
blackbox RC_DATA
blackbox RC_COMMAND
blackbox MOTORS
blackbox -GYRO_RAW
blackbox -PEAKS_R
blackbox -PEAKS_P
blackbox -PEAKS_Y

# Receiver: Channel map

# Ports

serial 1 2 115200 115200 0 115200
serial 7 33554432 115200 115200 0 115200

# LEDs

# LED color

# LED mode_color

# Modes [aux]

aux 0 0 0 1300 2100
aux 1 12 6 1200 2100
aux 2 11 7 1300 1700
aux 3 10 7 1750 2100
aux 4 21 4 1300 1975
aux 5 47 5 900 1250

# Adjustments [adjrange]

# Receiver rxrange

# temp_sensor

# Mission Control Waypoints [wp]

#wp 0 invalid

# OSD [osd_layout]

osd_layout 0 0 44 0 V
osd_layout 0 1 12 0 H
osd_layout 0 3 8 6 V
osd_layout 0 7 6 0 V
osd_layout 0 9 1 2 H
osd_layout 0 11 2 3 H
osd_layout 0 12 1 4 H
osd_layout 0 14 25 0 V
osd_layout 0 15 19 0 V
osd_layout 0 22 43 2 V
osd_layout 0 25 39 6 V
osd_layout 0 26 41 8 V
osd_layout 0 27 41 10 V
osd_layout 0 28 30 0 V
osd_layout 0 30 13 17 V
osd_layout 0 34 22 2 V
osd_layout 0 38 38 0 V
osd_layout 0 97 1 18 V

# Programming: logic

# Programming: global variables

# Programming: PID controllers

# OSD: custom elements

# master

set gyro_zero_x = 9
set gyro_zero_y = -11
set gyro_zero_z = -4
set ins_gravity_cmss = 1011.557
set acc_hardware = ICM42605
set acczero_x = 33
set acczero_y = 68
set acczero_z = -9
set accgain_x = 4349
set accgain_y = 4204
set accgain_z = 4083
set align_mag = CW270FLIP
set mag_hardware = QMC5883
set magzero_x = -110
set magzero_y = 81
set magzero_z = -120
set maggain_x = 1238
set maggain_y = 1257
set maggain_z = 1220
set baro_hardware = SPL06
set pitot_hardware = MS4525
set motor_pwm_protocol = STANDARD
set failsafe_procedure = RTH
set vbat_scale = 2100
set current_meter_scale = 150
set applied_defaults = 1
set gps_ublox_use_galileo = ON
set gps_ublox_use_beidou = ON
set gps_ublox_use_glonass = ON
set osd_video_system = BFHDCOMPAT
set osd_switch_indicators_align_left = OFF

# mixer_profile

mixer_profile 1

set platform_type = AIRPLANE
set has_flaps = ON
set model_preview_type = 14
set motorstop_on_low = ON

# Mixer: motor mixer

mmix reset

mmix 0 1.000 0.000 0.000 0.000

# Mixer: servo mixer

smix reset

smix 0 1 1 100 0 -1
smix 1 2 0 100 0 -1
smix 2 3 2 -100 0 -1
smix 3 4 9 100 0 -1
smix 4 5 10 100 0 -1
smix 5 6 11 100 0 -1

# mixer_profile

mixer_profile 2

# Mixer: motor mixer

# Mixer: servo mixer

# profile

profile 1

# profile

profile 2

# profile

profile 3

set fw_p_pitch = 30
set fw_p_roll = 30
set fw_p_yaw = 30

# battery_profile

battery_profile 1

set throttle_idle = 10.000

# battery_profile

battery_profile 2

# battery_profile

battery_profile 3

# restore original profile selection

mixer_profile 1
profile 3
battery_profile 1

# save configuration

save
```

## First flight (LoS)

The first flight need optimal conditions, no wind, no turbulence, and no obstacles. The first flight also is not FPV, i needed 3-4 flight to first understand the behavior of the aircraft at low speed, sensitivity of the controls

## First FPV flight

![IMG-20240726-WA0003](https://github.com/user-attachments/assets/b2eaa9da-559c-41e0-8d5e-4833f1001cfe)
