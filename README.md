# FPV Glider Drone

A little homemade project to learn how to build a FPV glider drone, with head tracking, return to home and autonomous flight capabilities like loitering.
Using my software skills, i managed to build decent true control system with a flightstick and a rudder, for a 100% immersive experience.

![DSC00611](https://github.com/user-attachments/assets/97567429-8ae6-410e-bb4f-380805c984bf)

## Personal experience in RC flight

I have been flying RC since i was young (small copters and planes). I joined a club in early 2024 and practiced flying (LoS) on an Aero scout, built a balsa wood plane, and a depron pusher prop plane. I did not have experience with soldering. I'm a developer so the programming part was an issue for me.

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
