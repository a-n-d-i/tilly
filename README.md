# Introduction
Meet Tilly, the silly Tillerpilot. 
A study in using Ardupilot for stuff which it is not made for.

This is just me horsing around, so don't use this. 
And don't expect it to work. 

At this stage, it's entertainment, not a product...

In case you want to join in on the fun, have at it. I'd love some company in this endeavor!

# What is a tillerpilot?
It's a device for sailboats that controls the tiller/steering to point the boat in a direction. 
This is not to be confused with autonomous sailing. Tillerpilots have been around since 1973,
and today, entry level devices still use the same technology (fluxgate compass, some dampening,
dc motor drive / actuator).

# Why?
Holding a course in bumpy seas it not easy. Humans are quite good at feeling sutle changes in movement and speed
and can do this quite good with training and experience.
For computers, this is a challenge. So affordable Tillerpilots mostly suck in heavy seas and expensive ones are 
really expensive (like 5-50k€). And this pricepoint is not because of certifications (there are none for pleasure boats), 
it's because of rich people racing expensive boats.

Im my mind Ardupilot seems to have all the bits and bops to make a decent Tillerpilot on a very tight budget.
So I wanna find out. 

# Hardware
The brain box is mounted belowdecks in a dry location with as little electromagnetic noise as possible.
The button box lives in the cockpit. They are both connected with a cable / TTL Level Serial. 
## Brain Box
* A cheap Flight Controller (Speedybee F405 Wing mini)
* A GPS/Compass Unit (Ublox Neo-8M and ?)
* A battery pack (mostly for testing but also very convenient for black/brownouts at sea)
* A Motor Driver for the Actuator which moves the Tiller

connect button box to rx5/tx5, 115k
connect gps to tx3/rx3, 230k
connect compass to SDA/SCL

## Button Box
* Display Board for displaying data. 
* Wifi Functionality for remote control/programming
* Buttons 

# Software
## Useful commands
sim_vehicle.py -v Rover -f sailboat -L ANDI -A  "--serial6=uart:/dev/ttyUSB1" --console --map

Sim parameters like wind and wave
https://ardupilot.org/rover/docs/parameters.html#parameters-sim

# Noteworthy Parameters
https://ardupilot.org/rover/docs/rover-tuning-steering-rate.html#rover-tuning-steering-rate
https://ardupilot.org/rover/docs/rover-tuning-navigation.html#rover-tuning-navigation


Serial Message sending frequencies
https://ardupilot.org/rover/docs/parameters-Rover-stable-V4.5.2.html#sr6-parameters


Steering
* Gains
  * ATC_STR_RAT_FF
  * ATC_STR_RAT_P
  * ATC_STR_RAT_I
  * ATC_STR_RAT_D
* Rates
  * ACRO_TURN_RATE
  *  ATC_STR_RAT_MAX


ATC_STR_RAT_F* Filter!
ATC_STR_RAT_NTF: Steering control Target notch filter index

# TODO
* COMPASS_OFFS_MAX and ARMING_MAGTHRESH settings review
* declutter serial/udp data
* Only works with GPS reception, should fall back on compass / DCM
  * probably needs new mode in ardupilot, see mode_guided_nogps for copter for example.
  * so we'll make our own mode(s) eventually, I suppose
* Get simulatin on hardware going for actuator tuning
* Get log replay going for actuator tuning
* publish parameter list for sim and speedybee
* publish cad models for housings
* switch to new esp board
* Tune ESC/Motor drive
* build actuator with feedback control
* switch from speed based steering to position based
* add some error handling, at least for mode switch and arm
* try simplefoc for brushed motor (and brushless)