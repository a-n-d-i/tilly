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
## Button Box
* Display Board for displaying data. 
* Wifi Functionality for remote control/programming
* Buttons 