# home-iot
Various sketches and stuff for my private IoT-sensors. Everything in this repo
is mainly for use with the eminent software
[Home Assistant](http://home-assistant.io). Amount of documentation will vary...

# esp8266
This directory contains (Arduino) sketches for the ESP8266 platform.

## GaragePort

Simple sketch used to control a garage port via a relay. A push button is used
to sense if the port is open or closed. If a temperature sensor is attached
(e.g. DHT11), the temperature and humidity is available as well. The
RJ45-jacket connects to the relay, so only two of the pins are used.

![Alt text](/esp8266/GaragePort/sensor.jpg?raw=true "Prototype sensor")

For communication, MQTT is used as protocol. Temperature, humidity and garage
port state is published at:

*home-assistant/garage/port/state*

*home-assistant/garage/temperature*

*home-assistant/garage/humidity*

and the port can be opened/closed using:

*/home-assistant/garage/port/set*

See the code for reference.

A small log of what is going on is saved and can be accessed at the built in
web server page: *http://<ip>/log*.

### Configuration

Look for the "Config" comment in the code and update constants as needed.