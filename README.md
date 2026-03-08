Semi-custom telemetry system for Greenpower student car. It uses the Echook made by Greenpower with a custom drop-in replacement for the stock BT module based on the ESP32-C3, which takes sensor data from the Echook’s onboard Arduino, packages it, and sends it wirelessly through ESP-Now to a custom ESP32-S3-based handheld with a 4.0" LCD screen.

Right now, only 1-way communication is set up, with the Echook as the transmitter and the handheld as the receiver, but in the future I want to add 2-way communication so I can control PWM outputs and a camera gimbal.
