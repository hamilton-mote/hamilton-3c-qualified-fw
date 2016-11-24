# hamilton-3C-qualified-fw
The factory image for hamilton-7C, 3C and 5C.

At present this is not sampling the magnetometer/accelerometer, and the radiant sensor is
disabled, so the sensor pack is:

- 0x04 - light sensor
- 0x10 - temperature and humidity sensor
- 0x40 - PIR sensor

We hope to update to having the magnetometer soon.
