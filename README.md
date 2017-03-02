# hamilton-3C-qualified-fw
The factory image for hamilton-7C/3C.

This code will work out the box on the hamilton-7C (with PIR) and hamilton-3C.

It senses every SAMPLE_INTERVAL microseconds, encrypts the sensor data under the
symmetric key flashed into the device with pflash, and sends it.

## Average power

The power consumption of a 7C with this firmware has been measured at

 INTERVAL  | POWER CONSUMPTION | IDEAL BATTERY LIFE  |
 --------- | ----------------- | ------------------- |
  10 s     | 37 uA             | 4.6 years           |
  20 s     | 21 uA             | 8.2 years           |
  30 s     | 17 uA             | 10.0 years          |
  NEVER    |  9 uA             | 19.0 years          |

The ideal battery life is assuming a 1500mAh battery with no self-discharge. In
real life, results will vary.

These results are taken in an unoccupied room. The hamilton-7C consumes on average
75uA if the PIR sensor is continuously triggered while the firmware is configured for
a 30 second reporting interval. This would equate to about 2.3 years.

These average power figures are very accurate for a 25C environment.

## Power profile

The power consumption profile looks like the following figure. Due to our measurement technique, the Y axis is only an approximation of the instantaneous power consumption. The average power figures above are far more accurate.

![Full Profile](doc/full_cycle.png)

There is a period of approximately 22ms in the beginning where the device senses air temperature and humidity.

![First sensing](doc/sense_start.png)

Then there is a long period of about 500ms where it senses radiant temperature (the power consumption in this period is about 250uA).
Then it senses illumination, accelerometer and magnetometer, as well as aggregating PIR events and button events.
It then encrypts all this data and transmits it. This second part looks as follows:

![Second sensing and send](doc/send.png)

If the PIR sensor is triggering, there will be periodic events that look as follows:

![PIR trigger event](doc/pir_trigger.png)

These are basically the chip waking up and recording the timestamp of the PIR event for later aggregation.
