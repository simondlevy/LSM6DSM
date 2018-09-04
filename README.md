This repository contains Arduino, Raspberry Pi (WiringPi), and Linux i2cdev
libraries and examples for working with the Invensense LSM6DSM Intertial Measurement Unit.
The class library and examples were adapted from Kris Winer's
[repository](https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB).

To use this library you will also need our cross-platform support 
[library](https://github.com/simondlevy/CrossPlatformDataBus).

We have tested this EM7180 library on the following hardware:

* Ladybug STM32L4 board from Tlera Corp

* Teensy 3.2, 3.6

* Raspberry Pi 3

* NVIDIA Jetson TX1

RaspberryPi users should download and install [WiringPi](http://wiringpi.com/),
then cd to <b>LSM6DSM/extras/wiringpi</b>, and run <b>make</b>
to build the examples.  You may have to run the examples as root; e.g., <tt>sudo ./GetInfo</tt>.

Users of NVIDIA Jetson and other Linux-based boards should install I<sup>2</sup>C support by running the command:
<pre>
  sudo apt-get install libi2c-dev i2c-tools
</pre>
You can then can cd to <b>LSM6DSM/extras/i2cdev</b>, and run
<b>make</b>. You may have to run the examples as root; e.g., <tt>sudo ./GetInfo</tt>.

