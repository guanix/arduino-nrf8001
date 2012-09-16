# nRF8001 support for Arduino

This is a work in progress. Most of the functionality of the nRF8001 chip
is implemented, but not all commands and interactions have been fully
tested.

## Installation

On a Mac:

    cd ~/Documents/Arduino/libraries
    git clone https://github.com/guanix/arduino-nrf8001.git nRF8001

The included example works with the Heart Rate function of the Nordic iOS app
and sends the nRF8001’s internal temperature reading as heart rate and the
fixed number 78 as battery level. It assumes the following pin assignments,
easily changed:

* RESET on digital pin 5
* REQN on digital pin 6
* RDYN on digital pin 7

This library uses the AVR chip’s hardware SPI support, so the SCK (CLK),
MISO and MOSI pins cannot be changed:

* MOSI on digital pin 11
* MISO on digital pin 12
* SCK on digital pin 13

Remember that nRF8001 expects 3.3V supply and logic levels. Connecting it
to a typical Arduino will fry it. You must either use a level shifter/buffer,
or a [3.3V Arduino][promini], or a clone such as [Seeeduino][seeeduino]
that can switch between 3.3V and 5V modes.

[promini]: https://www.sparkfun.com/products/11114
[seeeduino]: http://www.seeedstudio.com/depot/seeeduino-v221-atmega-328p-p-669.html

## Technical Overview

nRF8001 is a slave-only Bluetooth Low Energy (Bluetooth 4.0) transceiver.
It requires an external antenna and matching components, but little else.
At least one vendor markets a complete module incorporating the antenna,
which only requires a decoupling cap and a ground plane.

Communication with the nRF8001 uses a custom binary SPI protocol, but in
addition to the typical SS pin to send a command (called REQN here), there
is an additional pin, RDYN, which has two purposes. If the master (your
Arduino or other microcontroller) initiates a command, nRF8001 uses RDYN
to indicate that it is ready to receive a command. nRF8001 can also
initiate a transmission, for example when it has received data from a
connected device, in which case it brings RDYN low to indicate that it
wants to transmit, and the master brings REQN low to indicate that the
transmission can proceed.

All commands, responses to commands and other events are sent asynchronously.
You will receive a response, but it may come several transactions later. The
nRF8001 can also send events or responses to previous commands _while_ you
are transmitting a command. For this reason, while the Arduino nRF8001 library
contains a number of functions to send commands to the nRF8001, those
functions almost never immediately return a response. Instead, you must
register _handlers_, special functions that you define, which will in turn
be called when the response (or an event) is received. A typical simple
Arduino sketch that uses nRF8001 will use global variables to coordinate
between handlers and the main event loop.

This library is not interrupt driven. Responses from the nRF8001 may be
received whenever you send a command, but if you are not ready to send a
command immediately, you must call `nRF8001::poll()` (or its variant with
a timeout) to poll for responses or events.

In order to define services, you must use Nordic’s nRFgo Studio software.
The software will generate a `services.h` file that you must place in the
library’s folder inside your Arduino installation’s `libraries` folder.
The software will define a number of _pipes_. The included `services.h` is
Nordic’s heart rate monitor example, which allows the slave to send
heart rate information on pipe 5 and battery level information on pipe 8.
The exact format of the data messages are defined in the relevant
[service specification][servicespec].

[servicespec]: http://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx

## The heart rate example

The included heart rate example defines two event handlers,
`temperatureHandler` for receiving temperature from the nRF8001’s
built-in thermometer and stores it in a global variable,
and the catch-all `eventHandler`, which only calls `nRF8001::debugEvent`,
a function that prints extensive information about received events to the
serial port.

After initializing the `nRF8001` class, registering handlers and calling
`nRF8001::setup` to send setup messages from `services.h` to the nRF8001
chip, it then calls `nRF8001::getDeviceAddress()` to get the device
address and `nRF8001::getTemperature()` to read the temperature.
It then calls `nRF8001::connect` to wait for a connection from a peer
device.

In the loop, we continuously poll with `nRF8001::poll` using a 2-second
timeout, and every 2 seconds or so, we read a new temperature and send
that out as a heart rate. If the connection is disconnected, we call
`nRF8001::connect` again to wait for a new connection.

## License
Copyright © 2012 Guan Yang

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
“Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
