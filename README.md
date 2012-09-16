# nRF8001 support for Arduino

This is very much a work in progress. The current code base is designed for
interrupt driven operation, but because of the difficulty of debugging that
code and because non-interrupt-driven SPI is actually faster, I am currently
in the process of rewriting to remove interrupt handling. This will also make
the code more flexible because any pin can be used for RDYN and REQN.

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
