# nRF8001 support for Arduino

This is very much a work in progress. The current code base is designed for
interrupt driven operation, but because of the difficulty of debugging that
code and because non-interrupt-driven SPI is actually faster, I am currently
in the process of rewriting to remove interrupt handling. This will also make
the code more flexible because any pin can be used for RDYN and REQN.
