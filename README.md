# dcc-module
Universal DCC breakout board. Provides power management and converts DCC channel to UART.

## Premise

Digital Command Control (DCC) is a communications and power delivery standard for automating (electric) model trains. A general purpose breakout that provides power handling and a UART channel should provide greater hobbyist flexibility than a series of purpose-built PCBAs. Find the United States working group [here](https://www.nmra.org/dcc-working-group). 

## TODO
- [ ] finish debugging
- [ ] doxygen
- [ ] gh actions testing
- [ ] hardware BOM
  - [ ] select MCU
  - [ ] select attenuator NMOS
  - [ ] select buck converter
  - [ ] select ideal diode controller
- [ ] test edge detection on common-drain attenuator design


## Limitations
- This project provides no guarantee of correctness. Proceed at your own risk.
- It is unlikely this project will implement the extended DCC protocol.

## Parts of the Design

### Power Path
DCC provides square wave AC power. Full wave rectifier -> buck converter + filters. The power output is planned to be able to deliver 5V 2A. Please please please use an LDO between the power output and your micro.

### Signal Path
Bits are defined by signal edges. A half-wave rectifier -> common-drain attenuator should be sufficient for an MCU.

### Comms Handling
Edge-triggered interrupt + some kind of circular buffer with packet validation and DMA UART.
