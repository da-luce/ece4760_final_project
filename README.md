# ECE 4760 Final Project

Currently working in PWM_Demo.

## VGA + Motor Control

- VGA uses `pio0`
  - Three programs: `rgb.pio`, `hysync.pio`, and `vsync.pio`
- Control of a single motor requires three `pio` programs:
  - `pacer.pio`
  - `stepper.pio`
  - `counter.pio`

Since we only have two PIO blocks (`pio0` and `pio1`), with four state machines
for each, **all motor PIO programs must go on `pio1`**.

## Important Links

- [Course Webpage](https://ece4760.github.io/)
- [Lab Page](https://vanhunteradams.com/Pico/Helicopter/Helicopter.html)
- [Pico Pins](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf#%5B%7B%22num%22%3A6%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C115%2C841.89%2Cnull%5D)
- [VGA Pins](https://vanhunteradams.com/Pico/VGA/VGA.html#Attaching-hardware)
- [Debug Pins](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html)
