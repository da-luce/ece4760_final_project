---
title: "PicoScope Project"
---

![Test Image](test.png)

The project involves building a 2D lidar system using a
[Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf),
[Adafruit VL53L4CX ToF sensor](https://www.adafruit.com/product/5425), and
[28BYJ-48 stepper motor](https://www.mouser.com/datasheet/2/758/stepd-01-data-sheet-1143075.pdf) with a
[ULN2003 Driver](https://www.hadex.cz/spec/m513.pdf)
to capture and process distance measurements for mapping and navigation applications.

---

## Project Introduction

Provide a detailed summary of what you did and why.

---

## High-Level Design

The purpose of this project was to construct a 2-Dimensional LiDAR capable of scanning distances up to approximately 3 meters in distance. The high-level design of the project consisted of a few key components: the mechanical structure upon which the Time-of-Flight sensor was mounted, the software logic for interfacing with a stepper motor for rotating the Time-of-Flight Sensor, Arduino code and UART communication for extracting readings from the sensor, and graphics for visualizing measurements. Additionally, various mechanisms were implemented to allow for sensor calibration, including an optical-interrupter used to provide a reference point for the angle of the stepper motor. 

Background on Time-of-Flight Sensor:

Generally, Time-of-Flight sensors can measure surrounding terrain by emitting photons and sensing the duration of time before photons return back to the sensor. Note that from this point forward Time-of-Flight will be abbreviated with the acronym ToF. 
The ToF sensor utilized in the lab employed a wavelength of 940nm, indicating the use of infrared radiation. Infrared radiation is often used for such applications as it is “invisible” and can reduce interference from external light sources. In fact, infrared light is less susceptible to Rayleigh scattering, a well-known phenomenon where small atmospheric particles cause light to scatter. The intensity of Rayleigh scattering is inversely proportional to the wavelength of the scattered light raised to the power of 4:

Intensity of Scattered Light ~ 1/(4)

Thus, infrared light, specifically at a wavelength of 940nm, is a good choice for a ToF sensor. One can roughly estimate the distance of an object by employing a simplified formula based on the speed of light:

 Distance (t*c/2), 

where the variable t corresponds to the duration of time for a photon to return back to the sensor (time-of-flight). 

Of course, environmental factors can interfere with ToF measurements - aside from light scattering, ambient light sources can emit additional photons that can often interfere with the sensor’s ability to detect surrounding objects. This may explain the phenomenon observed where weaker signals were derived from objects farther away from the sensor. In other words, farther objects increase the chances of environmental interference.


---

## Program/Hardware Design

Discuss tricky parts, hardware, and software choices.

### Graphics

Arnav, you probably know this the best.

One of the challenges we encountered while rendering the data was dealing with volatile variables. A particularly tricky issue arose in the code snippet below:

```c
drawTrianglePointerOutline((int) RAD2DEG(current_angle), max_mm * PX_PER_MM, BLACK);
drawTrianglePointerOutline((int) RAD2DEG(current_angle), max_mm * PX_PER_MM, WHITE);
```

These lines are responsible for drawing the triangle which indicates the current LiDAR angle. At first glance, this code appears fine. However, we observed frequent flickering—sometimes the triangle wasn't fully erased, leading to inconsistent visual behavior. The randomness of this issue suggested that interrupts were affecting the value of current_angle between the two drawing calls.

The root cause was that current_angle is a volatile variable that may be modified by an interrupt at any time. As a result, the value could change between the two calls to drawTrianglePointerOutline(), causing the triangle to be drawn and erased at different angles. To solve this, we cached the computed value before using it, ensuring consistency:

```c
float angle_deg = RAD2DEG(current_angle);
drawTrianglePointerOutline((int) angle_deg, max_mm * PX_PER_MM, BLACK);
drawTrianglePointerOutline((int) angle_deg, max_mm * PX_PER_MM, WHITE);
```

By storing the result of current_angle in a temporary variable, we guaranteed that both calls used the exact same angle, eliminating flicker and improving rendering stability.
Hardware Schematic

Add schematic here!

## Results of the Design

Data, results, scope traces, etc.

## Conclusions

Analyze your results and discuss improvements.

## Appendix A

The group does not approve this report for inclusion on the course website.

The group does not approve the video for inclusion on the course YouTube channel.

© 2025 Mac Marsh (mpm297) | Dalton Luce (dcl252) | Arnav Muthiayen (am2589) — Cornell University
