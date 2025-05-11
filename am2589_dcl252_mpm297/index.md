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

Rationale, math, logical structure, etc.

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
