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

### Time-of-Flight Sensors

Generally, Time-of-Flight sensors can measure surrounding terrain by emitting photons and sensing the duration of time before photons return back to the sensor. Note that from this point forward Time-of-Flight will be abbreviated with the acronym ToF. 
The ToF sensor utilized in the lab employed a wavelength of 940nm, indicating the use of infrared radiation. Infrared radiation is often used for such applications as it is “invisible” and can reduce interference from external light sources. In fact, infrared light is less susceptible to Rayleigh scattering, a well-known phenomenon where small atmospheric particles cause light to scatter. The intensity of Rayleigh scattering is inversely proportional to the wavelength of the scattered light raised to the power of 4:

$$
\text{Intensity of Scattered Light} \propto \frac{1}{\lambda^4}
$$

Thus, infrared light—specifically at a wavelength of 940 nm—is a suitable choice for a ToF sensor. One can estimate the distance to an object using a simplified formula based on the speed of light:

$$
\text{Distance} \approx \frac{t \cdot c}{2}
$$

where $t$ is the time it takes for a photon to travel to the object and back (time-of-flight), and $c$ is the speed of light.

Of course, environmental factors can interfere with ToF measurements - aside from light scattering, ambient light sources can emit additional photons that can often interfere with the sensor’s ability to detect surrounding objects. This may explain the phenomenon observed where weaker signals were derived from objects farther away from the sensor. In other words, farther objects increase the chances of environmental interference.

---

## Program/Hardware Design

Discuss tricky parts, hardware, and software choices.

### Hardware Schematic

### Graphics

Arnav, you probably know this the best.

#### Volatile Variables

One of the challenges we encountered while rendering the data was dealing with volatile variables. A particularly tricky issue arose in the code snippet below:

```c
drawTrianglePointerOutline((int) RAD2DEG(current_angle), max_mm * PX_PER_MM, BLACK);
drawTrianglePointerOutline((int) RAD2DEG(current_angle), max_mm * PX_PER_MM, WHITE);
```

These lines are responsible for drawing the triangle which indicates the current LiDAR angle. At first glance, this code appears fine. However, we observed frequent flickering—sometimes the triangle wasn't fully erased, leading to inconsistent visual behavior. The randomness of this issue suggested that interrupts were affecting the value of `current_angle` between the two drawing calls.

The root cause was that `current_angle` is a volatile variable that may be modified by an interrupt at any time. As a result, the value could change between the two calls to `drawTrianglePointerOutline()`, causing the triangle to be drawn and erased at different angles. To solve this, we cached the computed value before using it, ensuring consistency:

```c
float angle_deg = RAD2DEG(current_angle);
drawTrianglePointerOutline((int) angle_deg, max_mm * PX_PER_MM, BLACK);
drawTrianglePointerOutline((int) angle_deg, max_mm * PX_PER_MM, WHITE);
```

By storing the result of `current_angle` in a temporary variable, we guaranteed that both calls used the exact same angle, eliminating flicker and improving rendering stability.

#### Color Map

Arnav, you also know this better

### Mechanical Assembly

One of the most challenging parts of this lab was the actual mechanical assembly. This biggest challenge is that we have a rotating structure--and wires and twisting do not generally mix very well. The most obvious solution
would be to use a slip ring connector, but these can be rather pricey ($25-40!).

Instead, we designed our LiDAR to only rotate 360° before reversing direction. In this fashion, we avoided overly twisting the wires, but still maintained a full 360° FOV. To build the device, we fixed a small stepper motor to a base built with LEGOs. We choose LEGOs as they were accessible, flexible, and easy to work with. Then, we fixed a four inch 8 mm shaft to the motor using a small coupler. From there, we fixed a linear mount to the shaft, and fixed the ToF sensor onto that. In order to wire the LiDAR to the Arduino Due, we used a ribbon cable with jumper wires soldered onto either end. This choice in wire was critical, as the normal solid core wires were far too stiff to permit the motion we required.

Even given this more flexible cable, things would still frequently get tied up. If we were to attempt a second prototype, we would mount the whole system on a rotating platform. This way, all sensitive electronic connections would be stationary relative to each other, and a slip ring delivering ground and power could be connected.

To zero the motor (to define a set zero angle), we mounted an optical interrupter to the LEGO base. To trigger it, we used one of the M4 threads in the coupler to fix a larger M4 bolt. On the head of the bolt, we hot glued a GPIO header to serve as a more precise way to block the [optical interrupter](https://www.rohm.com/electronics-basics/photointerrupters/what-is-a-photointerrupter). We found the header alone was not sufficient, so we place some of te PVC wire coating around the header, and this was sufficient to accurately and precisely zero the motor. Interesting, we found that the zeroing was too bouncy to start, thus, we found we needed to debounce the interrupter to get a reliable reading. To do this, we simply used the same debouncing code as the other buttons, and effectively treated the interrupter as a button.

### Generalized Button Code

Since we required multiple buttons for a design, we refactored the debouncing code of prior labs to make the code easier to reuse. We define a `Button` struct to hold all data pertinent to debouncing and firing a specific button.

```c
typedef struct 
{
    uint gpio;
    ButtonState state;
    void (*on_press)(void);     // Function on press
    void (*on_release)(void);   // Function on release
} Button;
```

We then updated the debouncing function to take one of these structures;

```c
/* Debounce button press
 */
void check_button(Button *button)
{
    bool button_reading = gpio_get(button->gpio);

    switch (button->state) {
        case NOT_PRESSED:
            if (button_reading == 0) {
                button->state = MAYBE_PRESSED;
            }
            break;

        case MAYBE_PRESSED:
            if (button_reading == 0) {
                button->state = PRESSED;

                // Press function
                if (button->on_press) {
                    button->on_press();
                }

            } else {
                button->state = NOT_PRESSED;
            }
            break;

        case PRESSED:
            if (button_reading == 1) {
                button->state = MAYBE_NOT_PRESSED;
            }
            break;

        case MAYBE_NOT_PRESSED:
            if (button_reading == 0) {
                button->state = PRESSED;
            } else {
                button->state = NOT_PRESSED;

                // Release function
                if (button->on_release) {
                    button->on_release();
                }
            }
            break;

        default:
            button->state = NOT_PRESSED;
    }
}
```

Thus, adding additional buttons became a breeze, as all we needed to do was define a struct and the `on_press` and/or the `on_release` functions. For example, here the code to treat the optical interrupter (called "zero gate" here) as a button:

```c
void zero_gate_on_block(void) {
    zeroed = true;
}
Button zero_gate = {
    .gpio = ZERO_GATE_PIN,
    .state = NOT_PRESSED,
    .on_press = zero_gate_on_block,
    .on_release = NULL,
};
```

All the buttons are then debounced in their own thread:

```c
static PT_THREAD (protothread_button(struct pt *pt))
{
    PT_BEGIN(pt) ;
    while(1) {
        PT_SEM_SIGNAL(pt, &vga_semaphore);
        check_button(&clear_button);
        check_button(&state_button);
        check_button(&zero_gate);
        check_button(&stop_button);
        PT_YIELD_usec(3000);
    }
    PT_END(pt) ;
}
```

A yield of three milliseconds proved sufficient to debounce the mechanical buttons without noticeably impacting overall system responsiveness.

### States

![State Diagram](state_diagram.drawio.png)

The program operates as a finite state machine to manage the LiDAR system's behavior based on user input. It begins in the `WAITING1` state, during which the system is idle and allows for sensor calibration (TODO: see elsewhere). If the user holds a the green state button, the system transitions to the `ZEROING` state, which is responsible for aligning the stepper motor with the zero position via the optical interrupter.

Once the zero position is reach, the system automatically enters the `WAITING2` state, signaling readiness to begin data collection. When the user presses the button again, the program transitions to the `LIDAR` state, actively rotating the motor and collecting distance data from the ToF sensor. Pressing the button once more returns the system to the initial `WAITING1` state, completing the control loop.

To display an boot screen, we added a boolean flag, which when true, will display the an [image](#images) and instructions during the `WAITING1` state. After the initial boot, the flag is set to false and all further `WAITING1` states do not display the boot screen.

### Images

![Man image](man.png)

---

## Results of the Design

Data, results, scope traces, etc.

- Without wire getting caught, the stepper was not overtorqued and would consistently return to 0. 

## Conclusions

Analyze your results and discuss improvements.

## Appendix A

The group does not approve this report for inclusion on the course website.

The group does not approve the video for inclusion on the course YouTube channel.

© 2025 Mac Marsh (mpm297) | Dalton Luce (dcl252) | Arnav Muthiayen (am2589) — Cornell University
