/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 * 
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 * 
 * HARDWARE CONNECTIONS
 *     MOTOR
 *   - GPIO 2 -> IN1
 *   - GPIO 3 -> IN2
 *   - GPIO 3 -> IN3
 *   - GPIO 3 -> IN4
 *   - +3.3 V or 5V power supply to + terminal on driver
 *   - REMEMBER TO GROUND DRIVER
 * 
 *     VGA (as usual)
 * 
 *   - GPIO 28 -> Button -> Ground
 */
// STD Libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"

// Hardware libraries
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

// Class libraries
#include "vga16_graphics.h"
#include "motor_library.h"
#include "pt_cornell_rp2040_v1_3.h"

// UART Setup
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// Only certain GPIO may be used for UART
#define UART_TX_PIN 12
#define UART_RX_PIN 13

// Screen parameters
#define SCREEN_Y 480 // screen height
#define SCREEN_X 640 // screen width
#define CENTER_X 321
#define CENTER_Y 240

// Constants
#define PX_PER_MM 1 // How many pixels make up a millimeters
#define RAD_PER_STEP 0.0015339807878818 // AKA Stride Angle for 28BYJ-48

// VGA semaphore
static struct pt_sem vga_semaphore;

// Store points in polar form
typedef struct 
{
    int distance; // In millimeters
    float angle;  // In radians
} Point;

volatile Point active_point;

volatile float current_angle; // Track the current angle of the motor in radians

// GPIO for timing the ISR
#define ISR_GPIO 15
// GPIO for motor
// IMPORTANT: Uses this + subsequent 3 pins, so this uses GPIO 2,3,4,5
#define MOTOR2_IN1 2
#define BUTTON_PIN 28 // GPIO Pin for BUTTON

// Program states (controlled by button)
typedef enum {
  ACTIVE,
  RESET
} ProgState;
volatile ProgState program_state = ACTIVE;

typedef enum {
  NOT_PRESSED,
  MAYBE_PRESSED,
  PRESSED,
  MAYBE_NOT_PRESSED
} ButtonState;
ButtonState button_state = NOT_PRESSED;

/* Debounce the button press
 */
bool check_button()
{
  bool button_reading = gpio_get(BUTTON_PIN);

  switch (button_state) {
    case NOT_PRESSED:
      if (button_reading == 0) {
        button_state = MAYBE_PRESSED;
      }
      break;

    case MAYBE_PRESSED:
      if (button_reading == 0) {
        button_state = PRESSED;

        // Reset
        program_state = RESET;

      } else {
        button_state = NOT_PRESSED;
      }
      break;

    case PRESSED:
      if (button_reading == 1) {
        button_state = MAYBE_NOT_PRESSED;
      }
      break;

    case MAYBE_NOT_PRESSED:
      if (button_reading == 0) {
        button_state = PRESSED;
      } else {
        button_state = NOT_PRESSED;
      }
      break;

    default:
        button_state = NOT_PRESSED;
  }
}

// character array
char screentext[200];
// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    while (true) {

        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);

        // Check button for presses
        check_button();

        if (program_state == RESET)
        {
            fillRect(0, 0, SCREEN_X, SCREEN_Y, BLACK);
            program_state = ACTIVE;
        }

        // Draw active point
        int dist = active_point.distance;
        float angle = active_point.angle;
        int x_pixel = CENTER_X + (int) (dist * PX_PER_MM * cos(angle));
        int y_pixel = CENTER_Y + (int) (dist * PX_PER_MM * sin(angle));

        drawPixel(x_pixel, y_pixel, BLUE);

        float angle_deg = angle * 180.0 / M_PI;

        // Display textual info
        setTextColor2(WHITE, BLACK);
        setTextSize(1);
        sprintf(screentext, "Distance (mm): %d      ", dist);
        setCursor(10, 10) ;
        writeString(screentext) ;
        sprintf(screentext, "Angle (deg):   %f      ", angle_deg) ;
        setCursor(10, 20);
        writeString(screentext);
    }

    // Indicate end of thread
    PT_END(pt);
}

// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        // Can we send it back?
        if (uart_is_writable(UART_ID)) {
            // Change it slightly first!
            ch++;
            uart_putc(UART_ID, ch);
        }
        // do something!! 
    }
}

// NOTE: This is called every time the motor finishes executing a command.
// Thus, we call our measurement stuff here...
void pio1_interrupt_handler() {
    pio_interrupt_clear(pio_1, 0) ;

    // 1. Read data from ToF
    // TODO: read from i2c
    active_point.distance += (rand() % 7) - 3; // 0 to 6000
    active_point.angle += RAD_PER_STEP;
    if (active_point.distance < 0)
    {
        active_point.distance = 0;
    }
    if (active_point.angle >= 2 * M_PI)
    {
        active_point.angle = 0.0;
    }

    // 2. Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

    // 3. Move motor again
    current_angle += RAD_PER_STEP;
    MOVE_STEPS_MOTOR_2(1);
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // NOTE: this needs to be called before VGA setup! This will use DMA 
    // channels 0 - 9, and VGA will claim the remaining 2
    // STEPPER CONFIGURATION ---------------------------------------------------
    setupMotor2(MOTOR2_IN1, pio1_interrupt_handler) ;

    // VGA CONFIGURATION -------------------------------------------------------
    initVGA();

    // UART CONFIGURATION
    uart_init(UART_ID, 2400); // Set up our UART with a basic baud rate.
    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);
    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);
    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    active_point.distance = 30;
    active_point.angle = 0.0;

    // Map BUTTON to GPIO port, make it low
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    // IMPORTANT: start the motor!
    pio1_interrupt_handler();

    // pt_add_thread(protothread_serial) ;
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;

    // IMPORTANT: must loop here
    while (true) {
    }
}
