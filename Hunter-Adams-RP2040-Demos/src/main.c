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

// Local libraries
#include "button.h"

// UART Setup
#define UART_ID uart1     // Need to use uart1 since debugger probe is uart0
#define BAUD_RATE 115200  // IMPORTANT: make sure this matches the Arduino baud rate
#define DATA_BITS 8       // These are Arduino default settings:
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// Only certain GPIO may be used for UART
#define UART_TX_PIN 8
#define UART_RX_PIN 9

// GPIO for motor
// IMPORTANT: Uses this + subsequent 3 pins, so this uses GPIO 2,3,4,5
#define MOTOR2_IN1 2

// Screen parameters
#define SCREEN_Y 480 // screen height
#define SCREEN_X 640 // screen width
#define CENTER_X 321
#define CENTER_Y 240

// Constants
#define PX_PER_MM 0.1 // How many pixels make up a millimeters
#define RAD_PER_STEP 0.0015339807878818 * 2 // AKA Stride Angle for 28BYJ-48

// VGA semaphore
static struct pt_sem vga_semaphore;

volatile int16_t current_distance = COUNTERCLOCKWISE;   // Track the current distance measurement
volatile float current_angle      = 0.0;                // Track the current angle of the motor in radians
volatile int current_direction    = 0;                  // Track the current direction of the motor

/* Overall program states
 */
typedef enum {
  WAITING1,         // Boot into this state, this is when we could be calibrating the sensor
  ZEROING,          // Hold button to move to zero (TODO: OR USE A SWITCH TO DETERMINE ZERO)
  WAITING2,         // Wait to statt collecting data
  LIDAR             // Start collecting data, next button press goes back to waiting
} ProgramState;
ProgramState prog_state = WAITING1;

// NOTE: This is called every time the motor finishes executing a command.
// Thus, we call our measurement stuff here...
void pio1_interrupt_handler() {
    pio_interrupt_clear(pio_1, 0);

    if (prog_state == ZEROING)
    {
        // If zeroing, just move, nothing else
        MOVE_STEPS_MOTOR_2(1);
        return;
    }

    // Only move otherwise if we are in LiDAR mode
    if (prog_state != LIDAR)
    {
        return;
    }

    // 1. Increment angle representation (switch direction if required)
    if (current_direction == CLOCKWISE)
    {
        current_angle -= RAD_PER_STEP;
        if (current_angle <= 0)
        {
            current_direction = COUNTERCLOCKWISE;
            SET_DIRECTION_MOTOR_2(current_direction);
            current_angle = 0.0;
        }
    } else
    {
        current_angle += RAD_PER_STEP;
        if (current_angle >= 2 * M_PI)
        {
            current_angle = 2 * M_PI;
            current_direction = CLOCKWISE;
            SET_DIRECTION_MOTOR_2(current_direction);
        }
    }

    // Normalize distance.
    // TODO: what does a negative distance mean? It appears the sensor may output
    // such a number
    if (current_distance < 0)
    {
        current_distance = 0;
    }

    // 2. Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

    // 3. Signal motor to move one step again
    MOVE_STEPS_MOTOR_2(1);
}

// Button to clear screen
volatile bool clear_screen = false; // Flag read by VGA thread
void clear_button_on_press(void) {
    clear_screen = true;
}
Button clear_button = {
    .gpio = 28,
    .state = NOT_PRESSED,
    .on_press = clear_button_on_press,
    .on_release = NULL,
};

// Button to manage program state
void state_button_on_press(void) {
    switch (prog_state) {
        case WAITING1:
            prog_state = ZEROING;
            // Start motor
            pio1_interrupt_handler();
            break;
        case ZEROING:
            // This shouldn't happen
            break;
        case WAITING2:
            prog_state = LIDAR;
            // Start motor ahgain
            pio1_interrupt_handler();
            break;
        case LIDAR:
            prog_state = WAITING1;
            break;
        default:
            printf("An error has occured.");
            break;
    }
}
void state_button_on_release(void) {
    switch (prog_state) {
        case WAITING1:
            // This shouldn't happen
            break;
        case ZEROING:
            prog_state = WAITING2;
            break;
        case WAITING2:
            // This shouldn't happen
            break;
        case LIDAR:
            // This shouldn't happen
            break;
        default:
            printf("An error has occured.");
            break;
    }
}
Button state_button = {
    .gpio = 29,
    .state = NOT_PRESSED,
    .on_press = state_button_on_press,
    .on_release = state_button_on_release,
};

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

        // Check buttons for press
        check_button(&clear_button);
        check_button(&state_button);

        if (clear_screen)
        {
            fillRect(0, 0, SCREEN_X, SCREEN_Y, BLACK);
            clear_screen = false;
        }

        // Draw active point
        int dist = current_distance;
        float angle = current_angle;
        // FIXME: is this correct. WTH is going on here.
        int x_pixel = CENTER_X + (int) (dist * PX_PER_MM * cos(angle));
        int y_pixel = CENTER_Y - (int) (dist * PX_PER_MM * sin(angle));

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

uint8_t rx_buf[2];
int received = 0;
#define TERMINATING_CHAR '\n' // This is what sendInt16() in the Arduino program uses
/* This is called every time we recieve a byte over the UART channel. Here, we are
 * only recieving two bytes that form a int16_t (current distance reading in mm).
 */
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        if (ch == '\n') {
            if (received == TERMINATING_CHAR) {
                int16_t dist = (rx_buf[0]) | (rx_buf[1] << 8);
                current_distance = dist;
                printf("Received int: %d\n", dist);
            }
            received = 0;
        } else if (received < 2) {
            rx_buf[received] = ch;
            received += 1;
        } else {
            printf("Error, more than 2 bytes recieved before newline.");
            received = 0;
        }
    }
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // NOTE: this needs to be called before VGA setup! This will use DMA
    // channels 0 - 9, and VGA will claim the remaining 2
    // STEPPER CONFIGURATION ---------------------------------------------------
    setupMotor2(MOTOR2_IN1, pio1_interrupt_handler) ;
    SET_DIRECTION_MOTOR_2(current_direction);

    // VGA CONFIGURATION -------------------------------------------------------
    initVGA();

    // UART CONFIGURATION
    uart_init(UART_ID, BAUD_RATE); // Set up our UART with a basic baud rate.
    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);
    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    // FIXME: do we want this or not?
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

    // Init buttons
    gpio_init(clear_button.gpio);
    gpio_set_dir(clear_button.gpio, GPIO_IN);
    gpio_pull_up(clear_button.gpio);

    gpio_init(state_button.gpio);
    gpio_set_dir(state_button.gpio, GPIO_IN);
    gpio_pull_up(state_button.gpio);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    // pt_add_thread(protothread_serial) ;
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;

    // IMPORTANT: must loop here
    while (true) {
    }
}
