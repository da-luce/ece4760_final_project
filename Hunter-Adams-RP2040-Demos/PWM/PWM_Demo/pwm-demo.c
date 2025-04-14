/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 * 
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 * 
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output
 * 
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

// Class libraries
#include "vga16_graphics.h"
#include "motor_library.h"
#include "pt_cornell_rp2040_v1_3.h"

// Screen parameters
#define SCREEN_Y 480 // screen height
#define SCREEN_X 640 // screen width
#define CENTER_X 321
#define CENTER_Y 240

// Constants
#define PX_PER_MM 2 // How many pixels make up a millimeters

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
#define RAD_PER_STEP 0.0534

// PWM wrap value and clock divide value
// For a CPU rate of 125 MHz, this gives
// a PWM frequency of 1 kHz.
#define WRAPVAL 5000
#define CLKDIV 25.0f

// GPIO we're using for PWM
#define PWM_OUT 4
// GPIO for timing the ISR
#define ISR_GPIO 15
// GPIO for Motor, + subsequent 3 pins
// So this uses GPIO 2,3,4,5
#define MOTOR2_IN1 2

// Variable to hold PWM slice number
uint slice_num ;

// PWM duty cycle
volatile int control ;
volatile int old_control ;

// PWM interrupt service routine
void on_pwm_wrap() {

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1) ;

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));

    // PT_SEM_SIGNAL(pt, &vga_semaphore);

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0) ;
}

// User input thread
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int test_in ;
    while(1) {
        sprintf(pt_serial_out_buffer, "input a duty cycle (0-5000): ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        if (test_in > 5000) continue ;
        else if (test_in < 0) continue ;
        else control = test_in ;
    }
    PT_END(pt) ;
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

// NOTE: This is called every time the motor finishes executing a command.
// Thus, we call our measurement stuff here...
void pio1_interrupt_handler() {
    pio_interrupt_clear(pio_1, 0) ;

    printf("EEEEEE");
    // 1. Read data from ToF
    // TODO: read from i2c
    active_point.distance += 1;
    active_point.angle += 0.01;

    // 2. Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

    // 3. Move motor again
    current_angle += RAD_PER_STEP;
    MOVE_STEPS_MOTOR_2(0xFF) ;
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

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;

    active_point.distance = 0;
    active_point.angle = 0.0;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // // Tell GPIO PWM_OUT that it is allocated to the PWM
    // gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
    // slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // // and register our interrupt handler
    // pwm_clear_irq(slice_num);
    // pwm_set_irq_enabled(slice_num, true);
    // irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    // irq_set_enabled(PWM_IRQ_WRAP, true);

    // // This section configures the period of the PWM signals
    // pwm_set_wrap(slice_num, WRAPVAL) ;
    // pwm_set_clkdiv(slice_num, CLKDIV) ;

    // // Invert?
    // // First argument is slice number.
    // // If second argument is true, channel A is inverted.
    // // If third argument is true, channel B is inverted.
    // pwm_set_output_polarity (slice_num, 0, 1);

    // // This sets duty cycle
    // pwm_set_chan_level(slice_num, PWM_CHAN_A, 3125);

    // // Start the channel
    // pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    // IMPORTANT: start the motor!
    pio1_interrupt_handler();

    pt_add_thread(protothread_serial) ;
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;

    while (true) {
    }
}
