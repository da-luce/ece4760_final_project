/**
 * Mac M., Arnav M., Dalton L.
 * PicoScope: LiDAR made with a RasberryPi Pico.
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
#include "vga_plus.h"
#include "image.h"
#include "image_2.h"

////////////////////////////////////////////////////////////////////////////////
//  Defines
////////////////////////////////////////////////////////////////////////////////

#define DEBUG true  // Enable/disable debug info

// UART Setup
#define UART_ID uart1     // Need to use uart1 since debugger probe is uart0
#define BAUD_RATE 115200  // IMPORTANT: make sure this matches the Arduino baud rate
#define DATA_BITS 8       // These are Arduino default settings
#define STOP_BITS 1       // ''
#define PARITY    UART_PARITY_NONE

// Only certain GPIO may be used for UART
#define UART_TX_PIN 8
#define UART_RX_PIN 9

#define STATE_BUT_PIN 22
#define CLEAR_BUT_PIN 28
#define ZERO_GATE_PIN 12    // NOTE: we can treat the optical interrupter as a button and reuse the same deboucing code
#define STOP_BUT_PIN 15
#define SIGNAL_BUF_SIZE 5

volatile float signals[SIGNAL_BUF_SIZE] = {0};

// GPIO for motor
// IMPORTANT: Uses this + subsequent 3 pins, so this uses GPIO 2,3,4,5
#define MOTOR2_IN1 2

// Screen parameters
#define SCREEN_Y 480 // screen height
#define SCREEN_X 640 // screen width
#define CENTER_X 321
#define CENTER_Y 240

// Constants
#define PX_PER_MM 0.0625 // How many pixels make up a millimeters
#define RAD_PER_STEP 0.0015339807878818 * 2 // AKA Stride Angle for 28BYJ-48
#define SIGNAL_BAR_WIDTH 150
#define SIGNAL_BAR_HEIGHT 8
#define SIGNAL_BAR_X 10
#define SIGNAL_BAR_Y 40
#define SIGNAL_MAX_MMCPS 15000
const int max_mm = 3000; // Furthest measurement that will show up on the screen
// it's actually 2500...

#define RAD2DEG(x) ((x) * 180.0 / M_PI) // Convert radians to degrees

const char rainbow_colors[14] = {RED, DARK_ORANGE, ORANGE, YELLOW,
  GREEN, MED_GREEN, DARK_GREEN,
  CYAN, LIGHT_BLUE, BLUE, DARK_BLUE,
  MAGENTA, PINK, LIGHT_PINK} ;

// VGA semaphore
static struct pt_sem vga_semaphore;

// VGA Flags
volatile bool clear_screen  = false;    // Clear the screen
volatile bool boot_screen   = true;     // Display the welcome screen
volatile bool draw_text     = true;     // Draw text for the current state (only want to draw once)

volatile int16_t current_distance   = 0;                // Track the current distance measurement
volatile int32_t current_signal     = 0;                // Track current signal strength
volatile float current_angle        = 0.0;              // Track the current angle of the motor in radians
volatile float current_triangle_ang = 0.0;              // Angle at which we drew the triangle
volatile int current_direction      = COUNTERCLOCKWISE; // Track the current direction of the motor

volatile bool zeroed = false;   // Set to true once we've hit the gate
volatile bool stopped = false;  // Emergency stop

/* Overall program states
 */
typedef enum {
  WAITING1, // Boot into this state, this is when we could be calibrating the sensor
  ZEROING,  // Upon button hold, move to zero (TODO: OR USE A SWITCH TO DETERMINE ZERO)
  WAITING2, // Upon button release, wait to start collecting data
  LIDAR     // Upon button press, start collecting data, next button press goes back to WAITING1
} ProgramState;
ProgramState prog_state = WAITING1;

// NOTE: This is called every time the motor finishes executing a command.
// Thus, we call our measurement stuff here...
void pio1_interrupt_handler() {
    pio_interrupt_clear(pio_1, 0);

    if (stopped)
    {
        return;
    }

    if (prog_state == ZEROING)
    {
        if (zeroed)
        {
            zeroed = false;
            // IMPORTANT: set the angle to zero!
            current_angle = 0.0;
            current_direction = COUNTERCLOCKWISE;
            SET_DIRECTION_MOTOR_2(current_direction);
            clear_screen = true;
            prog_state = WAITING2;
        } else
        {
            // If not zeroed yet, just keep moving
            MOVE_STEPS_MOTOR_2(1);
        }
        return;
    }

    // Only move otherwise if we are in LiDAR mode
    if (prog_state != LIDAR)
    {
        return;
    }

    // Increment angle representation (switch direction if required)
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
    if (current_distance < 0 )
    {
        current_distance = 0;
    }
    else if (current_distance > max_mm)
    {
        current_distance = max_mm;
    }

    // Signal motor to move one step again as we are in LiDAR mode
    MOVE_STEPS_MOTOR_2(1);
}

// Button to clear screen
void clear_button_on_press(void) {
    clear_screen = true;
}
Button clear_button = {
    .gpio = CLEAR_BUT_PIN,
    .state = NOT_PRESSED,
    .on_press = clear_button_on_press,
    .on_release = NULL,
};

void zero_gate_on_block(void) {
    zeroed = true;
}
Button zero_gate = {
    .gpio = ZERO_GATE_PIN,
    .state = NOT_PRESSED,
    .on_press = zero_gate_on_block,
    .on_release = NULL,
};

void stop_button_on_press(void) {
    stopped = !stopped;

    // If restarting, start motor again if necessary
    if (!stopped)
    {
        switch (prog_state) {
            case WAITING1:
                break;
            case ZEROING:
                // Start motor
                pio1_interrupt_handler();
                break;
            case WAITING2:
                break;
            case LIDAR:
                // Start motor
                pio1_interrupt_handler();
                break;
            default:
                break;
        }
    }
}

void add_signal(int32_t new_signal) {

// ################### BEGIN AI-GENERATED CODE ####################
    static int count = 0;
    static float sum = 0.0f;
// ################### END AI-GENERATED CODE ######################
  
    sum+=new_signal;
    count++;
    if (count>=25) {
        current_signal =(int32_t)(sum/25.0f);      
        count = 0;
        sum = 0.0f;
    }
}

Button stop_button = {
    .gpio = STOP_BUT_PIN,
    .state = NOT_PRESSED,
    .on_press = stop_button_on_press,
    .on_release = NULL,
};

char map_to_color(int value, int min_val, int max_val) {
    if (value <= min_val) return 0;
    if (value >= max_val) return 13;

    return rainbow_colors[(value - min_val) * 13 / (max_val - min_val)];
}

// Button to manage program state
void state_button_on_press(void) {

    // TODO: check if this works
    draw_text = true; // Redraw text

    switch (prog_state) {
        case WAITING1:
            // We want to always zero in the counterclockwise direction
            current_direction = COUNTERCLOCKWISE;
            SET_DIRECTION_MOTOR_2(current_direction);
            prog_state = ZEROING;

            // Start motor
            pio1_interrupt_handler();
            break;
        case ZEROING:
            // Don't do anything

            break;
        case WAITING2:
            clear_screen = true;
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

    // TODO: check if this works
    draw_text = true;

    switch (prog_state) {
        case WAITING1:
            // This shouldn't happen
            break;
        case ZEROING:
            // Doesn't matter
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
    .gpio = STATE_BUT_PIN,
    .state = NOT_PRESSED,
    .on_press = state_button_on_press,
    .on_release = state_button_on_release,
};

#define SCALE 1024
int sin_table[360];
int cos_table[360];

void init_trig_tables() {
    for (int deg = 0; deg < 360; deg++) {
        float rad = deg * M_PI / 180.0;
        sin_table[deg] = (int)(sin(rad) * SCALE);
        cos_table[deg] = (int)(cos(rad) * SCALE);
    }
}

void drawTrianglePointerOutline(int angle_deg, int radius_px, char color) {
    int label_margin = 40;       // space for angle text
    int triangle_offset = 4;     // buffer past labels
    int r_base = radius_px + label_margin + triangle_offset;

    int arrow_length = 14;       // from base to tip
    int half_width = 7;

    int cos_a = cos_table[angle_deg % 360];
    int sin_a = sin_table[angle_deg % 360];
    int perp_cos = cos_table[(angle_deg + 90) % 360];
    int perp_sin = sin_table[(angle_deg + 90) % 360];

    // Base center
    int base_cx = CENTER_X + (r_base * cos_a) / SCALE;
    int base_cy = CENTER_Y - (r_base * sin_a) / SCALE;

    // Tip pointing inward
    int tip_x = base_cx - (arrow_length * cos_a) / SCALE;
    int tip_y = base_cy + (arrow_length * sin_a) / SCALE;

    // Base corners (perpendicular to angle)
    int base1_x = base_cx + (half_width * perp_cos) / SCALE;
    int base1_y = base_cy - (half_width * perp_sin) / SCALE;

    int base2_x = base_cx - (half_width * perp_cos) / SCALE;
    int base2_y = base_cy + (half_width * perp_sin) / SCALE;

    // Draw triangle outline
    drawLine(tip_x, tip_y, base1_x, base1_y, color);
    drawLine(tip_x, tip_y, base2_x, base2_y, color);
    drawLine(base1_x, base1_y, base2_x, base2_y, color);
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
        // PT_SEM_WAIT(pt, &vga_semaphore);

        check_button(&clear_button);
        check_button(&state_button);
        check_button(&zero_gate);
        check_button(&stop_button);

        if (DEBUG)
        {
            // Display textual info
            setTextColor2(WHITE, BLACK);
            setTextSize(1);
            switch (prog_state) {
                case WAITING1:
                    sprintf(screentext, "Program State: Pending...");
                    setCursor(SCREEN_X - 200, 10) ;
                    writeString(screentext);
                    break;
                case ZEROING:
                    sprintf(screentext, "Program State: Zeroing");
                    setCursor(SCREEN_X - 200, 10) ;
                    writeString(screentext);
                    break;
                case WAITING2:
                    sprintf(screentext, "Program State: Pending...");
                    setCursor(SCREEN_X - 200, 10) ;
                    writeString(screentext);
                    break;
                case LIDAR:
                    sprintf(screentext, "Program State: LIDAR");
                    setCursor(SCREEN_X - 200, 10) ;
                    writeString(screentext);
                    break;
                default:
                    sprintf(screentext, "Program State: Pending...");
                    setCursor(SCREEN_X - 200, 10) ;
                    writeString(screentext);
                    break;
                }

            sprintf(screentext, "Clear Button (GPIO %d): %d ", clear_button.gpio, clear_button.state);
            setCursor(SCREEN_X - 200, 20) ;
            writeString(screentext);
            sprintf(screentext, "State Button (GPIO %d): %d ", state_button.gpio, state_button.state);
            setCursor(SCREEN_X - 200, 30) ;
            writeString(screentext);
            sprintf(screentext, "Zero Button (GPIO %d): %d ", zero_gate.gpio, zero_gate.state);
            setCursor(SCREEN_X - 200, 40) ;
            writeString(screentext);
            sprintf(screentext, "Stop Button (GPIO %d): %d ", stop_button.gpio, stop_button.state);
            setCursor(SCREEN_X - 200, 50) ;
            writeString(screentext);
            sprintf(screentext, "Stopped: %d ", stopped);
            setCursor(SCREEN_X - 200, 60) ;
            writeString(screentext);
        }

        if (clear_screen)
        {
            fillRect(0, 0, SCREEN_X, SCREEN_Y, BLACK);
            clear_screen = false;
        }

        // FIXME: this should only show up on boot
        // Also, we only want to draw text once for each state, add a flag to do this
        if (prog_state == WAITING1 && boot_screen && draw_text)
        {
            setTextSize(3);

            setTextColor2(BLUE, BLACK);
            sprintf(screentext, "Welcome to the PicoScope!");
            setCursor(CENTER_X - 300 - 4, CENTER_Y - 4);
            writeString(screentext);

            drawImage(0, 0, SCREEN_X, SCREEN_Y, image_data);
            drawImage(SCREEN_X, SCREEN_Y, IMAGE_WIDTH_2, IMAGE_HEIGHT_2, image_data_2);

            setTextColor2(WHITE, BLACK);
            sprintf(screentext, "Welcome to the PicoScope!");
            setCursor(CENTER_X - 300, CENTER_Y);
            writeString(screentext);

            setTextSize(1);
            setTextColor2(WHITE, BLACK);
            sprintf(screentext, "Press green button to zero...");
            setCursor(CENTER_X - 300, CENTER_Y + 50);
            writeString(screentext);

            setTextColor2(WHITE, BLACK);
            sprintf(screentext, "Press blue button to clear...");
            setCursor(CENTER_X - 300, CENTER_Y + 100);
            writeString(screentext);

            setTextColor2(WHITE, BLACK);
            sprintf(screentext, "Press gray button to stop...");
            setCursor(CENTER_X - 300, CENTER_Y + 150);
            writeString(screentext);

            draw_text = false;
            boot_screen = false;
        }

        if (prog_state == WAITING2 && draw_text)
        {
            setTextColor2(WHITE, BLACK);
            setTextSize(3);
            sprintf(screentext, "Press again to begin scanning!");
            setCursor(CENTER_X - 300, CENTER_Y);
            writeString(screentext);
        }

        // If we are not in LiDAR mode, don't draw any points
        if (prog_state != LIDAR)
        {
            continue;
        }

        // Erase the old triangle
        drawTrianglePointerOutline((int) (RAD2DEG(current_triangle_ang)), max_mm * PX_PER_MM, BLACK);

        // Store the current angle as a local variable
        // NOTE: if you do not do this, current_angle may update between and
        // cause strange visual glitches
        float current_temp = current_angle;
        drawTrianglePointerOutline((int)(RAD2DEG(current_temp)), max_mm * PX_PER_MM, WHITE);
        current_triangle_ang = current_temp;

        // Draw active point
        int dist = current_distance;
        float angle = current_angle;
        float angle_deg = RAD2DEG(angle);

        // FIXME: is this correct. WTH is going on here.
        int x_pixel = CENTER_X + (int) (dist * PX_PER_MM * cos(angle));
        int y_pixel = CENTER_Y - (int) (dist * PX_PER_MM * sin(angle));
        char color = map_to_color(dist, 0, max_mm);
        drawPixel(x_pixel, y_pixel, color);

        // Display textual info
        setTextColor2(WHITE, BLACK);
        setTextSize(1);
        sprintf(screentext, "Distance (mm): %d      ", dist);
        setCursor(10, 10) ;
        writeString(screentext) ;
        sprintf(screentext, "Angle (deg):   %f      ", angle_deg) ;
        setCursor(10, 20);
        writeString(screentext);
        sprintf(screentext, "Signal (mMCPS): %d     ", current_signal);
        setCursor(10, 30);
        writeString(screentext);


        // Calculate filled bar length based on signal light
        float signal_bar_length = (current_signal * SIGNAL_BAR_WIDTH) / SIGNAL_MAX_MMCPS;

        // Clamp the signal_bar_length for graphical display
        if (signal_bar_length > SIGNAL_BAR_WIDTH) signal_bar_length = SIGNAL_BAR_WIDTH;

       // Draw filled bar, maintain constant bar size
        fillRect(SIGNAL_BAR_X, SIGNAL_BAR_Y, SIGNAL_BAR_WIDTH, SIGNAL_BAR_HEIGHT, BLACK);
        fillRect(SIGNAL_BAR_X, SIGNAL_BAR_Y, signal_bar_length, SIGNAL_BAR_HEIGHT, CYAN);
      
        // Draw an outline for the bar
        drawRect(SIGNAL_BAR_X, SIGNAL_BAR_Y, SIGNAL_BAR_WIDTH, SIGNAL_BAR_HEIGHT, WHITE);

        // Draw distance rings
        for (int i = 1; i < 7; i++) {
            int radius = (max_mm * PX_PER_MM * i) / 6;
            drawCircle(CENTER_X, CENTER_Y, (short) radius, WHITE);

            // Label distance at right side of the circle
            char label[8];
            float m = (max_mm * (float)i) / 6.0f / 1000.0f;
            sprintf(label, "%.1fm", m);

            int x = CENTER_X + radius + 4; // small offset outside the circle
            int y = CENTER_Y - 4;          // small offset

            setCursor(x, y);
            writeString(label);
        }

        int label_radius = max_mm * PX_PER_MM + 8; // slightly outside the circle
        int label_width = 16;
        int label_height = 16;

        for (int angle_label = 45; angle_label < 360; angle_label += 45) {
            float angle_label_rad = angle_label * M_PI / 180.0;

            // Offset radius to push label slightly away from circle
            float padding = 6.0f; // pixels — adjust if needed
            float label_x = CENTER_X + (label_radius + padding) * cos(angle_label_rad);
            float label_y = CENTER_Y - (label_radius + padding) * sin(angle_label_rad);

            // Generate label string
            char label[4];
            sprintf(label, "%d", angle_label);

            int label_len = strlen(label);
            int label_width = label_len * 4;  // 4 px per character
            int label_height = 4;

            // Center horizontally and vertically
            int cursor_x = (int)(label_x - label_width / 2);
            int cursor_y = (int)(label_y - label_height / 2);

            setCursor(cursor_x, cursor_y);
            writeString(label);
        }
    }

    // Indicate end of thread
    PT_END(pt);
}

uint8_t rx_buf[6];
int received = 0;
#define TERMINATING_CHAR '\n' // This is what sendInt16() in the Arduino program uses
/* This is called every time we recieve a byte over the UART channel. Here, we are
 * only recieving two bytes that form a int16_t (current distance reading in mm).
 *
 * NOTE: important with printing here. Printing before reading the char will mess things
 * up for sure
 */

void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);

        if (ch == TERMINATING_CHAR) {
            if (received == 6) {
                int16_t dist = (rx_buf[0]) | (rx_buf[1] << 8);
                ///int16_t signal = (rx_buf[2]) | (rx_buf[3] << 8);

                float signal = (rx_buf[2]) | (rx_buf[3] << 8)|(rx_buf[4])<<16 |(rx_buf[5] << 24);
                current_distance = dist;
                add_signal(signal);

                received = 0;  // Reset for next packet
            }
        } else if (received < 6) {
            rx_buf[received++] = ch;
        } else {
            printf("Error: more than 6 bytes received before newline.\n");
            received = 0;
        }
    }

}

int main() {
    // set_sys_clock_khz(250000, true);
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
    // Turn off FIFO's (tried to use FIFO and take 2 bytes at a time via DMA but
    // RP2040 isn't set up to do this very well--see link in progress report 2)
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

    gpio_init(zero_gate.gpio);
    gpio_set_dir(zero_gate.gpio, GPIO_IN);
    gpio_pull_up(zero_gate.gpio);

    gpio_init(stop_button.gpio);
    gpio_set_dir(stop_button.gpio, GPIO_IN);
    gpio_pull_up(stop_button.gpio);

    pio1_interrupt_handler();

    init_trig_tables();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;

    // IMPORTANT: must loop here
    while (true) {
    }
}
