
/* Button resources. NOTE: cannot include this in more than one file since function
 * defines are in here.
 */

// STD Libraries
#include <stdbool.h>

// PICO libraries
#include "pico/stdlib.h"

typedef enum {
  NOT_PRESSED,
  MAYBE_PRESSED,
  PRESSED,
  MAYBE_NOT_PRESSED
} ButtonState;
ButtonState button_state = NOT_PRESSED;

typedef struct 
{
    uint gpio;
    ButtonState state;
    void (*on_press)(void);     // Function on press
    void (*on_release)(void);   // Function on release
} Button;

/* Debounce button press
 */
bool check_button(Button *button)
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