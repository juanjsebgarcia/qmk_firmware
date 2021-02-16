#include "pimoroni_trackball.h"
#include "i2c_master.h"

#ifndef TRACKBALL_NO_MATH
#include "math.h"
#   ifndef TRACKBALL_ANGLE_OFFSET
#   define TRACKBALL_ANGLE_OFFSET 0
#   endif
#endif

#ifndef TRACKBALL_ORIENTATION
#   define TRACKBALL_ORIENTATION 2
#endif

void trackball_init(void) {
    i2c_init();
#ifdef TRACKBALL_INTERRUPT_PIN
    setPinInput(TRACKBALL_INTERRUPT_PIN);
    writePinLow(TRACKBALL_INTERRUPT_PIN);
    uint8_t data[] = {REG_INTERRUPT_PIN, MASK_INTERRUPT_PIN_ENABLE};
    i2c_transmit(TRACKBALL_WRITE, data, 2, TB_I2C_TIMEOUT);
#endif
}

bool trackball_get_interrupt(void) {
#ifndef TRACKBALL_INTERRUPT_PIN
    uint8_t data[1] = {};
    i2c_readReg(TRACKBALL_WRITE, REG_INTERRUPT_PIN, data, 1, TB_I2C_TIMEOUT);

    return data[0] & MASK_INTERRUPT_TRIGGERED;
#else
    return !readPin(TRACKBALL_INTERRUPT_PIN);
#endif
}

void trackball_set_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
    uint8_t data[] = {0x00, red, green, blue, white};
    i2c_transmit(TRACKBALL_WRITE, data, 5, TB_I2C_TIMEOUT);
}

void trackball_read_state(uint8_t* data, uint16_t size_of_data) {
    i2c_readReg(TRACKBALL_WRITE, REG_LEFT, data, size_of_data, TB_I2C_TIMEOUT);
}

trackball_state_t trackball_get_state(void) {
    // up down left right button
    uint8_t s[5] = {};
    trackball_read_state(s, 5);

    trackball_state_t state = {
#if TRACKBALL_ORIENTATION == 0
        // Pimoroni text is up
        .y = s[0] - s[1],
        .x = s[3] - s[2],
#elif TRACKBALL_ORIENTATION == 1
        // Pimoroni text is right
        .y = s[3] - s[2],
        .x = s[1] - s[0],
#elif TRACKBALL_ORIENTATION == 2
        // Pimoroni text is down
        .y = s[1] - s[0],
        .x = s[2] - s[3],
#else
        // Pimoroni text is left
        .y = s[2] - s[3],
        .x = s[0] - s[1],
#endif
        .button_down = s[4] & 0x80,
        .button_triggered = s[4] & 0x01,
    };

#ifndef TRACKBALL_NO_MATH
    state.angle_rad = atan2(state.y, state.x) + TRACKBALL_ANGLE_OFFSET;
    state.vector_length = sqrt(pow(state.x, 2) + pow(state.y, 2));
    state.raw_x = state.x;
    state.raw_y = state.y;
    state.x = (int16_t)(state.vector_length * cos(state.angle_rad));
    state.y = (int16_t)(state.vector_length * sin(state.angle_rad));
#endif

    return state;
}

void trackball_sleep(void) {
    /* not sure how this is supposed to work */
    uint8_t data[] = {REG_CTRL, MSK_CTRL_FWRITE | MSK_CTRL_SLEEP};
    i2c_transmit(TRACKBALL_WRITE, data, 2, TB_I2C_TIMEOUT);
}

void trackball_set_brightness(uint8_t brightness) {
    uint8_t data[4] = {};
    i2c_readReg(TRACKBALL_WRITE, REG_RED, data, 4, TB_I2C_TIMEOUT);
    for (int i=0; i<4; i++) {
        if (data[i]) {
            data[i] = brightness;
        }
    }
    i2c_writeReg(TRACKBALL_WRITE, REG_RED, data, 4, TB_I2C_TIMEOUT);
}

#ifndef MIN
#    define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
void trackball_set_hsv(uint8_t hue, uint8_t sat, uint8_t brightness) {
    RGB rgb = hsv_to_rgb((HSV){hue, sat, brightness});
    uint8_t white = MIN(rgb.r, MIN(rgb.g, rgb.b));
    rgb.r -= white;
    rgb.g -= white;
    rgb.b -= white;

    trackball_set_rgbw(rgb.r, rgb.g, rgb.b, white);
}

__attribute__((weak)) void pointing_device_init(void) {
    trackball_set_rgbw(0,0,0,50);
}

__attribute__((weak)) void process_mouse_user(report_mouse_t* mouse_report, int16_t x, int16_t y) {
    mouse_report->x = x;
    mouse_report->y = y;
}
__attribute__((weak)) void update_member(int8_t* member, int16_t* offset) {
    if (*offset > 127) {
        *member = 127;
        *offset -= 127;
    } else if (*offset < -127) {
        *member = -127;
        *offset += 127;
    } else {
        *member = *offset;
        *offset = 0;
    }
}

__attribute__((weak)) bool has_report_changed(report_mouse_t new, report_mouse_t old) {
    return (new.buttons != old.buttons) ||
           (new.x && new.x != old.x) ||
           (new.y && new.y != old.y) ||
           (new.h && new.h != old.h) ||
           (new.v && new.v != old.v);
}

static int16_t x_offset = 0;
static int16_t y_offset = 0;
static int16_t v_offset = 0;
static int16_t h_offset = 0;
static int16_t tb_timer = 0;

__attribute__((weak)) void process_mouse(report_mouse_t* mouse) {
    static int8_t new_x_offset = 0;
    static int8_t new_y_offset = 0;
    static int8_t new_v_offset = 0;
    static int8_t new_h_offset = 0;
    if (trackball_get_interrupt() && (!tb_timer || timer_elapsed(tb_timer) > TRACKBALL_TIMEOUT)) {
        tb_timer = timer_read() | 1;
        // trigger_tapping();
        // uint8_t mods = get_mods();

        trackball_state_t state = trackball_get_state();

        if (state.button_triggered) {
            if(state.button_down) {
                mouse->buttons |= MOUSE_BTN1;
            } else {
                mouse->buttons &= ~MOUSE_BTN1;
            }
        } else {
            float power = 2.5;
            double newlen = pow(state.vector_length, power);
            x_offset += (newlen * cos(state.angle_rad));
            y_offset += (newlen * sin(state.angle_rad));
        }

    }

    while (x_offset || y_offset || h_offset || v_offset) {
        update_member(&new_x_offset, &x_offset);
        update_member(&new_y_offset, &y_offset);

        update_member(&new_v_offset, &v_offset);
        update_member(&new_h_offset, &h_offset);

        mouse->x = new_x_offset;
        mouse->y = new_y_offset;
        mouse->v = new_v_offset;
        mouse->h = new_h_offset;
    }
}

__attribute__((weak)) void pointing_device_task(void) {
    report_mouse_t mouse_report = pointing_device_get_report();
    if (!is_keyboard_left() || !is_keyboard_master()) {
        process_mouse(&mouse_report);
    }

    pointing_device_set_report(mouse_report);
    pointing_device_send();
}

__attribute__((weak)) void pointing_device_send(void) {
    static report_mouse_t old_report = {};
    report_mouse_t mouseReport = pointing_device_get_report();
    if (is_keyboard_left() || is_keyboard_master()) {
        int8_t x = mouseReport.x, y = mouseReport.y;
        mouseReport.x = 0;
        mouseReport.y = 0;
        process_mouse_user(&mouseReport, x, y);
        if (has_report_changed(mouseReport, old_report)) {
            host_mouse_send(&mouseReport);
        }
    } else {
        master_mouse_send(mouseReport.x, mouseReport.y, mouseReport.buttons);
    }
    mouseReport.x = 0;
    mouseReport.y = 0;
    mouseReport.v = 0;
    mouseReport.h = 0;
    old_report = mouseReport;
    pointing_device_set_report(mouseReport);
}
