/*
 * Copyright (c) 2025 @amgskobo
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_inertia

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(input_processor_inertia, CONFIG_ZMK_LOG_LEVEL);

#define DEFAULT_INERTIA_DECAY_FACTOR_INT 90
#define DEFAULT_INERTIA_INTERVAL_MS 35
#define DEFAULT_INERTIA_THRESHOLD_START 15
#define DEFAULT_INERTIA_THRESHOLD_STOP 1
#define DEFAULT_TRIGGER_MS 35

#define DEFAULT_INERTIA_SCROLL_DECAY_FACTOR_INT 85
#define DEFAULT_INERTIA_SCROLL_INTERVAL_MS 65
#define DEFAULT_INERTIA_SCROLL_THRESHOLD_START 2
#define DEFAULT_INERTIA_SCROLL_THRESHOLD_STOP 0

#define abs16(x) ((x) < 0 ? -(x) : (x))

struct inertia_config {
    // Mouse movement config
    uint16_t move_decay_factor_int;
    uint16_t move_interval_ms;
    uint16_t move_threshold_start;
    uint16_t move_threshold_stop;

    // Scroll config
    uint16_t scroll_decay_factor_int;
    uint16_t scroll_interval_ms;
    uint16_t scroll_threshold_start;
    uint16_t scroll_threshold_stop;
    uint16_t scroll_velocity_window_ms;
    uint16_t scroll_velocity_history_ms;

    uint16_t trigger_ms;
};

// Recent scroll events, used to measure velocity when the touch is released.
#define SCROLL_HISTORY_LEN 32

// Scroll inertia stops once velocity falls to this (Q8 wheel units per report, 16 = 1/16),
// or to scroll-threshold-stop if that is higher.
#define SCROLL_MIN_VELOCITY_Q8 16

struct scroll_sample {
    int64_t time_ms;
    int16_t dx;
    int16_t dy;
};

struct inertia_state {
    // Mouse movement state
    int16_t move_vx;
    int16_t move_vy;
    int16_t move_ema_vx;
    int16_t move_ema_vy;
    int16_t move_remainder_x_q8;
    int16_t move_remainder_y_q8;
    bool move_active;
    bool move_is_inertial;

    // Scroll state
    int16_t scroll_vx;
    int16_t scroll_vy;
    int16_t scroll_ema_vx;
    int16_t scroll_ema_vy;
    int16_t scroll_remainder_x_q8;
    int16_t scroll_remainder_y_q8;
    bool scroll_active;
    bool scroll_is_inertial;
    int32_t scroll_vel_x_q8; // inertia velocity, wheel units per report (Q8)
    int32_t scroll_vel_y_q8;
    struct scroll_sample scroll_history[SCROLL_HISTORY_LEN];
    uint8_t scroll_history_head; // index of the next slot to write
    uint8_t scroll_history_count;

    // Set once an INPUT_BTN_TOUCH event is seen. Scroll inertia then starts on touch
    // release instead of on the trigger-ms timer.
    bool touch_aware;
    bool touching;
};

struct inertia_data {
    const struct device *dev;
    struct inertia_state state;
    struct k_work_delayable move_work;
    struct k_work_delayable scroll_work;
    struct k_mutex lock;
};

// ====================================================================
// Q8 Fixed-Point Decay Function
// ====================================================================
// Q8 Definitions
#define Q8_VALUE (1 << 8) // 1.0 = 256
#define Q8_HALF (1 << 7)  // 0.5 = 128

static void calculate_decayed_movement_fixed(int16_t in_dx, int16_t in_dy, int16_t decay_factor_q8,
                                      int16_t *out_dx, int16_t *out_dy, int16_t *rem_x,
                                      int16_t *rem_y) {
    // 1. True Movement (Q8, including remainder)
    int32_t ideal_dx_q8 = ((int32_t)in_dx << 8) + *rem_x;
    int32_t ideal_dy_q8 = ((int32_t)in_dy << 8) + *rem_y;

    // 2. Apply Decay
    int32_t decayed_dx_q8 = (ideal_dx_q8 * decay_factor_q8) >> 8;
    int32_t decayed_dy_q8 = (ideal_dy_q8 * decay_factor_q8) >> 8;

    // 3. Extract Integer part (Output values)
    int16_t output_dx = (int16_t)((decayed_dx_q8 + Q8_HALF) >> 8);
    int16_t output_dy = (int16_t)((decayed_dy_q8 + Q8_HALF) >> 8);

    // 4. Update and save Remainder (Q8 value)
    *rem_x = (int16_t)(decayed_dx_q8 - ((int32_t)output_dx << 8));
    *rem_y = (int16_t)(decayed_dy_q8 - ((int32_t)output_dy << 8));

    *out_dx = output_dx;
    *out_dy = output_dy;
}
// ====================================================================

static void move_decay_callback(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct inertia_data *data = CONTAINER_OF(d_work, struct inertia_data, move_work);
    const struct inertia_config *cfg = data->dev->config;

    k_mutex_lock(&data->lock, K_FOREVER);
    if (!data->state.move_active) {
        k_mutex_unlock(&data->lock);
        return;
    }

    int16_t vx = data->state.move_ema_vx;
    int16_t vy = data->state.move_ema_vy;
    int16_t next_vx = vx;
    int16_t next_vy = vy;

    // Q8 Factor Scaling
    int16_t decay_factor_q8 = (cfg->move_decay_factor_int * 256) / 100;

    // STEP 1: Q8 Fixed-Point Decay
    calculate_decayed_movement_fixed(vx, vy, decay_factor_q8, &next_vx, &next_vy,
                                     &data->state.move_remainder_x_q8,
                                     &data->state.move_remainder_y_q8);

    // STEP 2: Termination Check
    if (abs16(next_vx) <= cfg->move_threshold_stop && abs16(next_vy) <= cfg->move_threshold_stop) {
        data->state.move_vx = 0;
        data->state.move_vy = 0;
        data->state.move_ema_vx = 0;
        data->state.move_ema_vy = 0;
        data->state.move_remainder_x_q8 = 0;
        data->state.move_remainder_y_q8 = 0;
        data->state.move_active = false;
        data->state.move_is_inertial = false;
        k_mutex_unlock(&data->lock);

        zmk_hid_mouse_movement_set(0, 0);
        zmk_endpoints_send_mouse_report();
        LOG_DBG("Move Inertia stopped naturally.");
        return;
    }

    // Continue inertia
    data->state.move_vx = next_vx;
    data->state.move_vy = next_vy;
    data->state.move_ema_vx = next_vx;
    data->state.move_ema_vy = next_vy;
    data->state.move_is_inertial = true;

    k_work_reschedule(&data->move_work, K_MSEC(cfg->move_interval_ms));
    k_mutex_unlock(&data->lock);

    // Call HID outside the lock to avoid blocking other input threads
    zmk_hid_mouse_movement_set(next_vx, next_vy);
    zmk_endpoints_send_mouse_report();
    // Clear HID state to prevent conflict and jumping cursor when other ZMK inputs (e.g., clicks) occur
    zmk_hid_mouse_movement_set(0, 0);
}

static void scroll_decay_callback(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct inertia_data *data = CONTAINER_OF(d_work, struct inertia_data, scroll_work);
    const struct inertia_config *cfg = data->dev->config;

    k_mutex_lock(&data->lock, K_FOREVER); 
    if (!data->state.scroll_active) {
        k_mutex_unlock(&data->lock);
        return;
    }

    struct inertia_state *s = &data->state;

    // Velocity is tracked in Q8 separately from the emitted-distance remainder, so it can
    // keep decaying below one wheel unit per report: the tail then emits a unit every few
    // reports and fades out, instead of stopping dead once a report would round to 0.
    if (!s->scroll_is_inertial) {
        // First run after a timer trigger: start from the EMA velocity.
        s->scroll_vel_x_q8 = (int32_t)s->scroll_ema_vx << 8;
        s->scroll_vel_y_q8 = (int32_t)s->scroll_ema_vy << 8;
        s->scroll_remainder_x_q8 = 0;
        s->scroll_remainder_y_q8 = 0;
    }

    // Q8 Factor Scaling
    int32_t decay_factor_q8 = (cfg->scroll_decay_factor_int * 256) / 100;

    // STEP 1: Q8 Fixed-Point Decay
    s->scroll_vel_x_q8 = (s->scroll_vel_x_q8 * decay_factor_q8) / Q8_VALUE;
    s->scroll_vel_y_q8 = (s->scroll_vel_y_q8 * decay_factor_q8) / Q8_VALUE;

    int32_t pos_x_q8 = s->scroll_remainder_x_q8 + s->scroll_vel_x_q8;
    int32_t pos_y_q8 = s->scroll_remainder_y_q8 + s->scroll_vel_y_q8;
    int16_t next_vx = (int16_t)((pos_x_q8 + Q8_HALF) >> 8);
    int16_t next_vy = (int16_t)((pos_y_q8 + Q8_HALF) >> 8);
    s->scroll_remainder_x_q8 = (int16_t)(pos_x_q8 - ((int32_t)next_vx << 8));
    s->scroll_remainder_y_q8 = (int16_t)(pos_y_q8 - ((int32_t)next_vy << 8));

    // STEP 2: Termination Check (velocity in wheel units per report, with a small floor)
    int32_t stop_q8 = MAX((int32_t)cfg->scroll_threshold_stop * Q8_VALUE, SCROLL_MIN_VELOCITY_Q8);
    if (abs(s->scroll_vel_x_q8) <= stop_q8 && abs(s->scroll_vel_y_q8) <= stop_q8) {
        data->state.scroll_vx = 0;
        data->state.scroll_vy = 0;
        data->state.scroll_ema_vx = 0;
        data->state.scroll_ema_vy = 0;
        data->state.scroll_remainder_x_q8 = 0;
        data->state.scroll_remainder_y_q8 = 0;
        data->state.scroll_active = false;
        data->state.scroll_is_inertial = false;
        k_mutex_unlock(&data->lock);

        zmk_hid_mouse_scroll_set(0, 0);
        zmk_endpoints_send_mouse_report();
        LOG_DBG("Scroll Inertia stopped naturally.");
        return;
    }

    // Continue inertia
    data->state.scroll_vx = next_vx;
    data->state.scroll_vy = next_vy;
    data->state.scroll_ema_vx = next_vx;
    data->state.scroll_ema_vy = next_vy;
    data->state.scroll_is_inertial = true;

    k_work_reschedule(&data->scroll_work, K_MSEC(cfg->scroll_interval_ms));
    k_mutex_unlock(&data->lock);

    if (next_vx == 0 && next_vy == 0) {
        return; // slow tail: nothing to emit this report
    }

    // Call HID outside the lock to avoid blocking other input threads
    zmk_hid_mouse_scroll_set(next_vx, next_vy);
    zmk_endpoints_send_mouse_report();
    // Clear HID state to prevent scroll remnants from interfering with other inputs
    zmk_hid_mouse_scroll_set(0, 0);
}

static void scroll_history_add(struct inertia_state *s, int16_t dx, int16_t dy) {
    s->scroll_history[s->scroll_history_head] = (struct scroll_sample){
        .time_ms = k_uptime_get(),
        .dx = dx,
        .dy = dy,
    };
    s->scroll_history_head = (s->scroll_history_head + 1) % SCROLL_HISTORY_LEN;
    if (s->scroll_history_count < SCROLL_HISTORY_LEN) {
        s->scroll_history_count++;
    }
}

// Returns the i-th most recent sample (0 = newest).
static const struct scroll_sample *scroll_history_get(const struct inertia_state *s, uint8_t i) {
    return &s->scroll_history[(s->scroll_history_head + SCROLL_HISTORY_LEN - 1 - i) %
                              SCROLL_HISTORY_LEN];
}

// Start scroll inertia at the velocity of the fastest scroll-velocity-window-ms window that
// ended within scroll-velocity-history-ms of release. Velocity is distance over time, scaled
// to units per scroll-report-interval-ms, so inertia continues at the speed of the fingers
// rather than the speed of the final reports as they slow down and lift.
// Must be called with data->lock held.
static void scroll_start_on_release(struct inertia_data *data, const struct inertia_config *cfg) {
    struct inertia_state *s = &data->state;
    int64_t now = k_uptime_get();
    int32_t best_x = 0, best_y = 0;

    for (uint8_t i = 0; i < s->scroll_history_count; i++) {
        const struct scroll_sample *end = scroll_history_get(s, i);
        if (now - end->time_ms > cfg->scroll_velocity_history_ms) {
            break;
        }
        int32_t sum_x = 0, sum_y = 0;
        for (uint8_t j = i; j < s->scroll_history_count; j++) {
            const struct scroll_sample *sample = scroll_history_get(s, j);
            if (end->time_ms - sample->time_ms >= cfg->scroll_velocity_window_ms) {
                break;
            }
            sum_x += sample->dx;
            sum_y += sample->dy;
        }
        if (abs(sum_x) + abs(sum_y) > abs(best_x) + abs(best_y)) {
            best_x = sum_x;
            best_y = sum_y;
        }
    }
    s->scroll_history_count = 0;

    // Q8 velocity in units per report interval.
    int32_t vx_q8 = best_x * cfg->scroll_interval_ms * Q8_VALUE / cfg->scroll_velocity_window_ms;
    int32_t vy_q8 = best_y * cfg->scroll_interval_ms * Q8_VALUE / cfg->scroll_velocity_window_ms;
    vx_q8 = CLAMP(vx_q8, INT16_MIN * Q8_VALUE, INT16_MAX * Q8_VALUE);
    vy_q8 = CLAMP(vy_q8, INT16_MIN * Q8_VALUE, INT16_MAX * Q8_VALUE);

    int32_t start_q8 = (int32_t)cfg->scroll_threshold_start * Q8_VALUE;
    if (abs(vx_q8) < start_q8 && abs(vy_q8) < start_q8) {
        return;
    }

    s->scroll_vel_x_q8 = vx_q8;
    s->scroll_vel_y_q8 = vy_q8;
    s->scroll_remainder_x_q8 = 0;
    s->scroll_remainder_y_q8 = 0;
    s->scroll_vx = (int16_t)(vx_q8 >> 8);
    s->scroll_vy = (int16_t)(vy_q8 >> 8);
    s->scroll_ema_vx = s->scroll_vx;
    s->scroll_ema_vy = s->scroll_vy;
    s->scroll_active = true;
    s->scroll_is_inertial = true;
    k_work_reschedule(&data->scroll_work, K_NO_WAIT);
    LOG_DBG("Scroll Inertia started on release. h %d, v %d (q8)", vx_q8, vy_q8);
}

/* --- Input Processor Handler (Event-Driven Pipeline) --- */

static int inertia_handle_event(const struct device *dev, struct input_event *event,
                                uint32_t param1, uint32_t param2,
                                struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    struct inertia_data *data = (struct inertia_data *)dev->data;
    const struct inertia_config *cfg = dev->config;

    if (event->type == INPUT_EV_KEY && event->code == INPUT_BTN_TOUCH && !event->value) {
        k_mutex_lock(&data->lock, K_FOREVER);
        data->state.touch_aware = true;
        data->state.touching = false;
        scroll_start_on_release(data, cfg);
        k_mutex_unlock(&data->lock);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->type == INPUT_EV_KEY && event->code == INPUT_BTN_TOUCH && event->value) {
        k_mutex_lock(&data->lock, K_FOREVER);
        data->state.touch_aware = true;
        data->state.touching = true;
        data->state.scroll_history_count = 0;
        if (data->state.move_active) {
            k_work_cancel_delayable(&data->move_work);
            data->state.move_active = false;
            data->state.move_vx = 0;
            data->state.move_vy = 0;
            data->state.move_ema_vx = 0;
            data->state.move_ema_vy = 0;
            data->state.move_remainder_x_q8 = 0;
            data->state.move_remainder_y_q8 = 0;
            data->state.move_is_inertial = false;
            zmk_hid_mouse_movement_set(0, 0);
        }
        if (data->state.scroll_active) {
            k_work_cancel_delayable(&data->scroll_work);
            data->state.scroll_active = false;
            data->state.scroll_vx = 0;
            data->state.scroll_vy = 0;
            data->state.scroll_ema_vx = 0;
            data->state.scroll_ema_vy = 0;
            data->state.scroll_remainder_x_q8 = 0;
            data->state.scroll_remainder_y_q8 = 0;
            data->state.scroll_is_inertial = false;
            zmk_hid_mouse_scroll_set(0, 0);
        }
        k_mutex_unlock(&data->lock);
        LOG_DBG("Inertia cancelled by touch-down.");
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // --- MOUSE MOVEMENT ---
    if (event->code == INPUT_REL_X || event->code == INPUT_REL_Y) {
        int16_t val = (int16_t)event->value;

        k_mutex_lock(&data->lock, K_FOREVER);

        // Transition: If background inertia is running, clear it for a fresh manual start.
        // We check move_is_inertial to prevent resetting X when the Y event of the same packet arrives.
        if (data->state.move_active && data->state.move_is_inertial) {
            k_work_cancel_delayable(&data->move_work);
            data->state.move_active = false;
            data->state.move_vx = 0;
            data->state.move_vy = 0;
            data->state.move_ema_vx = 0;
            data->state.move_ema_vy = 0;
            data->state.move_remainder_x_q8 = 0;
            data->state.move_remainder_y_q8 = 0;
            data->state.move_is_inertial = false;
            // Just clear HID state. The standard ZMK pipeline will handle sending the current manual event.
            zmk_hid_mouse_movement_set(0, 0);
            LOG_DBG("Move Inertia cancelled by manual input.");
        }
        // Also cancel scroll inertia to prevent conflict
        if (data->state.scroll_active && data->state.scroll_is_inertial) {
            k_work_cancel_delayable(&data->scroll_work);
            data->state.scroll_active = false;
            data->state.scroll_vx = 0;
            data->state.scroll_vy = 0;
            data->state.scroll_ema_vx = 0;
            data->state.scroll_ema_vy = 0;
            data->state.scroll_remainder_x_q8 = 0;
            data->state.scroll_remainder_y_q8 = 0;
            data->state.scroll_is_inertial = false;
            zmk_hid_mouse_scroll_set(0, 0);
            LOG_DBG("Scroll Inertia cancelled by move input.");
        }

        if (event->code == INPUT_REL_X) {
            data->state.move_vx = val;
            data->state.move_ema_vx = (int16_t)((val + data->state.move_ema_vx) >> 1);
        }
        if (event->code == INPUT_REL_Y) {
            data->state.move_vy = val;
            data->state.move_ema_vy = (int16_t)((val + data->state.move_ema_vy) >> 1);
        }

        // Manual movement is NOT marked as is_inertial yet.
        // It becomes is_inertial only when the background timer takes over.
        // NOTE: If the finger stops and hardware sends slow events (e.g., 0), the timer is NOT rescheduled.
        // When the timer eventually expires, the low velocity is caught and inertia safely cancels.
        if (abs16(data->state.move_vx) >= cfg->move_threshold_start ||
            abs16(data->state.move_vy) >= cfg->move_threshold_start) {
            data->state.move_active = true;
            data->state.move_is_inertial = false; 
            k_work_reschedule(&data->move_work, K_MSEC(cfg->trigger_ms));
            LOG_DBG("Move Inertia triggered. x %d, y %d (trigger %d ms)", data->state.move_vx,
                    data->state.move_vy, cfg->trigger_ms);
        }
        k_mutex_unlock(&data->lock);
    }
    // --- SCROLLING ---
    else if (event->code == INPUT_REL_WHEEL || event->code == INPUT_REL_HWHEEL) {
        int16_t val = (int16_t)event->value;

        k_mutex_lock(&data->lock, K_FOREVER);

        if (data->state.scroll_active && data->state.scroll_is_inertial) {
            k_work_cancel_delayable(&data->scroll_work);
            data->state.scroll_active = false;
            data->state.scroll_vx = 0;
            data->state.scroll_vy = 0;
            data->state.scroll_ema_vx = 0;
            data->state.scroll_ema_vy = 0;
            data->state.scroll_remainder_x_q8 = 0;
            data->state.scroll_remainder_y_q8 = 0;
            data->state.scroll_is_inertial = false;
            zmk_hid_mouse_scroll_set(0, 0);
            LOG_DBG("Scroll Inertia cancelled by manual input.");
        }
        // Also cancel move inertia to prevent conflict
        if (data->state.move_active && data->state.move_is_inertial) {
            k_work_cancel_delayable(&data->move_work);
            data->state.move_active = false;
            data->state.move_vx = 0;
            data->state.move_vy = 0;
            data->state.move_ema_vx = 0;
            data->state.move_ema_vy = 0;
            data->state.move_remainder_x_q8 = 0;
            data->state.move_remainder_y_q8 = 0;
            data->state.move_is_inertial = false;
            zmk_hid_mouse_movement_set(0, 0);
            LOG_DBG("Move Inertia cancelled by scroll input.");
        }

        if (event->code == INPUT_REL_HWHEEL) {
            data->state.scroll_vx = val;
            data->state.scroll_ema_vx = (int16_t)((val + data->state.scroll_ema_vx) >> 1);
        }
        if (event->code == INPUT_REL_WHEEL) {
            data->state.scroll_vy = val;
            data->state.scroll_ema_vy = (int16_t)((val + data->state.scroll_ema_vy) >> 1);
        }

        // With touch events available, inertia starts on release (see scroll_start_on_release)
        // so the trigger timer can't fire mid-gesture while the fingers slow down.
        if (data->state.touch_aware && data->state.touching) {
            scroll_history_add(&data->state, event->code == INPUT_REL_HWHEEL ? val : 0,
                               event->code == INPUT_REL_WHEEL ? val : 0);
            k_mutex_unlock(&data->lock);
            return ZMK_INPUT_PROC_CONTINUE;
        }

        // Same logic: slow stops will bypass rescheduling, safely canceling inertia on timer expiry.
        if (abs16(data->state.scroll_vx) >= cfg->scroll_threshold_start ||
            abs16(data->state.scroll_vy) >= cfg->scroll_threshold_start) {
            data->state.scroll_active = true;
            data->state.scroll_is_inertial = false;
            k_work_reschedule(&data->scroll_work, K_MSEC(cfg->trigger_ms));
            LOG_DBG("Scroll Inertia triggered. h %d, v %d (trigger %d ms)", data->state.scroll_vx,
                    data->state.scroll_vy, cfg->trigger_ms);
        }
        k_mutex_unlock(&data->lock);
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static int inertia_init(const struct device *dev) {
    struct inertia_data *data = (struct inertia_data *)dev->data;
    data->dev = dev;

    // Reset move state
    data->state.move_vx = 0;
    data->state.move_vy = 0;
    data->state.move_ema_vx = 0;
    data->state.move_ema_vy = 0;
    data->state.move_remainder_x_q8 = 0;
    data->state.move_remainder_y_q8 = 0;
    data->state.move_active = false;
    data->state.move_is_inertial = false;

    // Reset scroll state
    data->state.scroll_vx = 0;
    data->state.scroll_vy = 0;
    data->state.scroll_ema_vx = 0;
    data->state.scroll_ema_vy = 0;
    data->state.scroll_remainder_x_q8 = 0;
    data->state.scroll_remainder_y_q8 = 0;
    data->state.scroll_active = false;
    data->state.scroll_is_inertial = false;

    k_mutex_init(&data->lock);
    k_work_init_delayable(&data->move_work, move_decay_callback);
    k_work_init_delayable(&data->scroll_work, scroll_decay_callback);
    return 0;
}

/* Driver API */
static const struct zmk_input_processor_driver_api inertia_driver_api = {
    .handle_event = inertia_handle_event,
};

#define INERTIA_INST(n)                                                                            \
    static struct inertia_data processor_inertia_data_##n = {};                                    \
    static const struct inertia_config processor_inertia_config_##n = {                            \
        .move_decay_factor_int =                                                                   \
            DT_INST_PROP_OR(n, move_decay_factor_int, DEFAULT_INERTIA_DECAY_FACTOR_INT),           \
        .move_interval_ms =                                                                        \
            DT_INST_PROP_OR(n, move_report_interval_ms, DEFAULT_INERTIA_INTERVAL_MS),              \
        .move_threshold_start =                                                                    \
            DT_INST_PROP_OR(n, move_threshold_start, DEFAULT_INERTIA_THRESHOLD_START),             \
        .move_threshold_stop =                                                                     \
            DT_INST_PROP_OR(n, move_threshold_stop, DEFAULT_INERTIA_THRESHOLD_STOP),               \
                                                                                                   \
        .scroll_decay_factor_int =                                                                 \
            DT_INST_PROP_OR(n, scroll_decay_factor_int, DEFAULT_INERTIA_SCROLL_DECAY_FACTOR_INT),  \
        .scroll_interval_ms =                                                                      \
            DT_INST_PROP_OR(n, scroll_report_interval_ms, DEFAULT_INERTIA_SCROLL_INTERVAL_MS),     \
        .scroll_threshold_start =                                                                  \
            DT_INST_PROP_OR(n, scroll_threshold_start, DEFAULT_INERTIA_SCROLL_THRESHOLD_START),    \
        .scroll_threshold_stop =                                                                   \
            DT_INST_PROP_OR(n, scroll_threshold_stop, DEFAULT_INERTIA_SCROLL_THRESHOLD_STOP),      \
        .scroll_velocity_window_ms = DT_INST_PROP(n, scroll_velocity_window_ms),                   \
        .scroll_velocity_history_ms = DT_INST_PROP(n, scroll_velocity_history_ms),                 \
        .trigger_ms = DT_INST_PROP_OR(n, trigger_ms, DEFAULT_TRIGGER_MS),                          \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, inertia_init, NULL, &processor_inertia_data_##n,                      \
                          &processor_inertia_config_##n, POST_KERNEL,                              \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &inertia_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INERTIA_INST)