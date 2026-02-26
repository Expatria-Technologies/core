/*
  encoders.c - quadrature encoders interface (API)

  Part of grblHAL

  Copyright (c) 2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _ENCODERS_H_
#define _ENCODERS_H_

#include "plugins.h"

// Quadrature encoder interface

typedef union {
    uint8_t value;
    uint8_t events;
    struct {
        uint8_t position_changed  :1,
                direction_changed :1,
                click             :1,
                dbl_click         :1,
                long_click        :1,
                index_pulse       :1,
                unused            :2;
    };
} encoder_event_t;

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t bidirectional :1,
                select        :1,
                index         :1,
                spindle_rpm   :1,
                spindle_pos   :1,
                unused        :3;
    };
} encoder_caps_t;

typedef struct {
    uint32_t vel_timeout;
    uint32_t dbl_click_window;  //!< ms.
} encoder_cfg_t;

typedef struct {
    int32_t position;
    uint32_t velocity;
} encoder_data_t;

struct encoder;
typedef struct encoder encoder_t;

/*! \brief Pointer to callback function to receive encoder events.
\param encoder pointer to a \a encoder_t struct.
\param events pointer to a \a encoder_event_t struct.
\param context pointer to the context passed to the encoders claim function.
*/
typedef void (*encoder_on_event_ptr)(encoder_t *encoder, encoder_event_t *events, void *context);

/*! \brief Pointer to function for resetting encoder data.
\param encoder pointer to a \a encoder_t struct.
*/
typedef void (*encoder_reset_ptr)(encoder_t *encoder);

/*! \brief Pointer to function for claiming an encoder.
\param event_handler pointer to to the event handler callback.
\param context pointer to the context to be passed to event handler.
\returns \a true when claim was successful, \a false to otherwise.
*/
typedef bool (*encoder_claim_ptr)(encoder_on_event_ptr event_handler, void *context);

/*! \brief Pointer to function for getting encoder data.
\param encoder pointer to a \a encoder_t struct.
\returns pointer to a \a encoder_data_t struct containing the data.
*/
typedef encoder_data_t *(*encoder_get_data_ptr)(encoder_t *encoder);

/*! \brief Pointer to the callbak function to be called by encoders_enumerate().
\param encoder pointer to a \a encoder_t struct.
\returns \a true to stop the enumeration and return true from encoders_enumerate(), \a false otherwise.
*/
typedef bool (*encoder_enumerate_callback_ptr)(encoder_t *encoder, void *data);

/*! \brief Pointer to function for configuring an encoder.
\param encoder pointer to a \a encoder_t struct.
\param encoder pointer to a \a encoder_cfg_t struct.
\returns \a true when claim was successful, \a false to otherwise.
*/
typedef bool (*encoder_configure_ptr)(encoder_t *encoder, encoder_cfg_t *settings);

void encoder_register (encoder_t *encoder);
bool encoders_enumerate (encoder_enumerate_callback_ptr callback, void *data);
uint8_t encoders_get_count (void);

struct encoder {
    encoder_caps_t caps;
    encoder_claim_ptr claim;
    encoder_reset_ptr reset;
    encoder_get_data_ptr get_data;
    encoder_configure_ptr configure;
};

#endif // _ENCODERS_H_

// Quadrature Encoder Interface - static code for drivers/plugins

#if QEI_ENABLE && defined(QEI_A_PIN) && defined(QEI_B_PIN)

typedef enum {
   QEI_DirUnknown = 0,
   QEI_DirCW,
   QEI_DirCCW
} qei_dir_t;

typedef union {
    uint_fast8_t pins;
    struct {
        uint_fast8_t a :1,
                     b :1;
    };
} qei_state_t;

typedef struct {
    encoder_t encoder;
    encoder_data_t data;
    encoder_event_t event;
    void *context;
    int32_t vel_count;
    uint_fast16_t state;
    qei_dir_t dir;
    uint8_t port_a, port_b, port_select;
    volatile uint32_t dbl_click_timeout;
    volatile uint32_t vel_timeout;
    uint32_t vel_timestamp;
    encoder_on_event_ptr on_event;
    encoder_cfg_t settings;
} qei_t;

static qei_t qei = {
    .port_a = IOPORT_UNASSIGNED,
    .port_b = IOPORT_UNASSIGNED,
    .port_select = IOPORT_UNASSIGNED,
    .settings.dbl_click_window = 500,
    .encoder.caps.bidirectional = On
};

static void qei_post_event (void *data)
{
    qei.on_event(&qei.encoder, &qei.event, qei.context);
}

static void qei_dblclk_event (void *data)
{
    qei.event.dbl_click = On;
    qei.on_event(&qei.encoder, &qei.event, qei.context);
}

static void qei_reset (encoder_t *encoder)
{
    qei.vel_timeout = 0;
    qei.dir = QEI_DirUnknown;
    qei.data.position = qei.vel_count = 0;
    qei.vel_timestamp = hal.get_elapsed_ticks();
    qei.vel_timeout = qei.settings.vel_timeout;
}

static bool qei_configure (encoder_t *encoder, encoder_cfg_t *settings)
{
    if(qei.vel_timeout != settings->vel_timeout)
        qei.vel_timestamp = hal.get_elapsed_ticks();

    memcpy(&qei.settings, settings, sizeof(encoder_cfg_t));

    return true;
}

static encoder_data_t *qei_get_data (encoder_t *encoder)
{
    return &qei.data;
}

static void qei_poll (void *data)
{
    if(qei.vel_timeout && !(--qei.vel_timeout)) {

        uint32_t time = hal.get_elapsed_ticks();

        qei.data.velocity = abs(qei.data.position - qei.vel_count) * 1000 / (time - qei.vel_timestamp);
        qei.vel_timestamp = time;
        qei.vel_timeout = qei.settings.vel_timeout;
        if((qei.event.position_changed = !qei.dbl_click_timeout || qei.data.velocity == 0))
            qei.on_event(&qei.encoder, &qei.event, qei.context);
        qei.vel_count = qei.data.position;
    }

    if(qei.dbl_click_timeout && !(--qei.dbl_click_timeout)) {
        qei.event.click = On;
        task_delete(qei_dblclk_event, NULL);
        qei.on_event(&qei.encoder, &qei.event, qei.context);
    }
}

static void qei_ab_irq (uint8_t port, bool high)
{
    const uint8_t encoder_valid_state[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

    static qei_state_t state = {0};

    if(port == qei.port_a)
        state.a = high;
    else
        state.b = high;

    uint_fast8_t idx = (((qei.state << 2) & 0x0F) | state.pins);

    if(encoder_valid_state[idx] ) {

//        int32_t count = qei.count;

        qei.state = ((qei.state << 4) | idx) & 0xFF;

        if(qei.state == 0x42 || qei.state == 0xD4 || qei.state == 0x2B || qei.state == 0xBD) {
            qei.data.position--;
            if(qei.vel_timeout == 0 || qei.dir == QEI_DirCW) {
                qei.dir = QEI_DirCCW;
                qei.event.position_changed = On;
                task_add_immediate(qei_post_event, NULL);
            }
        } else if(qei.state == 0x81 || qei.state == 0x17 || qei.state == 0xE8 || qei.state == 0x7E) {
            qei.data.position++;
            if(qei.vel_timeout == 0 || qei.dir == QEI_DirCCW) {
                qei.dir = QEI_DirCW;
                qei.event.position_changed = On;
                task_add_immediate(qei_post_event, NULL);
            }
        }
    }
}

static void qei_select_irq (uint8_t port, bool high)
{
    if(high)
        return;

    if(!qei.dbl_click_timeout) {
        qei.dbl_click_timeout = qei.settings.dbl_click_window;
    } else if(qei.dbl_click_timeout < qei.settings.dbl_click_window) {
        qei.dbl_click_timeout = 0;
        task_delete(qei_dblclk_event, NULL);
        task_add_immediate(qei_dblclk_event, NULL);
    }
}

static bool qei_claim (encoder_on_event_ptr event_handler, void *context)
{
    if(event_handler == NULL || qei.on_event)
        return false;

    qei.context = context;
    qei.on_event = event_handler;
    qei.encoder.reset = qei_reset;
    qei.encoder.get_data = qei_get_data;
    qei.encoder.configure = qei_configure;

    if(qei.port_b != IOPORT_UNASSIGNED) {
        ioport_enable_irq(qei.port_a, IRQ_Mode_Change, qei_ab_irq);
        ioport_enable_irq(qei.port_b, IRQ_Mode_Change, qei_ab_irq);
    }

    if(qei.port_select != IOPORT_UNASSIGNED)
        ioport_enable_irq(qei.port_select, IRQ_Mode_Change, qei_select_irq);

    task_add_systick(qei_poll, NULL);

    return true;
}

static inline void encoder_pin_claimed (uint8_t port, xbar_t *pin)
{
    switch(pin->function) {

        case Input_QEI_A:
            qei.port_a = port;
            break;

        case Input_QEI_B:
            qei.port_b = port;
            qei.encoder.claim = qei_claim;
            if(qei.port_a != IOPORT_UNASSIGNED)
                encoder_register(&qei.encoder);
            break;

        case Input_QEI_Select:
            qei.port_select = port;
            qei.encoder.caps.select = On;
            break;

        default: break;
    }
}

#endif
