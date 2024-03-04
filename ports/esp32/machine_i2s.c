/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Mike Teachman
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// This file is never compiled standalone, it's included directly from
// extmod/machine_i2s.c via MICROPY_PY_MACHINE_I2S_INCLUDEFILE.

#include "py/mphal.h"

#include "driver/i2s.h"
#include "soc/i2s_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task.h"

#include <string.h>
#include "py/obj.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "mphalport.h"
#include "driver/i2s.h"


// Notes on this port's specific implementation of I2S:
// - a FreeRTOS task is created to implement the asynchronous background operations
// - a FreeRTOS queue is used to transfer the supplied buffer to the background task
// - all sample data transfers use DMA

#define I2S_TASK_PRIORITY        (ESP_TASK_PRIO_MIN + 1)
#define I2S_TASK_STACK_SIZE      (2048)

#define DMA_BUF_LEN_IN_I2S_FRAMES (256)

// The transform buffer is used with the readinto() method to bridge the opaque DMA memory on the ESP devices
// with the app buffer.  It facilitates audio sample transformations.  e.g.  32-bits samples to 16-bit samples.
// The size of 240 bytes is an engineering optimum that balances transfer performance with an acceptable use of heap space
#define SIZEOF_TRANSFORM_BUFFER_IN_BYTES (240)

#define I2S_NUM_MAX 2

typedef enum {
    I2S_TX_TRANSFER,
    I2S_RX_TRANSFER,
} direction_t;

typedef struct _non_blocking_descriptor_t {
    mp_buffer_info_t appbuf;
    mp_obj_t callback;
    direction_t direction;
} non_blocking_descriptor_t;

typedef struct _machine_i2s_obj_t {
    mp_obj_base_t base;
    i2s_port_t i2s_id;
    mp_hal_pin_obj_t sck;
    mp_hal_pin_obj_t ws;
    mp_hal_pin_obj_t sd;
    int8_t mode;
    i2s_bits_per_sample_t bits;
    format_t format;
    int32_t rate;
    int32_t ibuf;
    mp_obj_t callback_for_non_blocking;
    io_mode_t io_mode;
    uint8_t transform_buffer[SIZEOF_TRANSFORM_BUFFER_IN_BYTES];
    QueueHandle_t i2s_event_queue;
    QueueHandle_t non_blocking_mode_queue;
    TaskHandle_t non_blocking_mode_task;
} machine_i2s_obj_t;

STATIC mp_obj_t machine_i2s_deinit(mp_obj_t self_in);

// The frame map is used with the readinto() method to transform the audio sample data coming
// from DMA memory (32-bit stereo, with the L and R channels reversed) to the format specified
// in the I2S constructor.  e.g.  16-bit mono
STATIC const int8_t i2s_frame_map[NUM_I2S_USER_FORMATS][I2S_RX_FRAME_SIZE_IN_BYTES] = {
    { 6,  7, -1, -1, -1, -1, -1, -1 },  // Mono, 16-bits
    { 4,  5,  6,  7, -1, -1, -1, -1 },  // Mono, 32-bits
    { 6,  7,  2,  3, -1, -1, -1, -1 },  // Stereo, 16-bits
    { 4,  5,  6,  7,  0,  1,  2,  3 },  // Stereo, 32-bits
};

void machine_i2s_init0() {
    for (i2s_port_t p = 0; p < I2S_NUM_AUTO; p++) {
        MP_STATE_PORT(machine_i2s_obj)[p] = NULL;
    }
}

//  The following function takes a sample buffer and swaps L/R channels
//
//  Background:  For 32-bit stereo, the ESP-IDF API has a L/R channel orientation that breaks
//               convention with other ESP32 channel formats
//
//   appbuf[] = [L_0-7, L_8-15, L_16-23, L_24-31, R_0-7, R_8-15, R_16-23, R_24-31] = [Left channel, Right channel]
//   dma[] =    [R_0-7, R_8-15, R_16-23, R_24-31, L_0-7, L_8-15, L_16-23, L_24-31] = [Right channel, Left channel]
//
//   where:
//     L_0-7 is the least significant byte of the 32 bit sample in the Left channel
//     L_24-31 is the most significant byte of the 32 bit sample in the Left channel
//
//  Example:
//
//   appbuf[] = [0x99, 0xBB, 0x11, 0x22, 0x44, 0x55, 0xAB, 0x77] = [Left channel, Right channel]
//   dma[] =    [0x44, 0x55, 0xAB, 0x77, 0x99, 0xBB, 0x11, 0x22] = [Right channel,  Left channel]
//   where:
//      LEFT Channel =  0x99, 0xBB, 0x11, 0x22
//      RIGHT Channel = 0x44, 0x55, 0xAB, 0x77
//
//    samples in appbuf are in little endian format:
//       0x77 is the most significant byte of the 32-bit sample
//       0x44 is the least significant byte of the 32-bit sample
STATIC void swap_32_bit_stereo_channels(mp_buffer_info_t *bufinfo) {
    int32_t swap_sample;
    int32_t *sample = bufinfo->buf;
    uint32_t num_samples = bufinfo->len / 4;
    for (uint32_t i = 0; i < num_samples; i += 2) {
        swap_sample = sample[i + 1];
        sample[i + 1] = sample[i];
        sample[i] = swap_sample;
    }
}

STATIC int8_t get_frame_mapping_index(i2s_bits_per_sample_t bits, format_t format) {
    if (format == MONO) {
        if (bits == I2S_BITS_PER_SAMPLE_16BIT) {
            return 0;
        } else { // 32 bits
            return 1;
        }
    } else { // STEREO
        if (bits == I2S_BITS_PER_SAMPLE_16BIT) {
            return 2;
        } else { // 32 bits
            return 3;
        }
    }
}

STATIC i2s_bits_per_sample_t get_dma_bits(uint8_t mode, i2s_bits_per_sample_t bits) {
    if (mode == (I2S_MODE_MASTER | I2S_MODE_TX)) {
        return bits;
    } else { // Master Rx
        // read 32 bit samples for I2S hardware.  e.g. MEMS microphones
        return I2S_BITS_PER_SAMPLE_32BIT;
    }
}

STATIC i2s_channel_fmt_t get_dma_format(uint8_t mode, format_t format) {
    if (mode == (I2S_MODE_MASTER | I2S_MODE_TX)) {
        if (format == MONO) {
            return I2S_CHANNEL_FMT_ONLY_LEFT;
        } else {  // STEREO
            return I2S_CHANNEL_FMT_RIGHT_LEFT;
        }
    } else { // Master Rx
        // read stereo frames for all I2S hardware
        return I2S_CHANNEL_FMT_RIGHT_LEFT;
    }
}

STATIC uint32_t get_dma_buf_count(uint8_t mode, i2s_bits_per_sample_t bits, format_t format, int32_t ibuf) {
    // calculate how many DMA buffers need to be allocated
    uint32_t dma_frame_size_in_bytes =
        (get_dma_bits(mode, bits) / 8) * (get_dma_format(mode, format) == I2S_CHANNEL_FMT_RIGHT_LEFT ? 2: 1);

    uint32_t dma_buf_count = ibuf / (DMA_BUF_LEN_IN_I2S_FRAMES * dma_frame_size_in_bytes);

    return dma_buf_count;
}

STATIC uint32_t fill_appbuf_from_dma(machine_i2s_obj_t *self, mp_buffer_info_t *appbuf) {

    // copy audio samples from DMA memory to the app buffer
    // audio samples are read from DMA memory in chunks
    // loop, reading and copying chunks until the app buffer is filled
    // For asyncio mode, the loop will make an early exit if DMA memory becomes empty
    // Example:
    //   a MicroPython I2S object is configured for 16-bit mono (2 bytes per audio sample).
    //   For every frame coming from DMA (8 bytes), 2 bytes are "cherry picked" and
    //   copied to the supplied app buffer.
    //   Thus, for every 1 byte copied to the app buffer, 4 bytes are read from DMA memory.
    //   If a 8kB app buffer is supplied, 32kB of audio samples is read from DMA memory.

    uint32_t a_index = 0;
    uint8_t *app_p = appbuf->buf;
    uint8_t appbuf_sample_size_in_bytes = (self->bits / 8) * (self->format == STEREO ? 2: 1);
    uint32_t num_bytes_needed_from_dma = appbuf->len * (I2S_RX_FRAME_SIZE_IN_BYTES / appbuf_sample_size_in_bytes);
    while (num_bytes_needed_from_dma) {
        size_t num_bytes_requested_from_dma = MIN(sizeof(self->transform_buffer), num_bytes_needed_from_dma);
        size_t num_bytes_received_from_dma = 0;

        TickType_t delay;
        if (self->io_mode == ASYNCIO) {
            delay = 0; // stop i2s_read() operation if DMA memory becomes empty
        } else {
            delay = portMAX_DELAY;  // block until supplied buffer is filled
        }

        esp_err_t ret = i2s_read(
            self->i2s_id,
            self->transform_buffer,
            num_bytes_requested_from_dma,
            &num_bytes_received_from_dma,
            delay);
        check_esp_err(ret);

        // process the transform buffer one frame at a time.
        // copy selected bytes from the transform buffer into the user supplied appbuf.
        // Example:
        //   a MicroPython I2S object is configured for 16-bit mono.  This configuration associates to
        //   a frame map index of 0 = { 6,  7, -1, -1, -1, -1, -1, -1 } in the i2s_frame_map array
        //   This mapping indicates:
        //      appbuf[x+0] = frame[6]
        //      appbuf[x+1] = frame[7]
        //      frame bytes 0-5 are not used

        uint32_t t_index = 0;
        uint8_t f_index = get_frame_mapping_index(self->bits, self->format);
        while (t_index < num_bytes_received_from_dma) {
            uint8_t *transform_p = self->transform_buffer + t_index;

            for (uint8_t i = 0; i < I2S_RX_FRAME_SIZE_IN_BYTES; i++) {
                int8_t t_to_a_mapping = i2s_frame_map[f_index][i];
                if (t_to_a_mapping != -1) {
                    *app_p++ = transform_p[t_to_a_mapping];
                    a_index++;
                }
                t_index++;
            }
        }

        num_bytes_needed_from_dma -= num_bytes_received_from_dma;

        if ((self->io_mode == ASYNCIO) && (num_bytes_received_from_dma < num_bytes_requested_from_dma)) {
            // Unable to fill the entire app buffer from DMA memory.  This indicates all DMA RX buffers are empty.
            // Clear the I2S event queue so ioctl() indicates that the I2S object cannot currently
            // supply more audio samples
            xQueueReset(self->i2s_event_queue);
            break;
        }
    }

    return a_index;
}

STATIC size_t copy_appbuf_to_dma(machine_i2s_obj_t *self, mp_buffer_info_t *appbuf) {
    if ((self->bits == I2S_BITS_PER_SAMPLE_32BIT) && (self->format == STEREO)) {
        swap_32_bit_stereo_channels(appbuf);
    }

    size_t num_bytes_written = 0;

    TickType_t delay;
    if (self->io_mode == ASYNCIO) {
        delay = 0;  // stop i2s_write() operation if DMA memory becomes full
    } else {
        delay = portMAX_DELAY;  // block until supplied buffer is emptied
    }

    esp_err_t ret = i2s_write(self->i2s_id, appbuf->buf, appbuf->len, &num_bytes_written, delay);
    check_esp_err(ret);

    if ((self->io_mode == ASYNCIO) && (num_bytes_written < appbuf->len)) {
        // Unable to empty the entire app buffer into DMA memory.  This indicates all DMA TX buffers are full.
        // Clear the I2S event queue so ioctl() indicates that the I2S object cannot currently
        // accept more audio samples
        xQueueReset(self->i2s_event_queue);

        // Undo the swap transformation as the buffer has not been completely emptied.
        // The asyncio stream writer will use the same buffer in a future write call.
        if ((self->bits == I2S_BITS_PER_SAMPLE_32BIT) && (self->format == STEREO)) {
            swap_32_bit_stereo_channels(appbuf);
        }
    }
    return num_bytes_written;
}

// FreeRTOS task used for non-blocking mode
STATIC void task_for_non_blocking_mode(void *self_in) {
    machine_i2s_obj_t *self = (machine_i2s_obj_t *)self_in;

    non_blocking_descriptor_t descriptor;

    for (;;) {
        if (xQueueReceive(self->non_blocking_mode_queue, &descriptor, portMAX_DELAY)) {
            if (descriptor.direction == I2S_TX_TRANSFER) {
                copy_appbuf_to_dma(self, &descriptor.appbuf);
            } else { // RX
                fill_appbuf_from_dma(self, &descriptor.appbuf);
            }
            mp_sched_schedule(descriptor.callback, MP_OBJ_FROM_PTR(self));
        }
    }
}

STATIC void mp_machine_i2s_init_helper(machine_i2s_obj_t *self, mp_arg_val_t *args) {
    // are Pins valid?
    int8_t sck = args[ARG_sck].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_sck].u_obj);
    int8_t ws = args[ARG_ws].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_ws].u_obj);
    int8_t sd = args[ARG_sd].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_sd].u_obj);

    // is Mode valid?
    i2s_mode_t mode = args[ARG_mode].u_int;
    if ((mode != (I2S_MODE_MASTER | I2S_MODE_RX)) &&
        (mode != (I2S_MODE_MASTER | I2S_MODE_TX))) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid mode"));
    }

    // is Bits valid?
    i2s_bits_per_sample_t bits = args[ARG_bits].u_int;
    if ((bits != I2S_BITS_PER_SAMPLE_16BIT) &&
        (bits != I2S_BITS_PER_SAMPLE_32BIT)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid bits"));
    }

    // is Format valid?
    format_t format = args[ARG_format].u_int;
    if ((format != STEREO) &&
        (format != MONO)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid format"));
    }

    // is Rate valid?
    // Not checked:  ESP-IDF I2S API does not indicate a valid range for sample rate

    // is Ibuf valid?
    // Not checked: ESP-IDF I2S API will return error if requested buffer size exceeds available memory

    self->sck = sck;
    self->ws = ws;
    self->sd = sd;
    self->mode = mode;
    self->bits = bits;
    self->format = format;
    self->rate = args[ARG_rate].u_int;
    self->ibuf = args[ARG_ibuf].u_int;
    self->callback_for_non_blocking = MP_OBJ_NULL;
    self->i2s_event_queue = NULL;
    self->non_blocking_mode_queue = NULL;
    self->non_blocking_mode_task = NULL;
    self->io_mode = BLOCKING;

    i2s_config_t i2s_config;
    i2s_config.communication_format = I2S_COMM_FORMAT_I2S;
    i2s_config.mode = mode;
    i2s_config.bits_per_sample = get_dma_bits(mode, bits);
    i2s_config.channel_format = get_dma_format(mode, format);
    i2s_config.sample_rate = self->rate;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LOWMED;
    i2s_config.dma_buf_count = get_dma_buf_count(mode, bits, format, self->ibuf);
    i2s_config.dma_buf_len = DMA_BUF_LEN_IN_I2S_FRAMES;
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = true;
    i2s_config.fixed_mclk = 0;
    i2s_config.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    i2s_config.bits_per_chan = 0;

    // I2S queue size equals the number of DMA buffers
    check_esp_err(i2s_driver_install(self->i2s_id, &i2s_config, i2s_config.dma_buf_count, &self->i2s_event_queue));

    // apply low-level workaround for bug in some ESP-IDF versions that swap
    // the left and right channels
    // https://github.com/espressif/esp-idf/issues/6625
    #if CONFIG_IDF_TARGET_ESP32S3
    REG_SET_BIT(I2S_TX_CONF_REG(self->i2s_id), I2S_TX_MSB_SHIFT);
    REG_SET_BIT(I2S_TX_CONF_REG(self->i2s_id), I2S_RX_MSB_SHIFT);
    #else
    REG_SET_BIT(I2S_CONF_REG(self->i2s_id), I2S_TX_MSB_RIGHT);
    REG_SET_BIT(I2S_CONF_REG(self->i2s_id), I2S_RX_MSB_RIGHT);
    #endif

    i2s_pin_config_t pin_config;
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
    pin_config.bck_io_num = self->sck;
    pin_config.ws_io_num = self->ws;

    if (mode == (I2S_MODE_MASTER | I2S_MODE_RX)) {
        pin_config.data_in_num = self->sd;
        pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    } else { // TX
        pin_config.data_in_num = I2S_PIN_NO_CHANGE;
        pin_config.data_out_num = self->sd;
    }

    check_esp_err(i2s_set_pin(self->i2s_id, &pin_config));
}

STATIC machine_i2s_obj_t *mp_machine_i2s_make_new_instance(mp_int_t i2s_id) {
    if (i2s_id < 0 || i2s_id >= I2S_NUM_AUTO) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid id"));
    }

    machine_i2s_obj_t *self;
    if (MP_STATE_PORT(machine_i2s_obj)[i2s_id] == NULL) {
        self = m_new_obj_with_finaliser(machine_i2s_obj_t);
        self->base.type = &machine_i2s_type;
        MP_STATE_PORT(machine_i2s_obj)[i2s_id] = self;
        self->i2s_id = i2s_id;
    } else {
        self = MP_STATE_PORT(machine_i2s_obj)[i2s_id];
        machine_i2s_deinit(self);
    }

    return self;
}

STATIC void mp_machine_i2s_deinit(machine_i2s_obj_t *self) {
    i2s_driver_uninstall(self->i2s_id);

    if (self->non_blocking_mode_task != NULL) {
        vTaskDelete(self->non_blocking_mode_task);
        self->non_blocking_mode_task = NULL;
    }

    if (self->non_blocking_mode_queue != NULL) {
        vQueueDelete(self->non_blocking_mode_queue);
        self->non_blocking_mode_queue = NULL;
    }

    self->i2s_event_queue = NULL;
}

STATIC void mp_machine_i2s_irq_update(machine_i2s_obj_t *self) {
    if (self->io_mode == NON_BLOCKING) {
        // create a queue linking the MicroPython task to a FreeRTOS task
        // that manages the non blocking mode of operation
        self->non_blocking_mode_queue = xQueueCreate(1, sizeof(non_blocking_descriptor_t));

        // non-blocking mode requires a background FreeRTOS task
        if (xTaskCreatePinnedToCore(
            task_for_non_blocking_mode,
            "i2s_non_blocking",
            I2S_TASK_STACK_SIZE,
            self,
            I2S_TASK_PRIORITY,
            (TaskHandle_t *)&self->non_blocking_mode_task,
            MP_TASK_COREID) != pdPASS) {

            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create I2S task"));
        }
    } else {
        if (self->non_blocking_mode_task != NULL) {
            vTaskDelete(self->non_blocking_mode_task);
            self->non_blocking_mode_task = NULL;
        }

        if (self->non_blocking_mode_queue != NULL) {
            vQueueDelete(self->non_blocking_mode_queue);
            self->non_blocking_mode_queue = NULL;
        }
    }
}

MP_REGISTER_ROOT_POINTER(struct _machine_i2s_obj_t *machine_i2s_obj[I2S_NUM_AUTO]);




// Notes on naming conventions:
// 1. "id" versus "port"
//    The MicroPython API identifies instances of a peripheral using "id", while the ESP-IDF uses "port".
//    - for example, the first I2S peripheral on the ESP32 would be indicated by id=0 in MicroPython
//      and port=0 in ESP-IDF
// 2. any C type, macro, or function prefaced by "i2s" is associated with an ESP-IDF I2S interface definition
// 3. any C type, macro, or function prefaced by "machine_hw_i2s" is associated with the MicroPython implementation of I2S

typedef struct _machine_hw_i2s_obj_t {
    mp_obj_base_t          base;
    i2s_port_t             id;
    i2s_comm_format_t      standard;
    uint8_t                mode;
    i2s_bits_per_sample_t  dataformat;
    i2s_channel_fmt_t      channelformat;
    int32_t                samplerate;
    int16_t                dmacount;
    int16_t                dmalen;
    int32_t                apllrate;
    int8_t                 bck;
    int8_t                 ws;
    int8_t                 sdout;
    int8_t                 sdin;
    bool                   used;
} machine_hw_i2s_obj_t;

// Static object mapping to I2S peripherals
//   note:  I2S implementation makes use of the following mapping between I2S peripheral and I2S object
//      I2S peripheral 1:  machine_hw_i2s_obj[0]
//      I2S peripheral 2:  machine_hw_i2s_obj[1]
STATIC machine_hw_i2s_obj_t machine_hw_i2s_obj[I2S_NUM_MAX] = {
        [0].used = false,
        [1].used = false };

STATIC void machine_hw_i2s_init_helper(machine_hw_i2s_obj_t *self, size_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    enum {
        ARG_bck,
        ARG_ws,
        ARG_sdout,
        ARG_sdin,
        ARG_standard,
        ARG_mode,
        ARG_dataformat,
        ARG_channelformat,
        ARG_samplerate,
        ARG_dmacount,
        ARG_dmalen,
        ARG_apllrate,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bck,              MP_ARG_KW_ONLY                   | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_ws,               MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sdout,            MP_ARG_KW_ONLY                   | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sdin,             MP_ARG_KW_ONLY                   | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_standard,         MP_ARG_KW_ONLY                   | MP_ARG_INT,   {.u_int = I2S_COMM_FORMAT_I2S} },
        { MP_QSTR_mode,             MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_dataformat,       MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_channelformat,    MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_samplerate,       MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = -1} },
        { MP_QSTR_dmacount,         MP_ARG_KW_ONLY                   | MP_ARG_INT,   {.u_int = 16} },
        { MP_QSTR_dmalen,           MP_ARG_KW_ONLY                   | MP_ARG_INT,   {.u_int = 64} },
        { MP_QSTR_apllrate,         MP_ARG_KW_ONLY                   | MP_ARG_INT,   {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    
    //
    // ---- Check validity of arguments ----
    //

    // are I2S pin assignments valid?
    int8_t bck = args[ARG_bck].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_bck].u_obj);
    int8_t ws = args[ARG_ws].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_ws].u_obj);
    int8_t sdin = args[ARG_sdin].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_sdin].u_obj);
    int8_t sdout = args[ARG_sdout].u_obj == MP_OBJ_NULL ? -1 : machine_pin_get_id(args[ARG_sdout].u_obj);

    if ((sdin == -1) && (args[ARG_mode].u_int == (I2S_MODE_MASTER | I2S_MODE_RX))) {
        mp_raise_ValueError(MP_ERROR_TEXT("sdin must be specified for RX mode"));
    }

    if ((sdin == -1) && (args[ARG_mode].u_int == (I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM))) {
        mp_raise_ValueError(MP_ERROR_TEXT("sdin must be specified for PDM mode"));
    }

    if ((sdout == -1) && (args[ARG_mode].u_int == (I2S_MODE_MASTER | I2S_MODE_TX))) {
        mp_raise_ValueError(MP_ERROR_TEXT("sdout must be specified for TX mode"));
    }

    if ((sdin != -1) && (sdout != -1)) {
        mp_raise_ValueError(MP_ERROR_TEXT("only one of sdin or sdout can be specified"));
    }

    if ((bck == -1) && (args[ARG_mode].u_int == (I2S_MODE_MASTER | I2S_MODE_RX) || args[ARG_mode].u_int == (I2S_MODE_MASTER | I2S_MODE_TX))) {
        mp_raise_ValueError(MP_ERROR_TEXT("bck must be specified for RX/TX mode"));
    }

    // if pdm only I2S_NUM_0 supported
    if ((self->id != 0) && (args[ARG_mode].u_int == (I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM))) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S.NS0 only supports PDM mode"));
    }

    // is Standard valid?
    i2s_comm_format_t i2s_commformat = args[ARG_standard].u_int;
    if ((i2s_commformat != I2S_COMM_FORMAT_I2S) &&
        (i2s_commformat != (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB))) {
        mp_raise_ValueError(MP_ERROR_TEXT("Standard is not valid"));
    }

    // is Mode valid?
    i2s_mode_t i2s_mode = args[ARG_mode].u_int;
    if ((i2s_mode != (I2S_MODE_MASTER | I2S_MODE_RX)) &&
        (i2s_mode != (I2S_MODE_MASTER | I2S_MODE_TX)) &&
        (i2s_mode != (I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM))) {
        mp_raise_ValueError(MP_ERROR_TEXT("Only Master Rx, Master Tx, Master PDM Modes are supported"));
    }

    // is Data Format valid?
    i2s_bits_per_sample_t i2s_bits_per_sample = args[ARG_dataformat].u_int;
    if ((i2s_bits_per_sample != I2S_BITS_PER_SAMPLE_16BIT) &&
        (i2s_bits_per_sample != I2S_BITS_PER_SAMPLE_24BIT) &&
        (i2s_bits_per_sample != I2S_BITS_PER_SAMPLE_32BIT)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Data Format is not valid"));
    }

    // is Channel Format valid?
    i2s_channel_fmt_t i2s_channelformat = args[ARG_channelformat].u_int;
    if ((i2s_channelformat != I2S_CHANNEL_FMT_RIGHT_LEFT) &&
        (i2s_channelformat != I2S_CHANNEL_FMT_ALL_RIGHT) &&
        (i2s_channelformat != I2S_CHANNEL_FMT_ALL_LEFT) &&
        (i2s_channelformat != I2S_CHANNEL_FMT_ONLY_RIGHT) &&
        (i2s_channelformat != I2S_CHANNEL_FMT_ONLY_LEFT)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Channel Format is not valid"));
    }

    // is Sample Rate valid?
    // No validation done:  ESP-IDF API does not indicate a valid range for sample rate

    // is DMA Buffer Count valid?
    // ESP-IDF API code checks for buffer count in this range:  [2, 128]
    int16_t i2s_dmacount = args[ARG_dmacount].u_int;
    if ((i2s_dmacount < 2) || (i2s_dmacount > 128)) {
        mp_raise_ValueError(MP_ERROR_TEXT("DMA Buffer Count is not valid.  Allowed range is [2, 128]"));
    }

    // is DMA Buffer Length valid?
    // ESP-IDF API code checks for buffer length in this range:  [8, 1024]
    int16_t i2s_dmalen = args[ARG_dmalen].u_int;
    if ((i2s_dmalen < 8) || (i2s_dmalen > 1024)) {
        mp_raise_ValueError(MP_ERROR_TEXT("DMA Buffer Length is not valid.  Allowed range is [8, 1024]"));
    }

    // is APLL Rate valid?
    // No validation done:  ESP-IDF API does not indicate a valid range for APLL rate
    

    self->bck = bck;
    self->ws = ws;
    self->sdout = sdout;
    self->sdin = sdin;
    self->standard = args[ARG_standard].u_int;
    self->mode = args[ARG_mode].u_int;
    self->dataformat = args[ARG_dataformat].u_int;
    self->channelformat = args[ARG_channelformat].u_int;
    self->samplerate = args[ARG_samplerate].u_int;
    self->dmacount = args[ARG_dmacount].u_int;
    self->dmalen = args[ARG_dmalen].u_int;
    self->apllrate = args[ARG_apllrate].u_int;

    i2s_config_t i2s_config;
    i2s_config.communication_format = self->standard;
    i2s_config.mode = self->mode;
    i2s_config.bits_per_sample = self->dataformat;
    i2s_config.channel_format = self->channelformat;
    i2s_config.sample_rate = self->samplerate;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = self->dmacount;
    i2s_config.dma_buf_len = self->dmalen;
    if (self->apllrate != 0) {
        i2s_config.use_apll = true;
    } else {
        i2s_config.use_apll = false;
        i2s_config.tx_desc_auto_clear = true;
    }
    i2s_config.fixed_mclk = self->apllrate;

    // uninstall I2S driver when changes are being made to an active I2S peripheral
    if (self->used) {
        i2s_driver_uninstall(self->id);
    }

    esp_err_t ret = i2s_driver_install(self->id, &i2s_config, 0, NULL);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install: Parameter error"));
            break;
        case ESP_ERR_NO_MEM:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install: Out of memory"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S driver install: Undocumented error")); 
            break;
    }

    i2s_pin_config_t pin_config;
    pin_config.bck_io_num = self->bck;
    pin_config.ws_io_num = self->ws;
    pin_config.data_out_num = self->sdout;
    pin_config.data_in_num = self->sdin;

    ret = i2s_set_pin(self->id, &pin_config);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin: Parameter error"));
            break;
        case ESP_FAIL:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin: IO error"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S set pin: Undocumented error")); 
            break;
    }

    self->used = true;
}

/******************************************************************************/
// MicroPython bindings for I2S
STATIC void machine_hw_i2s_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_hw_i2s_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "I2S(id=%u, bck=%d, ws=%d, sdout=%d, sdin=%d\n"
            "standard=%u, mode=%u,\n"
            "dataformat=%u, channelformat=%u,\n"
            "samplerate=%d,\n"
            "dmacount=%d, dmalen=%d,\n"
            "apllrate=%d)",
            self->id, self->bck, self->ws, self->sdout, self->sdin,
            self->standard, self->mode,
            self->dataformat, self->channelformat,
            self->samplerate,
            self->dmacount, self->dmalen,
            self->apllrate
            );
}

STATIC mp_obj_t machine_hw_i2s_make_new(const mp_obj_type_t *type, size_t n_pos_args, size_t n_kw_args, const mp_obj_t *args) {
    mp_arg_check_num(n_pos_args, n_kw_args, 1, MP_OBJ_FUN_ARGS_MAX, true);

    machine_hw_i2s_obj_t *self;

    // note: it is safe to assume that the arg pointer below references a positional argument because the arg check above
    //       guarantees that at least one positional argument has been provided
    i2s_port_t i2s_id = mp_obj_get_int(args[0]);
    if (i2s_id == I2S_NUM_0) {
        self = &machine_hw_i2s_obj[0];
    } else if (i2s_id == I2S_NUM_1) {
        self = &machine_hw_i2s_obj[1];
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S ID is not valid"));
    }

    self->base.type = &machine_hw_i2s_type;
    self->id = i2s_id;

    // is I2S peripheral already in use?
    if (self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is already in use"));
    }

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw_args, args + n_pos_args);
    // note:  "args + 1" below has the effect of skipping over the ID argument
    machine_hw_i2s_init_helper(self, n_pos_args - 1, args + 1, &kw_args);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t machine_hw_i2s_init(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // note:  "pos_args + 1" below has the effect of skipping over "self"
    machine_hw_i2s_init_helper(pos_args[0], n_pos_args - 1, pos_args + 1, kw_args);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_hw_i2s_init_obj, 1, machine_hw_i2s_init);

STATIC mp_obj_t machine_hw_i2s_readinto(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buf, ARG_timeout };
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_buf,                      MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY                   | MP_ARG_INT,  {.u_int = -1} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    machine_hw_i2s_obj_t *self = pos_args[0];
    
    if (!self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is not initialized"));
    }

    if (self->mode != (I2S_MODE_MASTER | I2S_MODE_RX) && self->mode != (I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM)) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S not configured for read method"));
    }

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_WRITE);

    TickType_t timeout_in_ticks = portMAX_DELAY;
    if (args[ARG_timeout].u_int != -1) {
        timeout_in_ticks = pdMS_TO_TICKS(args[ARG_timeout].u_int);
    }

    uint32_t num_bytes_read = 0;
    esp_err_t ret = i2s_read(self->id, bufinfo.buf, bufinfo.len, &num_bytes_read, timeout_in_ticks);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S read: Parameter error"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S read: Undocumented error")); 
            break;
    }

    return mp_obj_new_int(num_bytes_read);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_hw_i2s_readinto_obj, 2, machine_hw_i2s_readinto);

STATIC mp_obj_t machine_hw_i2s_write(mp_uint_t n_pos_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_buf, ARG_timeout };
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_buf, MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_pos_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    machine_hw_i2s_obj_t *self = pos_args[0];

    if (!self->used) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S port is not initialized"));
    }

    if (self->mode != (I2S_MODE_MASTER | I2S_MODE_TX)) {
        mp_raise_ValueError(MP_ERROR_TEXT("I2S not configured for write method"));
    }
    
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_WRITE);

    TickType_t timeout_in_ticks = portMAX_DELAY;
    if (args[ARG_timeout].u_int != -1) {
        timeout_in_ticks = pdMS_TO_TICKS(args[ARG_timeout].u_int);
    }

    uint32_t num_bytes_written = 0;
    esp_err_t ret = i2s_write(self->id, bufinfo.buf, bufinfo.len, &num_bytes_written, timeout_in_ticks);
    switch (ret) {
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S write: Parameter error"));
            break;
        default:
            // this error not documented in ESP-IDF
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2S write: Undocumented error")); 
            break;
    }
    
    return mp_obj_new_int(num_bytes_written);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_hw_i2s_write_obj, 2, machine_hw_i2s_write);

STATIC mp_obj_t machine_hw_i2s_deinit(mp_obj_t self_in) {
    machine_hw_i2s_obj_t *self = self_in;
    i2s_driver_uninstall(self->id);
    self->used = false;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_i2s_deinit_obj, machine_hw_i2s_deinit);

STATIC const mp_rom_map_elem_t machine_hw_i2s_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_init),            MP_ROM_PTR(&machine_hw_i2s_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto),        MP_ROM_PTR(&machine_hw_i2s_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),           MP_ROM_PTR(&machine_hw_i2s_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&machine_hw_i2s_deinit_obj) },

    // Constants
    { MP_ROM_QSTR(MP_QSTR_NUM0),            MP_ROM_INT(I2S_NUM_0) },
    { MP_ROM_QSTR(MP_QSTR_NUM1),            MP_ROM_INT(I2S_NUM_1) },
    { MP_ROM_QSTR(MP_QSTR_PHILIPS),         MP_ROM_INT(I2S_COMM_FORMAT_I2S) },
    { MP_ROM_QSTR(MP_QSTR_LSB),             MP_ROM_INT(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB) },
    { MP_ROM_QSTR(MP_QSTR_NO_PIN),          MP_ROM_INT(I2S_PIN_NO_CHANGE) },
    // note:  ESP-IDF does not implement the MSB standard (even though the Macro I2S_COMM_FORMAT_I2S_MSB is defined)
    { MP_ROM_QSTR(MP_QSTR_MASTER_RX),       MP_ROM_INT(I2S_MODE_MASTER | I2S_MODE_RX) },
    { MP_ROM_QSTR(MP_QSTR_MASTER_TX),       MP_ROM_INT(I2S_MODE_MASTER | I2S_MODE_TX) },
    { MP_ROM_QSTR(MP_QSTR_MASTER_PDW),      MP_ROM_INT(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM) },
    { MP_ROM_QSTR(MP_QSTR_B16),             MP_ROM_INT(I2S_BITS_PER_SAMPLE_16BIT) },
    { MP_ROM_QSTR(MP_QSTR_B24),             MP_ROM_INT(I2S_BITS_PER_SAMPLE_24BIT) },
    { MP_ROM_QSTR(MP_QSTR_B32),             MP_ROM_INT(I2S_BITS_PER_SAMPLE_32BIT) },
    { MP_ROM_QSTR(MP_QSTR_RIGHT_LEFT),      MP_ROM_INT(I2S_CHANNEL_FMT_RIGHT_LEFT) },
    { MP_ROM_QSTR(MP_QSTR_ALL_RIGHT),       MP_ROM_INT(I2S_CHANNEL_FMT_ALL_RIGHT) },
    { MP_ROM_QSTR(MP_QSTR_ALL_LEFT),        MP_ROM_INT(I2S_CHANNEL_FMT_ALL_LEFT) },
    { MP_ROM_QSTR(MP_QSTR_ONLY_RIGHT),      MP_ROM_INT(I2S_CHANNEL_FMT_ONLY_RIGHT) },
    { MP_ROM_QSTR(MP_QSTR_ONLY_LEFT),       MP_ROM_INT(I2S_CHANNEL_FMT_ONLY_LEFT) },
};
MP_DEFINE_CONST_DICT(machine_hw_i2s_locals_dict, machine_hw_i2s_locals_dict_table);

//const mp_obj_type_t machine_hw_i2s_type = {
//    { &mp_type_type },
//    .name = MP_QSTR_I2S,
//    .print = machine_hw_i2s_print,
//    .make_new = machine_hw_i2s_make_new,
//    .locals_dict = (mp_obj_dict_t *) &machine_hw_i2s_locals_dict,
//};

MP_DEFINE_CONST_OBJ_TYPE(
    machine_hw_i2s_type,
    MP_QSTR_I2S,
    MP_TYPE_FLAG_NONE,
    make_new, machine_hw_i2s_make_new,
    print, machine_hw_i2s_print,
    locals_dict, (mp_obj_dict_t *) &machine_hw_i2s_locals_dict
    );
