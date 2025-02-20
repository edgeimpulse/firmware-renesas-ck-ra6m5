/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef INGESTION_SDK_PLATFORM_RENESAS_CK_RA6M5_EI_DEVICE_RENESAS_CK_RA6M5_H_
#define INGESTION_SDK_PLATFORM_RENESAS_CK_RA6M5_EI_DEVICE_RENESAS_CK_RA6M5_H_

/* Include ----------------------------------------------------------------- */
#include <renesas-ck-ra6m5/ei_memory_ck_ra6m5.h>
#include "ei_device_info_lib.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_fusion_sensors_config.h"

/** Baud rates */
#define DEFAULT_BAUD 115200

/** Max size for device id array */
#define EI_DEVICE_ID_MAX_SIZE 32

/** Sensors */
typedef enum
{
    MICROPHONE      = 0,
}used_sensors_t;

/** Number of sensors used without sensor fusioning */
#define EI_DEVICE_N_SENSORS            1    /* just the microphone */

class EiDeviceCKRA6M5: public EiDeviceInfo {
private:
    EiDeviceCKRA6M5() = delete;
    ei_device_sensor_t sensors[EI_DEVICE_N_SENSORS];
    EiDeviceMemory *data_flash;
    bool warmup_required;
    uint32_t warmup_time;
    bool is_sampling;
    
public:
    EiDeviceCKRA6M5(EiDeviceMemory* code_flash, EiDeviceMemory* data_flash_to_set);
    ~EiDeviceCKRA6M5();

    bool is_warmup_required(void);
    uint32_t get_warmup_time();
    void (*warmup)(void);

    void set_delay_for_sensor(const char *input_list);
    void clear_config(void);
    void init_device_id(void);
    uint32_t get_data_output_baudrate(void);
    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size);
    bool save_config(void);
    void load_config(void);
    bool test_flash(void);
    uint32_t read_raw(uint8_t *buffer, uint32_t pos, uint32_t bytes);
    void write_residual(void);
    void copy_device_info(char* device_id, char* device_type);

    void (*sample_read_callback)(void);
    void (*sample_multi_read_callback)(uint8_t);
    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms) override;
    bool stop_sample_thread(void) override;

#if MULTI_FREQ_ENABLED == 1
    bool start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned) override;
#endif

    void sample_thread(void);
};

/* Function prototypes ----------------------------------------------------- */
extern char ei_get_serial_byte(uint8_t is_inference_running);


#endif /* INGESTION_SDK_PLATFORM_RENESAS_CK_RA6M5_EI_DEVICE_RENESAS_CK_RA6M5_H_ */
