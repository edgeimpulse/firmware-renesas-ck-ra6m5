/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
