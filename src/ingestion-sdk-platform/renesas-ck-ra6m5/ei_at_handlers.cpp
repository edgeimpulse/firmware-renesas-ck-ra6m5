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
/* Include */
#include "ei_at_handlers.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_fusion.h"
#include "ei_device_lib.h"
#include "inference/ei_run_impulse.h"
#include "sensors/ei_air_quality_indoor.h"
#include "sensors/ei_air_quality_outdoor.h"

#define AT_WARMUP_ARGS              "SENSOR"
#define WARMUP_INDOOR_INDEX         0
#define WARMUP_OUTDOOR_INDEX        1

EiDeviceCKRA6M5 *pei_device;

/* Private function declaration */
static bool at_list_config(void);
static bool at_clear_config(void);

static bool at_device_info(void);
static bool at_get_sample_settings(void);
static bool at_set_sample_settings(const char **argv, const int argc);
static bool at_get_upload_settings(void);
static bool at_set_upload_settings(const char **argv, const int argc);
static bool at_set_upload_host(const char **argv, const int argc);
static bool at_list_sensors(void);
static bool at_list_fusion_sensors(void);

static bool at_read_buffer(const char **argv, const int argc);

static bool at_sample_start(const char **argv, const int argc);

static bool at_unlink_file(const char **argv, const int argc);
static bool at_read_raw(const char **argv, const int argc);

static bool at_run_nn_normal(void);
static bool at_run_nn_normal_cont(void);

static bool at_get_mgmt_settings(void);
static bool at_set_mgmt_settings(const char **argv, const int argc);

static bool at_warmup_get(void);
static bool at_warmup_set(const char **argv, const int argc);

static inline bool check_args_num(const int &required, const int &received);

/* Public function definition */
/**
 *
 * @return
 */
ATServer *ei_at_init(EiDeviceCKRA6M5 *ei_device)
{
  ATServer *at;

  at = ATServer::get_instance();
  pei_device = ei_device;

  at->register_command(AT_CONFIG, AT_CONFIG_HELP_TEXT, nullptr, at_list_config, nullptr, nullptr);
  at->register_command(AT_CLEARCONFIG, AT_CLEARCONFIG_HELP_TEXT, at_clear_config, nullptr, nullptr, nullptr);
  at->register_command(AT_SAMPLESTART, AT_SAMPLESTART_HELP_TEXT, nullptr, nullptr, at_sample_start, AT_SAMPLESTART_ARGS);
  at->register_command(AT_SAMPLESETTINGS, AT_SAMPLESETTINGS_HELP_TEXT, nullptr, at_get_sample_settings, at_set_sample_settings, AT_SAMPLESETTINGS_ARGS);
  at->register_command(AT_RUNIMPULSE, AT_RUNIMPULSE_HELP_TEXT, at_run_nn_normal, nullptr, nullptr, nullptr);
  at->register_command(AT_RUNIMPULSECONT, AT_RUNIMPULSECONT_HELP_TEXT, at_run_nn_normal_cont, nullptr, nullptr, nullptr);
  at->register_command(AT_READBUFFER, AT_READBUFFER_HELP_TEXT, nullptr, nullptr, at_read_buffer, AT_READBUFFER_ARGS);
  at->register_command(AT_MGMTSETTINGS, AT_MGMTSETTINGS_HELP_TEXT, nullptr, at_get_mgmt_settings, at_set_mgmt_settings, AT_MGMTSETTINGS_ARGS);
  at->register_command(AT_UPLOADSETTINGS, AT_UPLOADSETTINGS_HELP_TEXT, nullptr, at_get_upload_settings, at_set_upload_settings, AT_UPLOADSETTINGS_ARGS);
  at->register_command(AT_UPLOADHOST, AT_UPLOADHOST_HELP_TEXT, nullptr, nullptr, at_set_upload_host, AT_UPLOADHOST_ARGS);
  at->register_command(AT_READRAW, AT_READRAW_HELP_TEXT, nullptr, nullptr, at_read_raw, AT_READRAW_ARS);
  at->register_command(AT_UNLINKFILE, AT_UNLINKFILE_HELP_TEXT, nullptr, nullptr, at_unlink_file, AT_UNLINKFILE_ARGS);

  at->register_command("WARMUP", "Set or get warmup", nullptr, at_warmup_get, at_warmup_set, nullptr);

  return at;
}

static bool at_warmup_get(void)
{
    ei_printf("No warmup ongoing\r\n");

    return true;
}

static bool at_warmup_set(const char **argv, const int argc)
{
    uint32_t start_time = 0;
    uint32_t stop_time = 0;

    if(argc < 1) {
        ei_printf("Missing sensor name!\n");
        return true;
    }

    uint8_t sensor_index = (uint8_t)atoi(argv[0]);

    if (sensor_index == WARMUP_INDOOR_INDEX)
    {
        ei_printf("Launching warmup!\n");
        start_time = ei_read_timer_ms();
        ei_air_quality_indoor_warmup();
        stop_time = ei_read_timer_ms();
        ei_printf("Warmup completed in %ld ms!\n", (stop_time - start_time));
    }
    else if (sensor_index == WARMUP_OUTDOOR_INDEX)
    {
        ei_printf("Launching warmup!\n");
        start_time = ei_read_timer_ms();
        ei_air_quality_outdoor_warmup();
        stop_time = ei_read_timer_ms();
        ei_printf("Warmup completed in %ld ms!\n", (stop_time - start_time));
    }
    else
    {
        ei_printf("Wrong sensor index!\n");
    }

    return true;
}

/* Private function definition */
/**
 *
 * @return
 */
static bool at_list_config(void)
{
    ei_printf("===== Device info =====\n");
    at_device_info();
    ei_printf("\n");
    ei_printf("===== Sensors ======\n");
    at_list_sensors();
    at_list_fusion_sensors();
    ei_printf("\n");
    ei_printf("===== Snapshot ======\n");
    ei_printf("Has snapshot:    0\n");
    ei_printf("\n");
    ei_printf("===== WIFI =====\n");
    ei_printf("SSID:      \n");
    ei_printf("Password:  \n");
    ei_printf("Security:  0\n");
    ei_printf("MAC:       00:00:00:00:00:00\n");
    ei_printf("Connected: 0\n");
    ei_printf("Present:   0\n");
    ei_printf("\n");
    ei_printf("===== Sampling parameters =====\n");
    at_get_sample_settings();
    ei_printf("\n");
    ei_printf("===== Upload settings =====\n");
    at_get_upload_settings();
    ei_printf("\n");
    ei_printf("===== Remote management =====\n");
    at_get_mgmt_settings();
    ei_printf("\n");

    return true;
}

/**
 *
 * @return
 */
static bool at_device_info(void)
{
    bool ret_val = false;

    if (pei_device != nullptr)
    {
        ei_printf("ID:         %s\n", pei_device->get_device_id().c_str());
        ei_printf("Type:       %s\n", pei_device->get_device_type().c_str());
        ei_printf("AT Version: %s\n", AT_COMMAND_VERSION);
        ei_printf("Data Transfer Baudrate: %lu\n", pei_device->get_data_output_baudrate());
        ret_val = true;
    }
    else
    {

    }

    return ret_val;
}

/**
 *
 * @return
 */
static bool at_clear_config(void)
{
    bool ret_val = false;

    if (pei_device != nullptr)
    {
        ei_printf("Clearing config and restarting system...\n");
        pei_device->clear_config();
        //pei_device->init_device_id(); // done in clear config
        ret_val = true;
    }
    else
    {

    }
    

    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_sample_start(const char **argv, const int argc)
{
    if(argc < 1) {
        ei_printf("Missing sensor name!\n");
        return true;
    }

    const ei_device_sensor_t *sensor_list;
    size_t sensor_list_size;

    pei_device->get_sensor_list((const ei_device_sensor_t **)&sensor_list, &sensor_list_size);

    for (size_t ix = 0; ix < sensor_list_size; ix++) {
        if (strcmp(sensor_list[ix].name, argv[0]) == 0) {
            if (!sensor_list[ix].start_sampling_cb()) {
                ei_printf("ERR: Failed to start sampling\n");
            }
            return true;
        }
    }

    if (ei_connect_fusion_list(argv[0], SENSOR_FORMAT)) {
        pei_device->set_delay_for_sensor(argv[0]);
        if (!ei_fusion_setup_data_sampling()) {
            ei_printf("ERR: Failed to start sensor fusion sampling\n");
        }
    }
    else {
        ei_printf("ERR: Failed to find sensor '%s' in the sensor list\n", argv[0]);
    }


    return true;
}

/**
 *
 * @return
 */
static bool at_get_sample_settings(void)
{
    bool ret_val = false;

    if (pei_device != nullptr)
    {
        ei_printf("Label:     %s\n", pei_device->get_sample_label().c_str());
        ei_printf("Interval:  %.2f ms.\n", pei_device->get_sample_interval_ms());
        ei_printf("Length:    %lu ms.\n", pei_device->get_sample_length_ms());
        ei_printf("HMAC key:  %s\n", pei_device->get_sample_hmac_key().c_str());
        ret_val = true;
    }
    else
    {

    }
    
    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_sample_settings(const char **argv, const int argc)
{
    if ((argc < 3) || (pei_device == nullptr)) {
        ei_printf("Missing argument! Required: " AT_SAMPLESETTINGS_ARGS "\n");
        return false;
    }

    pei_device->set_sample_label(argv[0]);

    //TODO: sanity check and/or exception handling
    std::string interval_ms_str(argv[1]);
    pei_device->set_sample_interval_ms(stof(interval_ms_str));

    //TODO: sanity check and/or exception handling
    std::string sample_length_str(argv[2]);
    pei_device->set_sample_length_ms(stoi(sample_length_str));

    if(argc >= 4) {
        pei_device->set_sample_hmac_key(argv[3]);
    }

    ei_printf("OK\n");

    return true;
}

/**
 * @brief Handler for RUNIMPULE
 *
 * @return
 */
static bool at_run_nn_normal(void)
{
    ei_start_impulse(false, false, false);

    return (is_inference_running());
}

/**
 * @brief Handler for RUNIMPULSECONT
 *
 * @return
 */
static bool at_run_nn_normal_cont(void)
{
    ei_start_impulse(true, false, false);

    return (is_inference_running());
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_read_buffer(const char **argv, const int argc)
{
    bool success = false;

    if(argc < 2) {
        ei_printf("Missing argument! Required: " AT_READBUFFER_ARGS "\n");
        return true;
    }

    if (pei_device != nullptr)
    {
        size_t start = (size_t)atoi(argv[0]);
        size_t length = (size_t)atoi(argv[1]);

        bool use_max_baudrate = false;
        if (argc >= 3 && argv[2][0] == 'y') {
           use_max_baudrate = true;
        }

        if (use_max_baudrate) {
            ei_printf("OK\r\n");
            pei_device->set_max_data_output_baudrate();
            ei_sleep(100);
        }

        success = read_encode_send_sample_buffer(start, length);

        if (use_max_baudrate) {
            ei_printf("\r\nOK\r\n");
            ei_sleep(100);
            pei_device->set_default_data_output_baudrate();
            ei_sleep(100);
        }

        if (!success) {
            ei_printf("Failed to read from buffer\n");
        }
        else {
            ei_printf("\n");
        }
    }

    return success;
}

/**
 *
 * @return
 */
static bool at_get_mgmt_settings(void)
{
    ei_printf("%s\n", pei_device->get_management_url().c_str());

    return true;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_mgmt_settings(const char **argv, const int argc)
{
    bool ret_val = false;

    if (check_args_num(1, argc) == false) {
        return true;
    }

    if (pei_device != nullptr)
    {
        pei_device->set_management_url(argv[0]);
        ei_printf("OK\n");

        ret_val = true;
    }

    return ret_val;
}

/**
 *
 * @return
 */
static bool at_list_sensors(void)
{    
    bool ret_val = false;
    const ei_device_sensor_t *list;
    size_t list_size;

    if (pei_device != nullptr)
    {
        int r = pei_device->get_sensor_list((const ei_device_sensor_t **)&list, &list_size);
        if (r != 0) {
            ei_printf("Failed to get sensor list (%d)\n", r);
            return true;
        }
        ret_val = true;

        for (size_t ix = 0; ix < list_size; ix++) {
            ei_printf(
                "Name: %s, Max sample length: %hus, Frequencies: [",
                list[ix].name,
                list[ix].max_sample_length_s);
            for (size_t fx = 0; fx < EI_MAX_FREQUENCIES; fx++) {
                if (list[ix].frequencies[fx] != 0.0f) {
                    if (fx != 0) {
                        ei_printf(", ");
                    }
                    ei_printf_float(list[ix].frequencies[fx]);
                    ei_printf("Hz");
                }
            }
            ei_printf("]\n");
        }
    }
    else
    {

    }

    return ret_val;
}

/**
 *
 * @return
 */
static bool at_list_fusion_sensors(void)
{
    ei_built_sensor_fusion_list();

    return true;
}

/**
 *
 * @return
 */
static bool at_get_upload_settings(void)
{
    bool ret_val = false;

    if (pei_device != nullptr)
    {
        ei_printf("Api Key:   %s\n", pei_device->get_upload_api_key().c_str());
        ei_printf("Host:      %s\n", pei_device->get_upload_host().c_str());
        ei_printf("Path:      %s\n", pei_device->get_upload_path().c_str());

        ret_val = true;
    }
    else
    {

    }

    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_upload_settings(const char **argv, const int argc)
{
    bool ret_val = false;

    if (check_args_num(2, argc) == false) {
        return false;
    }
    if (pei_device != nullptr)
    {
        pei_device->set_upload_api_key(argv[0]);
        pei_device->set_upload_host(argv[1]);

        ret_val = true;
    }

    ei_printf("OK\n");

    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_upload_host(const char **argv, const int argc)
{
    bool ret_val = false;

    if (check_args_num(1, argc) == false) {
        return false;
    }

    if (pei_device != nullptr)
    {
        pei_device->set_upload_host(argv[0]);
        ret_val = true;
    }

    ei_printf("OK\n");

    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_read_raw(const char **argv, const int argc)
{
    if(argc < 2) {
        ei_printf("Missing argument! Required: " AT_READBUFFER_ARGS "\n");
        return true;
    }

    volatile uint32_t start = (uint32_t)atoi(argv[0]);
    volatile uint32_t length = (uint32_t)atoi(argv[1]);

    unsigned char buffer[32];

    for(; (start < length); start += 32)
    {
        pei_device->read_raw(buffer, start, 32);

        int n_display_bytes = (length - start) < 32 ? (length - start) : 32;
        for(int i=0; i<n_display_bytes; i++)
        {
            ei_printf("%.2X ", (unsigned char)buffer[i]);
        }
        ei_printf("\b\r\n");

        if (start > length)
            return true;
    }

    return true;

    //ei_ti_launchxl_fs_close_sample_file();
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_unlink_file(const char **argv, const int argc)
{
    FSP_PARAMETER_NOT_USED(argv);
    FSP_PARAMETER_NOT_USED(argc);
    ei_printf("\n");

    return true;
}

static inline bool check_args_num(const int &required, const int &received)
{
    if (received < required) {
        ei_printf("Too few arguments! Required: %d\n", required);
        return false;
    }

    return true;
}