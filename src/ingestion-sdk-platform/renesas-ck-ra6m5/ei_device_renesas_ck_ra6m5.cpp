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
/** Include ----------------------------------------------------------------- */
#include <renesas-ck-ra6m5/ei_device_renesas_ck_ra6m5.h>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_fusion.h"
#include "peripheral/uart_ep.h"
#include "peripheral/flash_handler.h"
#include "peripheral/timer_handler.h"
#include "sensors/ei_inertial_sensor.h"
#include "sensors/ei_microphone.h"
#include "sensors/ei_air_quality_indoor.h"
#include "sensors/ei_air_quality_outdoor.h"

/* Constants --------------------------------------------------------------- */
const ei_device_data_output_baudrate_t ei_dev_normal_data_output_baudrate = {
    "115200",
    115200,
};

/** Private function declarations ------------------------------------------- */

/** Public functions ------------------------------------------- */
/**
 * @brief
 */
EiDeviceCKRA6M5::EiDeviceCKRA6M5(EiDeviceMemory* code_flash, EiDeviceMemory* data_flash_to_set)
{
    EiDeviceInfo::memory = code_flash;
    EiDeviceCKRA6M5::data_flash = data_flash_to_set;

#if MULTI_FREQ_ENABLED == 1
    fusioning = 1;
#endif

    device_type = "RENESAS_CK_RA6M5";
    init_device_id();   // set ID

    load_config();

    sensors[MICROPHONE].name = "Microphone";
    sensors[MICROPHONE].frequencies[0] = 16000.0f;
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_sample_start;
    sensors[MICROPHONE].max_sample_length_s = (uint16_t)(code_flash->get_available_sample_bytes() / (16000 * sizeof(microphone_sample_t)));
}

EiDeviceCKRA6M5::~EiDeviceCKRA6M5()
{

}

/**
 * @brief Singleton class
 *
 * @return pointer to class
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    static EiFlashMemory code_memory(e_flash_code, sizeof(EiConfig));
    static EiFlashMemory data_memory(e_flash_data, 0);                  /* code flash doesn't store config !*/
    static EiDeviceCKRA6M5 dev(&code_memory, &data_memory);

    return &dev;
}

/**
 *
 * @param sensor_list
 * @param sensor_list_size
 * @return
 */
bool EiDeviceCKRA6M5::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    *sensor_list      = sensors;
    *sensor_list_size = EI_DEVICE_N_SENSORS;

    return false;
}

/**Ì
 *
 * @param baudrate
 * @return
 */
uint32_t EiDeviceCKRA6M5::get_data_output_baudrate(void)
{
    return ei_dev_normal_data_output_baudrate.val;
}

/**
 *
 */
void EiDeviceCKRA6M5::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

/**
 *
 */
void EiDeviceCKRA6M5::init_device_id(void)
{
    const bsp_unique_id_t *pdev_id;
    char temp[20];

    pdev_id = R_BSP_UniqueIdGet();

    snprintf(temp, sizeof(temp), "%02x:%02x:%02x:%02x:%02x:%02x",
            pdev_id->unique_id_bytes[5],
            pdev_id->unique_id_bytes[4],
            pdev_id->unique_id_bytes[3],
            pdev_id->unique_id_bytes[2],
            pdev_id->unique_id_bytes[1],
            pdev_id->unique_id_bytes[0]);

    device_id = std::string(temp);
}

/**
 *
 * @return
 */
bool EiDeviceCKRA6M5::save_config(void)
{
    EiConfig buf;

    memset(&buf, 0, sizeof(EiConfig));

    strncpy(buf.wifi_ssid, wifi_ssid.c_str(), 128);
    strncpy(buf.wifi_password, wifi_password.c_str(), 128);
    buf.wifi_security = wifi_security;
    buf.sample_interval_ms = sample_interval_ms;
    buf.sample_length_ms = sample_length_ms;
    strncpy(buf.sample_label, sample_label.c_str(), 128);
    strncpy(buf.sample_hmac_key, sample_hmac_key.c_str(), 33);
    strncpy(buf.upload_host, upload_host.c_str(), 128);
    strncpy(buf.upload_path, upload_path.c_str(), 128);
    strncpy(buf.upload_api_key, upload_api_key.c_str(), 128);
    strncpy(buf.mgmt_url, management_url.c_str(), 128);
    buf.magic = 0xdeadbeef;

    return data_flash->save_config((uint8_t *)&buf, sizeof(EiConfig)); /* save config in data flash memory */
}

void EiDeviceCKRA6M5::copy_device_info(char* char_device_id, char* char_device_type)
{
    std::strcpy(char_device_id, device_id.c_str());
    std::strcpy(char_device_type, device_type.c_str());
}

/**
 *
 */
void EiDeviceCKRA6M5::load_config(void)
{
    EiConfig buf;

    memset(&buf, 0, sizeof(EiConfig));
    data_flash->load_config((uint8_t *)&buf, sizeof(EiConfig)); /* load from data flash */

    if (buf.magic == 0xdeadbeef)
    {
        wifi_ssid = std::string(buf.wifi_ssid, 128);
        wifi_password = std::string(buf.wifi_password, 128);
        wifi_security = buf.wifi_security;
        sample_interval_ms = buf.sample_interval_ms;
        sample_length_ms = buf.sample_length_ms;
        sample_label = std::string(buf.sample_label, 128);
        sample_hmac_key = std::string(buf.sample_hmac_key, 33);
        upload_host = std::string(buf.upload_host, 128);
        upload_path = std::string(buf.upload_path, 128);
        upload_api_key = std::string(buf.upload_api_key, 128);
        management_url = std::string(buf.mgmt_url, 128);
    }
}

bool EiDeviceCKRA6M5::test_flash(void)
{
    static const uint32_t buffer_size = 400;
    EiFlashMemory* mem_flash = static_cast<EiFlashMemory*>(this->get_memory());

    uint8_t pippo[buffer_size];
    uint8_t leggi_pippo[buffer_size] = {0};
    bool ret_val = true;

    for (int i = 0; i< 400; i++)
    {
        pippo[i] = i;
    }

    memory->erase_sample_data(0, buffer_size);
    memory->write_sample_data(pippo, 0, buffer_size);
    mem_flash->write_residual();

    memory->read_sample_data(&leggi_pippo[0], 0, buffer_size);

    for (int i = 0; i< buffer_size; i++)
    {
        if (pippo[i] != leggi_pippo[i]){
            return false;
        }
    }

    return ret_val;
}

uint32_t EiDeviceCKRA6M5::read_raw(uint8_t *buffer, uint32_t pos, uint32_t bytes)
{
    return memory->read_sample_data(buffer, pos, bytes);
}

void EiDeviceCKRA6M5::write_residual(void)
{
    EiFlashMemory* mem_flash = static_cast<EiFlashMemory*>(this->get_memory());

    mem_flash->write_residual();
}

/**
 *
 * @param sample_read_cb
 * @param sample_interval_ms
 * @return
 */
bool EiDeviceCKRA6M5::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    this->is_sampling = true;
    this->sample_read_callback = sample_read_cb;
    this->sample_interval_ms = sample_interval_ms;
    this->actual_timer = 0;
    this->fusioning = 1;        //

    ei_timer1_start((uint32_t)this->sample_interval_ms);

    return true;
}

/**
 *
 * @return
 */
bool EiDeviceCKRA6M5::stop_sample_thread(void)
{
    this->is_sampling = false;

    ei_timer1_stop();

    return true;
}

/**
 *
 */
void EiDeviceCKRA6M5::sample_thread(void)
{
#if MULTI_FREQ_ENABLED == 1    
    if (this->fusioning == 1){
        if (_timer_1_set == true)
        {
            ei_timer1_stop();
            if (this->sample_read_callback != nullptr)
            {
                this->sample_read_callback();

                if (this->is_sampling == true)
                {
                    ei_timer1_start((uint32_t)this->sample_interval_ms);
                }
            }
        }
    }
    else{
        uint8_t flag = 0;
        uint8_t i = 0;

        this->actual_timer += (uint32_t)this->sample_interval;

        if (_timer_1_set == true)
        {
            ei_timer1_stop();

            for (i = 0; i < this->fusioning; i++){
                if (((uint32_t)(this->actual_timer % (uint32_t)this->multi_sample_interval[i])) == 0) {
                    flag |= (1<<i);
                }
            }

            if (this->sample_multi_read_callback != nullptr)
            {
                this->sample_multi_read_callback(flag);

                if (this->is_sampling == true)
                {
                    ei_timer1_start((uint32_t)this->sample_interval);
                }
            }
        }
    }
#else
    if (_timer_1_set == true)
    {
        ei_timer1_stop();
        if (this->sample_read_callback != nullptr)
        {
            this->sample_read_callback();

            if (this->is_sampling == true)
            {
                ei_timer1_start((uint32_t)this->sample_interval_ms);
            }
        }
    } 

#endif

}

#if MULTI_FREQ_ENABLED == 1
/**
 *
 * @param sample_read_cb
 * @param multi_sample_interval_ms
 * @param num_fusioned
 * @return
 */
bool EiDeviceCKRA6M5::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
    uint8_t i;
    uint8_t flag = 0;

    this->is_sampling = true;
    this->sample_multi_read_callback = sample_multi_read_cb;
    this->fusioning = num_fusioned;

    this->multi_sample_interval.clear();

    for (i = 0; i < num_fusioned; i++){
        this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
    }

    this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

    /* force first reading */
    for (i = 0; i < this->fusioning; i++){
            flag |= (1<<i);
    }
    this->sample_multi_read_callback(flag);

    this->actual_timer = 0;
    ei_timer1_start((uint32_t)this->sample_interval);

    return true;
}
#endif

/**
 *
 */
void EiDeviceCKRA6M5::set_delay_for_sensor(const char *input_list)
{
    if ((strstr(input_list, "Indoor air quality") != NULL) &&  /* ugly for now. */
            (ei_air_quality_indoor_is_warmup() == false))
    {
        this->warmup_time = 180000;
        this->warmup_required = true;
        this->warmup = &ei_air_quality_indoor_warmup;
    }
    else if ((strstr(input_list, "Outdoor air quality") != NULL) && /* ugly for now. */
            (ei_air_quality_outdoor_is_warmup() == false))
    {
        this->warmup_time = 180000000;
        this->warmup_required = true;
        this->warmup = &ei_air_quality_outdoor_warmup;
    }
    else
    {
        this->warmup = nullptr;
        this->warmup_time = 0;
        this->warmup_required = false;
    }
}

/**
 *
 * @return
 */
bool EiDeviceCKRA6M5::is_warmup_required(void)
{
    return this->warmup_required;
}

/**
 * @brief
 *
 * @return
 */
uint32_t EiDeviceCKRA6M5::get_warmup_time(void)
{
    return this->warmup_time;
}

/**
 * @brief Returns char from uart rx buffer
 *
 * @param is_inference_running If inference is running, we need to check for a single 'b'
 * @return
 */
char ei_get_serial_byte(uint8_t is_inference_running)
{
    return uart_get_rx_data(is_inference_running);
}

/** Private function definition ------------------------------------------- */
