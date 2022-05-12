/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
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
/* Include ----------------------------------------------------------------- */
#include "ei_microphone.h"
#include "peripheral/i2s.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ingestion-sdk-platform/renesas-ck-ra6m5/ei_device_renesas_ck_ra6m5.h"
#include "ingestion-sdk/sensor_aq_mbedtls_hs256.h"
#include "ingestion-sdk/ei_sampler.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/sensor_aq.h"
#include "edge-impulse-sdk/CMSIS/DSP/Include/arm_math.h"

#include "ei_dc_blocking.h"
#define FAKE_ON 0
#if FAKE_ON == 1
#include "fake_mic.h"
#endif
/* Constant ---------------------------------------------------------------- */
#define MIC_SAMPLE_SIZE             (EI_I2S_SAMPLES_PER_READ/2)
#define MIC_SAMPLE_BUFF_SIZE        (EI_I2S_SAMPLES_PER_READ/2)
#define MIC_USE_DC_BLOCKING         (0u)

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t     *buffers[2];
    uint8_t     buf_select;
    uint8_t     buf_ready;
    uint32_t    buf_count;
    uint32_t    n_samples;
} inference_t;



/* Private functions ------------------------------------------------ */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM * stream);
static int ei_seek(EI_SENSOR_AQ_STREAM * stream, long int offset, int origin);
static int insert_ref(char *buffer, int hdrLength);
static bool create_header(sensor_aq_payload_info *payload);


static void ingestion_samples_callback(const int16_t *buffer, uint32_t sample_count);

static void ei_mic_init(void);
static void ei_mic_deinit(void);

static uint32_t ei_mic_get_int16_from_buffer(volatile int32_t* src, int16_t* dst, uint32_t element);

/* Private variables ------------------------------------------------------- */
static inference_t inference;
static uint32_t required_samples_size;
static uint32_t headerOffset = 0;
microphone_sample_t *sample_buffer_processed;
static uint32_t current_sample;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/**
 *
 * @return
 */
bool ei_microphone_sample_start(void)
{
    EiDeviceCKRA6M5 *dev = EiDeviceCKRA6M5::get_device();
    EiFlashMemory* mem = static_cast<EiFlashMemory*>(dev->get_memory());
    uint8_t *page_buffer;
    int ret;
    uint32_t required_samples;

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    required_samples = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if(required_samples & 1) {
     required_samples++;
    }

    required_samples_size = required_samples * sizeof(microphone_sample_t);
    current_sample = 0;

    if(required_samples_size > mem->get_available_sample_bytes()) {
      ei_printf("ERR: Sample length is too long. Maximum allowed is %lu ms at 16000 Hz.\r\n",
          ((mem->get_available_sample_bytes() / (16000 * sizeof(microphone_sample_t))) * 1000));
      return false;
    }

    ei_mic_init();

    uint32_t delay_time_ms = ((required_samples_size / mem->block_size) + 1) * mem->block_erase_time;
    ei_printf("Starting in %lu ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);

    if(mem->erase_sample_data(0, required_samples_size) != (required_samples_size)) {
       return false;
    }

    // if erasing took less than 2 seconds, wait additional time
    if(delay_time_ms < 2000) {
       ei_sleep(2000 - delay_time_ms);
    }

    if (create_header(&payload) == false) {
        return false;
    }

//    sample_buffer_processed = (microphone_sample_t*)ei_malloc(MIC_SAMPLE_SIZE);
//    if(sample_buffer_processed == nullptr) {
//        ei_printf("ERR: Failed to allocate audio buffer for processing\n");
//        ei_free(sample_buffer_processed);
//        return false;
//    }

    ei_printf("Sampling...\n");

    while (current_sample < required_samples_size) {
        __WFI();
        ei_mic_thread(&ingestion_samples_callback);
        }
    ei_mic_deinit();

    mem->write_residual();  /* write any additional data */

    //ei_free(sample_buffer_processed);

    ret = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ret != 0) {
     ei_printf("Failed to finish signature (%d)\n", ret);
     return false;
    }

    // load the first page in flash...
    page_buffer = (uint8_t*)ei_malloc(mem->block_size);
    if (!page_buffer) {
     ei_printf("Failed to allocate a page buffer to write the hash\n");
     return false;
    }

    ret = mem->read_sample_data(page_buffer, 0, mem->block_size);
    if ((uint32_t)ret != mem->block_size) {
     ei_printf("Failed to read first page (read %d, requersted %lu)\n", ret, mem->block_size);
     ei_free(page_buffer);
     return false;
    }

    // update the hash
    uint8_t *hash = ei_mic_ctx.hash_buffer.buffer;
    // we have allocated twice as much for this data (because we also want to be able to represent in hex)
    // thus only loop over the first half of the bytes as the signature_ctx has written to those
    for (size_t hash_ix = 0; hash_ix < ei_mic_ctx.hash_buffer.size / 2; hash_ix++) {
     // this might seem convoluted, but snprintf() with %02x is not always supported e.g. by newlib-nano
     // we encode as hex... first ASCII char encodes top 4 bytes
     uint8_t first = (hash[hash_ix] >> 4) & 0xf;
     // second encodes lower 4 bytes
     uint8_t second = hash[hash_ix] & 0xf;

     // if 0..9 -> '0' (48) + value, if >10, then use 'a' (97) - 10 + value
     char first_c = first >= 10 ? 87 + first : 48 + first;
     char second_c = second >= 10 ? 87 + second : 48 + second;

     page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 0] = first_c;
     page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 1] = second_c;
    }

    ret = mem->erase_sample_data(0, mem->block_size);
    if ((uint32_t)ret != mem->block_size) {
     ei_printf("Failed to erase first page (read %d, requested %lu)\n", ret, mem->block_size);
     ei_free(page_buffer);
     return false;
    }

    ret = mem->write_sample_data(page_buffer, 0, mem->block_size);
    mem->write_residual();
    ei_free(page_buffer);

    if ((uint32_t)ret != mem->block_size) {
     ei_printf("Failed to write first page with updated hash (read %d, requested %lu)\n", ret, mem->block_size);
     return false;
    }

    ei_printf("Done sampling, total bytes collected: %lu\n", required_samples_size);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", required_samples_size + headerOffset);
    ei_printf("OK\n");

    return true;
}

/**
 *
 * @param cb
 */
void ei_mic_thread(mic_sampler_callback cb)
{
#if FAKE_ON == 1
    const uint16_t  steps = 4000;
    static uint32_t fake_index = 0;
    int16_t _readl_buffer_audio[steps] = {0};

    memcpy(_readl_buffer_audio, &fake_on[fake_index], sizeof(_readl_buffer_audio));

    fake_index+=steps;
    if (fake_index >= (steps*4))
    {
        fake_index = 0;
    }
    if ((cb != nullptr))
    {
        cb(_readl_buffer_audio, steps);
    }

#else
    int32_t temp_read_buffer[MIC_SAMPLE_BUFF_SIZE] = {0};
    int16_t _readl_buffer_audio[MIC_SAMPLE_BUFF_SIZE] = {0};
    uint32_t index = 0;

    if (ei_i2s_get_status() == I2S_EVENT_RX_FULL)
    {
        uint16_t read_samples = 0;
        read_samples = ei_i2s_get_buffer(temp_read_buffer, EI_I2S_SAMPLES_PER_READ);    /* */
        index += ei_mic_get_int16_from_buffer(temp_read_buffer, &_readl_buffer_audio[index], read_samples);

        if ((cb != nullptr)
                && (index != 0))
        {
            cb(_readl_buffer_audio, read_samples);
        }
    }
#endif
}

/**
 *
 * @param buffer
 * @param sample_count
 */
void inference_samples_callback(const int16_t *buffer, uint32_t sample_count)
{
    for(uint32_t i = 0; i < sample_count; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = buffer[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

/**
 * @brief Write mic
 *
 * @param buffer
 * @param sample_count
 */
static void ingestion_samples_callback(const int16_t *buffer, uint32_t sample_count)
{
    EiDeviceCKRA6M5 *dev = EiDeviceCKRA6M5::get_device();
    EiFlashMemory* mem = static_cast<EiFlashMemory*>(dev->get_memory());

    if(sample_count > MIC_SAMPLE_SIZE) {
        ei_printf("ERR: too much data from microphone: %lu Discarding...\n", sample_count);
        return;
    }

    //write raw data into memory
    mem->write_sample_data((uint8_t*)buffer, headerOffset + current_sample, (sample_count * 2));    /* *2 because we are storing samples of 16 bit */

    //update data hash
    ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)buffer, (sample_count * 2));

    current_sample += sample_count;
}

/**
 *
 * @param offset
 * @param length
 * @param out_ptr
 * @return
 */
int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr)
{
    arm_q15_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    inference.buf_ready = 0;

    return 0;
}

/**
 *
 * @param n_samples
 * @param interval_ms
 * @return
 */
bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    //EiDeviceEKRA65 *dev = EiDeviceEKRA65::get_device();

    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    // double buffer because of double buffering
    sample_buffer_processed = (microphone_sample_t*)ei_malloc(MIC_SAMPLE_SIZE * sizeof(microphone_sample_t));
    if(sample_buffer_processed == nullptr) {
        ei_printf("ERR: Failed to allocate audio buffer\n");
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    /* start mic */
    ei_mic_init();

    return true;
}



/**
 *
 * @return
 */
bool ei_microphone_inference_is_recording(void)
{
    return (inference.buf_ready == 0);
}

/**
 *
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

/**
 *
 * @return
 */
bool ei_microphone_inference_end(void)
{
    ei_i2s_deinit();

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    ei_free(sample_buffer_processed);

    return true;
}

/**
 *
 * @param src
 * @param dst
 * @param element
 * @return
 */
static uint32_t ei_mic_get_int16_from_buffer(volatile int32_t* src, int16_t* dst, uint32_t element)
{
#if MIC_USE_DC_BLOCKING == 1
    uint32_t temp_sample = 0;

    for (uint32_t i = 0; i < element; i++)
    {
        temp_sample = dc_block_filter(src[i]);
        dst[i] = (int16_t)(temp_sample >> 16);    /* shift */
    }
#else
    for (uint32_t i = 0; i < element; i++)
    {
        dst[i] = (int16_t)(src[i] >> 16);    /* shift */
    }

#endif

    return element;
}

/**
 *
 */
void ei_mic_test(void)
{
    static bool test = true;
    static bool print = false;
    volatile uint32_t cycles = 0;

    int16_t _readl_buffer_audio[2048];
    int32_t buffer_audio[MIC_SAMPLE_BUFF_SIZE] = {0};

    uint32_t index = 0 ;

    ei_mic_init();

    while(test)
    {
        if (ei_i2s_get_status() == I2S_EVENT_RX_FULL)
        {
            uint16_t read_samples = 0;
            read_samples = ei_i2s_get_buffer(buffer_audio, MIC_SAMPLE_BUFF_SIZE);
            index += ei_mic_get_int16_from_buffer(buffer_audio, &_readl_buffer_audio[index], read_samples);

            if (print == true)
            {
                for (uint32_t i = 0; i< index; i++)
                {
                    ei_printf("%d = %ld\r\n", i, _readl_buffer_audio[i]);
                }
            }

            if (index >= 2048)
            {
                memset(_readl_buffer_audio, 0, sizeof(_readl_buffer_audio));
                index = 0;
            }
            cycles++;

            memset((void*)buffer_audio, 0, sizeof(buffer_audio));
        }

        __WFI();    /* I2S based on interrupt ... */
    }

    ei_mic_deinit();
}

/* Private functions ------------------------------------------------------- */
/* Dummy functions for sensor_aq_ctx type */
/**
 *
 * @param
 * @param size
 * @param count
 * @param
 * @return
 */
static size_t ei_write(const void* buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM* stream)
{
    (void)buffer;
    (void)size;
    (void)stream;

    return count;
}

/**
 *
 * @param
 * @param offset
 * @param origin
 * @return
 */
static int ei_seek(EI_SENSOR_AQ_STREAM* stream, long int offset, int origin)
{
    (void)stream;
    (void)offset;
    (void)origin;

    return 0;
}

/**
 *
 * @param buffer
 * @param hdrLength
 * @return
 */
static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(unsigned int i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

/**
 *
 * @param payload
 * @return
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    int ret;
    EiDeviceCKRA6M5 *dev = EiDeviceCKRA6M5::get_device();
    EiFlashMemory* mem = static_cast<EiFlashMemory*>(dev->get_memory());
    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    ret = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (ret != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);
    // and update the signature
    ret = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)(ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), ref_size);
    if (ret != 0) {
        ei_printf("Failed to update signature from header (%d)\n", ret);
        return false;
    }
    end_of_header_ix += ref_size;

    // Write to blockdevice
    ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if ((size_t)ret != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

/**
 *
 */
static void ei_mic_init(void)
{
    int err_code = ei_i2s_init();
    if (err_code != 0)
    {
        ei_printf("I2S: error in configuration. Error: %d", err_code);
    }
}

/**
 *
 */
static void ei_mic_deinit(void)
{
    ei_sleep(1000);
    return;
    ei_i2s_deinit();
}
