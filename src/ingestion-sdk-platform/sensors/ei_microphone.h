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

#ifndef INGESTION_SDK_PLATFORM_SENSORS_EI_MICROPHONE_H_
#define INGESTION_SDK_PLATFORM_SENSORS_EI_MICROPHONE_H_

#include <cstdint>
#include <cstdlib>

typedef int16_t microphone_sample_t;
typedef void (*mic_sampler_callback)(const int16_t *sample_buf, uint32_t byteLenght);

void ei_mic_test(void);

bool ei_microphone_sample_start(void);
int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr);
bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms);
bool ei_microphone_inference_is_recording(void);
void ei_microphone_inference_reset_buffers(void);
bool ei_microphone_inference_end(void);
void ei_mic_thread(mic_sampler_callback cb);
void inference_samples_callback(const int16_t *buffer, uint32_t sample_count);


#endif /* INGESTION_SDK_PLATFORM_SENSORS_EI_MICROPHONE_H_ */