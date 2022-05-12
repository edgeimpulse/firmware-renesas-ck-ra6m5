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

#ifndef INGESTION_SDK_PLATFORM_SENSORS_EI_TEST_SENSORS_H_
#define INGESTION_SDK_PLATFORM_SENSORS_EI_TEST_SENSORS_H_

#define EI_TEST_INDOOR_AQS      0
#define EI_TEST_OUTDOOR_AQS     0
#define EI_TEST_TEMP_HUM_TEST   0
#define EI_TEST_BAROMETRIC      0
#define EI_TEST_MIC             0
#define EI_TEST_INERTIAL        0

extern void ei_test_sensors(void);

#endif /* INGESTION_SDK_PLATFORM_SENSORS_EI_TEST_SENSORS_H_ */
