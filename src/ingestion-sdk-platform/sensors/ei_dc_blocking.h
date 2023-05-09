/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef INGESTION_SDK_PLATFORM_SENSORS_EI_DC_BLOCKING_H_
#define INGESTION_SDK_PLATFORM_SENSORS_EI_DC_BLOCKING_H_

#include <stdint.h>

#include "common_utils.h"
#include "fsp_common_api.h"

FSP_CPP_HEADER

int32_t dc_block_filter(int32_t x);

FSP_CPP_FOOTER

#endif
