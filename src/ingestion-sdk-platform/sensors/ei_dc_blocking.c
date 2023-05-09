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
#include "ei_dc_blocking.h"

static float ym1 = 0.0;
static float xm1 = 0.0;

/**
 *
 * @param x
 * @return
 */
int32_t dc_block_filter(int32_t x)
{
    int32_t y;

    ym1 = (float)(x - xm1 + (0.995 * ym1));

    xm1 = x;
    y = ym1;

    return y;
}
