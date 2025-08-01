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

#ifndef _EI_CLASSIFIER_INFERENCING_ENGINE_TENSAILFOW_H_
#define _EI_CLASSIFIER_INFERENCING_ENGINE_TENSAILFOW_H_

#if (EI_CLASSIFIER_INFERENCING_ENGINE == EI_CLASSIFIER_TENSAIFLOW)

#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/classifier/ei_run_dsp.h"

#include "mcu.h"

extern "C" void infer(const void *impulse_arg, uint32_t* time, uint32_t* cycles);
int8_t *processed_features;

#ifdef EI_CLASSIFIER_NN_OUTPUT_COUNT
int8_t infer_result[EI_CLASSIFIER_NN_OUTPUT_COUNT];
#else
int8_t infer_result[EI_CLASSIFIER_LABEL_COUNT];
#endif

extern "C" void get_data(const void *impulse_arg, int8_t *in_buf_0, uint16_t in_buf_0_dim_0, uint16_t in_buf_0_dim_1, uint16_t in_buf_0_dim_2)
{
    ei_impulse_t *impulse = (ei_impulse_t *) impulse_arg;

    if ((impulse->sensor == EI_CLASSIFIER_SENSOR_CAMERA) &&
        ((impulse->dsp_blocks_size == 1) ||
        (impulse->dsp_blocks[0].extract_fn == extract_image_features))) {

        memcpy(in_buf_0, processed_features, impulse->nn_input_frame_size);
    }
}

extern "C" void post_process(const void *block_config_arg, int8_t *out_buf_0, int8_t *out_buf_1)
{
    ei_config_tensaiflow_graph_t *block_config = (ei_config_tensaiflow_graph_t *) block_config_arg;

    #ifdef EI_CLASSIFIER_NN_OUTPUT_COUNT
    memcpy(infer_result, out_buf_0, block_config->output_features_count);
    #else
    memcpy(infer_result, out_buf_0, impulse->label_count);
    #endif
}

/**
 * @brief      Do neural network inferencing over the processed feature matrix
 *
 * @param      fmatrix  Processed matrix
 * @param      result   Output classifier results
 * @param[in]  debug    Debug output enable
 *
 * @return     The ei impulse error.
 */
EI_IMPULSE_ERROR run_nn_inference(
    const ei_impulse_t *impulse,
    ei_feature_t *fmatrix,
    uint32_t learn_block_index,
    uint32_t* input_block_ids,
    uint32_t input_block_ids_size,
    ei_impulse_result_t *result,
    void *config_ptr,
    bool debug = false)
{
    ei_learning_block_config_tflite_graph_t *block_config = (ei_learning_block_config_tflite_graph_t*)config_ptr;
    ei_config_tensaiflow_graph_t *graph_config = (ei_config_tensaiflow_graph_t*)block_config->graph_config;

    uint64_t ctx_start_us = ei_read_timer_us();
    uint32_t time, cycles;

    /* Run tensaiflow inference */
    infer((const void *)graph_config, &time, &cycles);

    // Inference results returned by post_process() and copied into infer_results

    result->timing.classification_us = ei_read_timer_us() - ctx_start_us;
    result->timing.classification = (int)(result->timing.classification_us / 1000);

    result->_raw_outputs[learn_block_index].matrix = new matrix_t(1, graph_config->output_features_count);
    result->_raw_outputs[learn_block_index].blockId = block_config->block_id;

    for (uint32_t ix = 0; ix < impulse->label_count; ix++) {
        result->_raw_outputs[learn_block_index].matrix->buffer[ix] = infer_result[ix];
    }
    return EI_IMPULSE_OK;
}

/**
 * Special function to run the classifier on images, only works on TFLite models (either interpreter or EON or for tensaiflow)
 * that allocates a lot less memory by quantizing in place. This only works if 'can_run_classifier_image_quantized'
 * returns EI_IMPULSE_OK.
 */
EI_IMPULSE_ERROR run_nn_inference_image_quantized(
    const ei_impulse_t *impulse,
    signal_t *signal,
    uint32_t learn_block_index,
    ei_impulse_result_t *result,
    void *config_ptr,
    bool debug = false)
{
    ei_learning_block_config_tflite_graph_t *block_config = (ei_learning_block_config_tflite_graph_t*)config_ptr;
    ei_config_tensaiflow_graph_t *graph_config = (ei_config_tensaiflow_graph_t*)block_config->graph_config;

    uint64_t ctx_start_us;
    uint64_t dsp_start_us = ei_read_timer_us();

    ei::matrix_i8_t features_matrix(1, impulse->nn_input_frame_size);
    processed_features = (int8_t *) features_matrix.buffer;

    // run DSP process and quantize automatically
    int ret = extract_image_features_quantized(
        signal,
        &features_matrix,
        impulse->dsp_blocks[0].config,
        graph_config->input_scale,
        graph_config->input_zeropoint,
        impulse->frequency,
        impulse->learning_blocks[0].image_scaling);

    if (ret != EIDSP_OK) {
        ei_printf("ERR: Failed to run DSP process (%d)\n", ret);
        return EI_IMPULSE_DSP_ERROR;
    }

    if (ei_run_impulse_check_canceled() == EI_IMPULSE_CANCELED) {
        return EI_IMPULSE_CANCELED;
    }

    result->timing.dsp_us = ei_read_timer_us() - dsp_start_us;
    result->timing.dsp = (int)(result->timing.dsp_us / 1000);

    if (debug) {
        ei_printf("Features (%d ms.): ", result->timing.dsp);
        for (size_t ix = 0; ix < features_matrix.cols; ix++) {
            ei_printf_float((features_matrix.buffer[ix] - graph_config->input_zeropoint) * graph_config->input_scale);
            ei_printf(" ");
        }
        ei_printf("\n");
    }

    uint32_t time, cycles;
    ctx_start_us = ei_read_timer_us();

    /* Run tensaiflow inference */
    infer((const void *)impulse, &time, &cycles);

    // Inference results returned by post_process() and copied into infer_results

    size_t output_size = graph_config->output_features_count;

    result->_raw_outputs[learn_block_index].matrix = new matrix_t(1, output_size);
    result->_raw_outputs[learn_block_index].blockId = block_config->block_id;

    for (size_t i = 0; i < output_size; i++) {
        result->_raw_outputs[learn_block_index].matrix->buffer[i] = infer_result[i];
    }

    result->timing.classification_us = ei_read_timer_us() - ctx_start_us;
    result->timing.classification = (int)(result->timing.classification_us / 1000);
    return EI_IMPULSE_OK;
}

#endif // #if (EI_CLASSIFIER_INFERENCING_ENGINE == EI_CLASSIFIER_TENSAILFOW)
#endif // _EI_CLASSIFIER_INFERENCING_ENGINE_TENSAILFOW_H_
