/*
 * Copyright (c) 2020, Kiel University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Valentin Poirot
 */

#include <stdio.h>
#include <string.h>
#include "dimmer.h"
#include "dimmer-dqn.h"

/* This should be compiled only if Dimmer is used */
#if USE_DIMMER

#define DIMMER_DQN_MULT(a,b) (((dimmer_dqn_compute_type_t) a)*((dimmer_dqn_compute_type_t) b))
#define DIMMER_DQN_ReLU(a) (((a) < 0) ? 0 : (a))

static dimmer_dqn_compute_type_t previous_layer[DIMMER_DQN_MAX_LAYER_SIZE];
static dimmer_dqn_compute_type_t current_layer[DIMMER_DQN_MAX_LAYER_SIZE];
static uint8_t indexes[DIMMER_NUM_OF_NODES];
static dimmer_dqn_compute_type_t tmp_arr[DIMMER_NUM_OF_NODES];

void dimmer_dqn_run(struct dimmer_dqn_config_struct *config, struct dimmer_dqn_nn_struct *nn, dimmer_dqn_compute_type_t input[], dimmer_dqn_compute_type_t output[]) {
    // create placeholder for temporary results
    memcpy(previous_layer, input, DIMMER_DQN_INPUT_SIZE*sizeof(dimmer_dqn_compute_type_t));
    memset(current_layer, 0, DIMMER_DQN_MAX_LAYER_SIZE*sizeof(dimmer_dqn_compute_type_t));

    // for memory management
    uint8_t previous_layer_size = DIMMER_DQN_INPUT_SIZE;

    int i = 0;
    printf("DEBUG_DQN_INPT: [");
    for (i=0;i<DIMMER_DQN_INPUT_SIZE;++i) {
        printf("%li,", input[i]);
    }
    printf("]\n");

    int layer, neuron, connection;
    for (layer = 0; layer < DIMMER_DQN_NB_LAYERS; ++layer) {
        uint8_t layer_size = config->layer_sizes[layer];
        for (neuron = 0; neuron < layer_size; ++neuron) {
            // compute through each weight
            current_layer[neuron] = 0;
            for (connection = 0; connection < previous_layer_size; ++connection) {
                current_layer[neuron] += DIMMER_DQN_MULT((dimmer_dqn_compute_type_t) nn->weights[layer][previous_layer_size*neuron+connection], previous_layer[connection]);
            }
            // add bias
            current_layer[neuron] += DIMMER_DQN_MULT((dimmer_dqn_compute_type_t)  (nn->biases[layer][neuron]), 100);
            // apply activation function
            //current_layer[neuron] = dimmer_dqn_activation_fn(current_layer[neuron], nn->layers_act_fn[layer]);
            if (layer < DIMMER_DQN_NB_LAYERS-1) {
                current_layer[neuron] = DIMMER_DQN_ReLU(current_layer[neuron]);
            }
            // reduce scaling
            current_layer[neuron] /= DIMMER_DQN_FIXED_POINT_SCALE;
        }
        memcpy(previous_layer, current_layer, layer_size*sizeof(dimmer_dqn_compute_type_t));
        //memset(&current_layer, 0x00, MAX_LAYER_SIZE*sizeof(dimmer_dqn_type_t));
        previous_layer_size = layer_size;
    }
    memcpy(output, previous_layer, DIMMER_DQN_OUTPUT_SIZE*sizeof(dimmer_dqn_compute_type_t));


    printf("DEBUG_DQN_OUTPT: [");
    for (i=0;i<3;++i) {
        printf("%li,", output[i]);
    }
    printf("]\n");
}


dimmer_dqn_compute_type_t dimmer_dqn_activation_fn(dimmer_dqn_compute_type_t in, uint8_t activation_fn) {
    //dimmer_dqn_type_t out;
    if (activation_fn == DIMMER_ACTIVATION_FN_LINEAR) {
            return in;
    } else if (activation_fn == DIMMER_ACTIVATION_FN_RELU) {
        return ((in < 0) ? 0 : in);
    } else if (activation_fn == DIMMER_ACTIVATION_FN_SIGMOID) {
        // Not implemented yet
        return in;
    }
    // Not implemented yet
    return in;
}


int8_t partition(dimmer_dqn_compute_type_t arr[], int8_t low, int8_t high) {
    dimmer_dqn_compute_type_t pivot = arr[high];
    int8_t i = (low - 1);
    uint8_t j;
    for (j=low; j<=high-1; ++j) {
        if (arr[j] < pivot) {
            ++i;
            dimmer_dqn_compute_type_t tmp = arr[i];
            uint8_t tmp_index = indexes[i];
            arr[i] = arr[j];
            indexes[i] = indexes[j];
            arr[j] = tmp;
            indexes[j] = tmp_index;
        }
    }
    dimmer_dqn_compute_type_t tmp = arr[i+1];
    uint8_t tmp_index = indexes[i+1];
    arr[i+1] = arr[high];
    indexes[i+1] = indexes[high];
    arr[high] = tmp;
    indexes[high] = tmp_index;
    return (i + 1);
}

void quicksort(dimmer_dqn_compute_type_t arr[], int8_t low, int8_t high) {
    if (low < high) {
        uint8_t pi = partition(arr, low, high);
        quicksort(arr, low, pi-1);
        quicksort(arr, pi+1, high);
    }
}

void dimmer_dqn_select_n_worst_nodes(dimmer_dqn_compute_type_t statistics[], dimmer_dqn_compute_type_t input[], uint8_t k_worst_size) {
    // generate indexes array
    uint8_t i;
    for (i = 0; i<DIMMER_NUM_OF_NODES; ++i) {
        indexes[i] = i;
        // we order based on reliability measures
        tmp_arr[i] = statistics[DIMMER_NUM_OF_NODES+i];
    }
    // run quicksort
    quicksort(tmp_arr, 0, DIMMER_NUM_OF_NODES-1);
    // select N first elements and write into input
    for (i = 0; i<k_worst_size; ++i) {
        // we order based on reliability measures
        input[i] = statistics[indexes[i]];
        input[k_worst_size+i] = statistics[DIMMER_NUM_OF_NODES+indexes[i]];
    }
}

void dimmer_dqn_update_one_hot(dimmer_dqn_compute_type_t input[], uint8_t n_value) {
  memset(&(input[2*DIMMER_NUM_OF_INPUTS]), 0, DIMMER_ONE_HOT_LENGTH*sizeof(dimmer_dqn_compute_type_t));
  input[2*DIMMER_NUM_OF_INPUTS + n_value] = 100;
}

void dimmer_dqn_update_historical_values(dimmer_dqn_compute_type_t input[]) {
  dimmer_dqn_compute_type_t historical_value = 0;
  historical_value = -100;
  if (input[DIMMER_NUM_OF_INPUTS] == 100) {
      historical_value = 100;
  }
  int i;
//   for (i=DIMMER_NUM_OF_INPUTS; i<2*DIMMER_NUM_OF_INPUTS; ++i) {
//     // compute average for last step
//     average += input[i];
//   }
  for (i=0; i<DIMMER_HISTORY_LENGTH-1; ++i) {
    // shift historical values by one to the left
    input[2*DIMMER_NUM_OF_INPUTS+DIMMER_ONE_HOT_LENGTH+i] = input[2*DIMMER_NUM_OF_INPUTS+DIMMER_ONE_HOT_LENGTH+i+1];
  }
  // add historical_value to last input
  input[2*DIMMER_NUM_OF_INPUTS+DIMMER_ONE_HOT_LENGTH+DIMMER_HISTORY_LENGTH-1] = historical_value;// /DIMMER_NUM_OF_INPUTS;
}

#endif /* USE_DIMMER */