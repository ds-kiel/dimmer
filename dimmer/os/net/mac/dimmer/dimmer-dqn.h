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

#ifndef DIMMER_DQN_H_
#define DIMMER_DQN_H_

#include "contiki.h"
#include "dimmer.h"

/*---------------------------------------------------------------------------*/
#ifndef DIMMER_DQN_INPUT_SIZE
#define DIMMER_DQN_INPUT_SIZE 31//2*DIMMER_NUM_OF_INPUTS+DIMMER_ONE_HOT_LENGTH+DIMMER_HISTORY_LENGTH
#endif
#ifndef DIMMER_DQN_NB_LAYERS
#define DIMMER_DQN_NB_LAYERS 2
#endif
#ifndef DIMMER_DQN_MAX_LAYER_SIZE
#define DIMMER_DQN_MAX_LAYER_SIZE 31
#endif
#ifndef DIMMER_DQN_LAYER0_SIZE
#define DIMMER_DQN_LAYER0_SIZE 30
#endif
#ifndef DIMMER_DQN_LAYER1_SIZE
#define DIMMER_DQN_LAYER1_SIZE 3
#endif
#ifndef DIMMER_DQN_OUTPUT_SIZE
#define DIMMER_DQN_OUTPUT_SIZE DIMMER_DQN_LAYER1_SIZE
#endif
#ifndef DIMMER_DQN_FIXED_POINT_SCALE
#define DIMMER_DQN_FIXED_POINT_SCALE 100
#endif


/*---------------------------------------------------------------------------*/
enum {DIMMER_ACTIVATION_FN_LINEAR = 0, DIMMER_ACTIVATION_FN_RELU, DIMMER_ACTIVATION_FN_SIGMOID};


/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
struct __attribute__((packed)) dimmer_dqn_config_struct {
  //uint8_t input_size;
  //uint8_t output_size; // equal to the last layer size
  //uint8_t nb_layers;
  uint8_t layer_sizes[DIMMER_DQN_NB_LAYERS];
};

struct dimmer_dqn_nn_struct {
    dimmer_dqn_type_t* weights[DIMMER_DQN_NB_LAYERS];
    dimmer_dqn_type_t* biases[DIMMER_DQN_NB_LAYERS];
    // layer 1
    dimmer_dqn_type_t layer0_weights[DIMMER_DQN_INPUT_SIZE*DIMMER_DQN_LAYER0_SIZE];
    dimmer_dqn_type_t layer0_biases[DIMMER_DQN_LAYER0_SIZE];
    // layer 2
    dimmer_dqn_type_t layer1_weights[DIMMER_DQN_LAYER0_SIZE*DIMMER_DQN_LAYER1_SIZE];
    dimmer_dqn_type_t layer1_biases[DIMMER_DQN_LAYER1_SIZE];
    uint8_t layers_act_fn[DIMMER_DQN_NB_LAYERS];
};
/*---------------------------------------------------------------------------*/




void dimmer_dqn_run(struct dimmer_dqn_config_struct *config, struct dimmer_dqn_nn_struct *nn, dimmer_dqn_compute_type_t input[], dimmer_dqn_compute_type_t output[]);

dimmer_dqn_compute_type_t dimmer_dqn_activation_fn(dimmer_dqn_compute_type_t in, uint8_t activation_fn);

dimmer_dqn_compute_type_t dimmer_dqn_mult(dimmer_dqn_compute_type_t x, dimmer_dqn_compute_type_t y);

void dimmer_dqn_select_n_worst_nodes(dimmer_dqn_compute_type_t statistics[], dimmer_dqn_compute_type_t input[], uint8_t k_worst_size);

void dimmer_dqn_update_historical_values(dimmer_dqn_compute_type_t input[]);

void dimmer_dqn_update_one_hot(dimmer_dqn_compute_type_t input[], uint8_t n_value);

#endif /* DIMMER_DQN_H_ */