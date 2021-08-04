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

#ifndef DIMMER_EXP3_H_
#define DIMMER_EXP3_H_

#include "contiki.h"
#include "dimmer.h"

/*---------------------------------------------------------------------------*/
#ifndef DIMMER_EXP3_GAMMA
#define DIMMER_EXP3_GAMMA 0.1
#endif
#ifndef DIMMER_EXP3_NUMBER_ARMS
#define DIMMER_EXP3_NUMBER_ARMS 2 // ON-OFF
#endif
#ifndef DIMMER_EXP3_OPTIMIZATION_ROUND_NB
#define DIMMER_EXP3_OPTIMIZATION_ROUND_NB 10 // each node has X rounds to update its optimization probability
#endif
/*---------------------------------------------------------------------------*/

struct __attribute__((packed)) dimmer_exp3_struct {
  float probability[DIMMER_EXP3_NUMBER_ARMS];
  float omega[DIMMER_EXP3_NUMBER_ARMS];
  uint8_t collect_t;
  uint8_t collect_tp1;
  uint8_t collect_tp2;
  uint8_t decision_t;
  uint8_t decision_tm1;
  uint8_t decision_tm2;
  uint8_t default_action;
};
/*---------------------------------------------------------------------------*/


void dimmer_exp3_init(struct dimmer_exp3_struct *exp3);

uint8_t dimmer_exp3_run(struct dimmer_exp3_struct *exp3);

void dimmer_exp3_update(struct dimmer_exp3_struct *exp3, uint8_t chosen_action, float reward);

void dimmer_exp3_optimize(struct dimmer_exp3_struct *exp3, uint16_t round_id, uint8_t execute_next_round, dimmer_dqn_compute_type_t input[], uint8_t dimmer_flood_N_parameter, uint8_t is_coordinator);

float dimmer_exp3_compute_reward(dimmer_dqn_compute_type_t input[], uint8_t arm);

void dimmer_exp3_punish_passivity(struct dimmer_exp3_struct *exp3);

#endif /* DIMMER_EXP3_H_ */