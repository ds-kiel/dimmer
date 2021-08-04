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

#include "contiki.h"
#if USE_DIMMER
#include <stdio.h>
#include <random.h>
#include <math.h>
#include "node-id.h"
#include "dimmer-exp3.h"
#include "dimmer.h"
#include "lwb/lwb.h"


#define ARGMAX(a,b) (((a)<(b))?1:0)

#define FORWARDER_ARM 1
#define LEAF_ARM 0

#if KIEL
  static uint8_t dimmer_exp3_pseudorandom_order[DIMMER_NUM_OF_NODES] = {7, 17, 1, 18, 2, 9, 15, 13, 8, 11, 5, 6, 4, 3, 16, 14, 12, 10};
#else
  static uint8_t dimmer_exp3_pseudorandom_order[DIMMER_NUM_OF_NODES] = {1,2,3,4,5,6,7,8,9,10};
#endif

/* Init omega */
void dimmer_exp3_init(struct dimmer_exp3_struct *exp3) {
  // set initial values
  exp3->omega[LEAF_ARM] = 1.0f;
  exp3->probability[LEAF_ARM] = 0.45f;
  exp3->probability[FORWARDER_ARM] = 0.55f;
  exp3->omega[FORWARDER_ARM] = 1.1f;
  exp3->collect_t = 0;
  exp3->collect_tp1 = 0;
  exp3->collect_tp2 = 0;
  exp3->decision_t = 0;
  exp3->decision_tm1 = 0;
  exp3->decision_tm2 = 0;
  exp3->default_action = 0;
  // Values obtained after several iterations, used to bootstrap the network
  // the higher the omega, the more confident the system is
  //  #if KIEL 
  //   if (node_id == 1) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 2) {
  //     exp3->probability[0] = 0.99;
  //     exp3->probability[1] = 0.01;
  //     exp3->omega[0] = 200.0;
  //     exp3->omega[1] = 1.0;
  //   } else if (node_id == 3) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 4) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 5) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 6) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 7) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 8) {
  //     exp3->probability[0] = 0.0;
  //     exp3->probability[1] = 1.0;
  //     exp3->omega[0] = 0.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 9) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 10) {
  //     exp3->probability[0] = 0.99;
  //     exp3->probability[1] = 0.01;
  //     exp3->omega[0] = 200.0;
  //     exp3->omega[1] = 1.0;
  //   } else if (node_id == 11) {
  //     exp3->probability[0] = 0.99;
  //     exp3->probability[1] = 0.01;
  //     exp3->omega[0] = 200.0;
  //     exp3->omega[1] = 1.0;
  //   } else if (node_id == 12) {
  //     exp3->probability[0] = 0.99;
  //     exp3->probability[1] = 0.01;
  //     exp3->omega[0] = 200.0;
  //     exp3->omega[1] = 1.0;
  //   } else if (node_id == 13) {
  //     exp3->probability[0] = 0.99;
  //     exp3->probability[1] = 0.01;
  //     exp3->omega[0] = 200.0;
  //     exp3->omega[1] = 1.0;
  //   } else if (node_id == 14) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 15) {
  //     exp3->probability[0] = 0.01;
  //     exp3->probability[1] = 0.99;
  //     exp3->omega[0] = 1.0;
  //     exp3->omega[1] = 200.0;
  //   } else if (node_id == 16) {
  //     exp3->probability[0] = 0.99;
  //     exp3->probability[1] = 0.01;
  //     exp3->omega[0] = 200.0;
  //     exp3->omega[1] = 1.0;
  //   } else if (node_id == 17) {
  //     exp3->probability[0] = 0.99;
  //     exp3->probability[1] = 0.01;
  //     exp3->omega[0] = 200.0;
  //     exp3->omega[1] = 1.0;
  //   } else if (node_id == 18) {
  //     exp3->probability[0] = 0.99;
  //     exp3->probability[1] = 0.01;
  //     exp3->omega[0] = 200.0;
  //     exp3->omega[1] = 1.0;
  //   }
  //   #endif
}


uint8_t dimmer_exp3_run(struct dimmer_exp3_struct *exp3) {
    uint8_t i;
    // Exp3's probabilities
    // P_i = (1-GAMMA) * omega_i/SUM_j(omega_j)  + GAMMA/NUM_ARMS 
    float sum_omega = 0;
    for (i=0;i<DIMMER_EXP3_NUMBER_ARMS;++i) {
        sum_omega += exp3->omega[i];
    }
    // compute probability for each arm
    for (i=0;i<DIMMER_EXP3_NUMBER_ARMS;++i) {
        exp3->probability[i] = (DIMMER_EXP3_NUMBER_ARMS*((1-DIMMER_EXP3_GAMMA) * exp3->omega[i]/sum_omega) + DIMMER_EXP3_GAMMA)/DIMMER_EXP3_NUMBER_ARMS;
    }
    // randomly select arm based on probabilities
    // Only two arms, we check if our random number is within the probability of the first arm
    unsigned short rand = random_rand()%100;
    if (rand < (int)(100*exp3->probability[LEAF_ARM])) {
        return LEAF_ARM;
    }
    return FORWARDER_ARM;

}

void dimmer_exp3_update(struct dimmer_exp3_struct *exp3, uint8_t chosen_action, float reward) {
  // Update omega's value based on the reward
  float est_reward = reward / exp3->probability[chosen_action];
  exp3->omega[chosen_action] = exp3->omega[chosen_action] * expf((DIMMER_EXP3_GAMMA*est_reward)/((float)DIMMER_EXP3_NUMBER_ARMS));
  exp3->omega[chosen_action] = MAX(exp3->omega[chosen_action], 1.0); // since we include negative rewards, we must ensure that omega does not tend to 0
}

void dimmer_exp3_punish_passivity(struct dimmer_exp3_struct *exp3) {
  exp3->omega[0] = 1.0;
}

float dimmer_exp3_compute_reward(dimmer_dqn_compute_type_t input[], uint8_t arm) {
    // If any node's reliability falls below 80%, the optimization was harmful
    uint8_t i, suffered_minor_losses = 0;
    for (i=0; i<DIMMER_NUM_OF_NODES; ++i) {
      if (i == 7) // we must keep the initiator ID
        continue;
        if (input[DIMMER_NUM_OF_NODES+i] < 100) { // 80% reported between -100,100, wehere -100 = 50%
          if (arm == LEAF_ARM && input[DIMMER_NUM_OF_NODES+i] < 60) {
            return -1.0f;
          }
          suffered_minor_losses |= 0x01;
        }
    }
    // Some losses recorded, but not enough to warrant deactivating the passive arm
    if (suffered_minor_losses)
      return -0.1f; // 0.2

    // No losses
    if (arm == FORWARDER_ARM)
      return 0.8f; // slightly lower reward to get some leaf nodes
    return 1.0f;
}

void dimmer_exp3_optimize(struct dimmer_exp3_struct *exp3, uint16_t round_id, uint8_t execute_tp1, dimmer_dqn_compute_type_t input[], uint8_t dimmer_flood_N_parameter, uint8_t is_coordinator) {

  // set the flags to the new round
  // Should we check what we've just received?
  exp3->collect_t = exp3->collect_tp1;
  // Did we just try something, and need to check next time?
  exp3->collect_tp1 = exp3->collect_tp2;
  // collect feedback in two rounds if we can try something next
  exp3->collect_tp2 = execute_tp1 && (dimmer_exp3_pseudorandom_order[(round_id/DIMMER_EXP3_OPTIMIZATION_ROUND_NB)%DIMMER_NUM_OF_NODES] == node_id);
  // oldest decision we keep track off, will be rewarded now
  exp3->decision_tm2 = exp3->decision_tm1;
  // We have just tried it, feedback is coming next round
  exp3->decision_tm1 = exp3->decision_t;
  // By default, take the best probability, we choose a random arm later on
  exp3->decision_t = ARGMAX(((int) (100*exp3->probability[LEAF_ARM])), ((int)(100*exp3->probability[FORWARDER_ARM])));

  // update EXP3 stats if needed
  if (exp3->collect_t) {
    // compute reward and update arms' probability
    float reward = dimmer_exp3_compute_reward(input, exp3->decision_tm2);
    #if KIEl
    printf("exp3 rew: %i\n", (int) (1000*reward));
    #endif
    // If not being a forwarder was harmful, we forcefully increase the probability of being a forwarder
    if (exp3->decision_tm2 == LEAF_ARM && reward <= -0.9f) {
      dimmer_exp3_update(exp3, FORWARDER_ARM, 1.0f);
      dimmer_exp3_punish_passivity(exp3);
    }
    dimmer_exp3_update(exp3, exp3->decision_tm2, reward);
  }

  if (execute_tp1 && !is_coordinator) {
    // we have DIMMER_EXP3_OPTIMIZATION_ROUND_NB consecutive rounds to update our probabilities
    if (dimmer_exp3_pseudorandom_order[(round_id/DIMMER_EXP3_OPTIMIZATION_ROUND_NB)%DIMMER_NUM_OF_NODES] == node_id) {
      uint8_t stay_forwarder = dimmer_exp3_run(exp3);
      // We need to try it and wait for the feedback
      exp3->decision_t = stay_forwarder;
      if (stay_forwarder) {
        lwb_set_conf_tx_cnt_data(dimmer_flood_N_parameter); // forwarder
      } else {
        lwb_set_conf_tx_cnt_data(0); // leaf
      }
      #if KIEL
      printf("exp3: stay_forwarder=%u, prob=(%i,%i), omeg=(%li,%li)\n", stay_forwarder,
                                                                      (int)(100*exp3->probability[0]),
                                                                      (int)(100*exp3->probability[1]),
                                                                      (long int)(1000*exp3->omega[0]),
                                                                      (long int)(1000*exp3->omega[1]));
      #endif
    } else {
      /* Keep action with highest probability */
      uint8_t stay_forwarder = ARGMAX(((int) (100*exp3->probability[LEAF_ARM])), ((int)(100*exp3->probability[FORWARDER_ARM])));
      #if KIEL
      printf("exp3: stay_forwarder=%u, prob=(%i,%i), omeg=(%li,%li)\n", stay_forwarder,
                                                                      (int)(100*exp3->probability[0]),
                                                                      (int)(100*exp3->probability[1]),
                                                                      (long int)(1000*exp3->omega[0]),
                                                                      (long int)(1000*exp3->omega[1]));
      #endif
      if (stay_forwarder) {
        lwb_set_conf_tx_cnt_data(dimmer_flood_N_parameter); // forwarder
      } else {
        lwb_set_conf_tx_cnt_data(0); // leaf
      }
    }
  }
}

#endif /* USE_DIMMER */