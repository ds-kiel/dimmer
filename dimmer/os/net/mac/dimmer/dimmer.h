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

#ifndef DIMMER_H_
#define DIMMER_H_

#include "contiki.h"

#undef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON 1

// Size of a fixed point integer
typedef int16_t dimmer_dqn_type_t;
typedef int32_t dimmer_dqn_compute_type_t;

enum {DIMMER_COMMAND_NO_CMD_RECEIVED = 0,
      DIMMER_COMMAND_APPLY_LOCAL_OPTIMIZATION = 1,
      DIMMER_COMMAND_EXECUTE_LOCAL_DQN = 2,
      DIMMER_COMMAND_EXECUTE_DECREASE_N_PARAMETER = 3,
      DIMMER_COMMAND_EXECUTE_KEEP_N_PARAMETER = 4,
      DIMMER_COMMAND_EXECUTE_INCREASE_N_PARAMETER = 5,
      DIMMER_COMMAND_USE_INITIATOR_N_PARAMETER = 6,
      DIMMER_COMMAND_EXECUTE_FORWARDER_SELECTION = 7};



// Number of nodes in the network, must be fixed
#ifndef DIMMER_NUM_OF_NODES
#define DIMMER_NUM_OF_NODES                              18
#endif

// Number of nodes in the network, must be fixed
#ifndef DIMMER_NUM_OF_INPUTS
#define DIMMER_NUM_OF_INPUTS                              10
#endif

#ifndef DIMMER_HISTORY_LENGTH
#define DIMMER_HISTORY_LENGTH                                2
#endif

#ifndef DIMMER_ONE_HOT_LENGTH
#define DIMMER_ONE_HOT_LENGTH                                9
#endif

#ifndef DIMMER_PRINT_FLOOD_STATISTICS
#define DIMMER_PRINT_FLOOD_STATISTICS                       1
#endif

#ifndef DIMMER_INITIAL_RETRANSMISSION_PARAM
#define DIMMER_INITIAL_RETRANSMISSION_PARAM                              3
#endif /* DIMMER_INITIAL_RETRANSMISSION_PARAM */

// Use the output of a DQN running outside of the node?
#ifndef DIMMER_USE_SERIAL_LINE_FOR_N_PARAMETER
#define DIMMER_USE_SERIAL_LINE_FOR_N_PARAMETER              0
#endif
// How often is the Dimmer update called?
#ifndef DIMMER_DQN_PERIOD_IN_FLOODS
#define DIMMER_DQN_PERIOD_IN_FLOODS                         3
#endif
// A normal NN requires input in [-1,1], what is our int scale?
#ifndef DIMMER_INPUT_SCALE
#define DIMMER_INPUT_SCALE                                  100
#endif
// To normalize radio-on time input, an upper bound is required
#ifndef DIMMER_MAX_RADIO_ON_TIME
#define DIMMER_MAX_RADIO_ON_TIME                            20
#endif
// To normalize the N parameter input, an upper bound is required
#ifndef DIMMER_MAX_N_PARAM
#define DIMMER_MAX_N_PARAM                                  8
#endif
// To normalize the N parameter input, an upper bound is required
#ifndef DIMMER_MAX_N_PARAM_NORMALIZATION
#define DIMMER_MAX_N_PARAM_NORMALIZATION                    9
#endif
// To normalize the N parameter input, a lower bound is required
#ifndef DIMMER_MIN_N_PARAM
#define DIMMER_MIN_N_PARAM                                  1
#endif
//
#ifndef DIMMER_HEADER_LEN
#define DIMMER_HEADER_LEN                                   3
#endif
//
#ifndef DIMMER_USE_MULTICHANNEL
#define DIMMER_USE_MULTICHANNEL                                   0
#endif

#ifndef DIMMER_CONF_SCHED_RF_CHANNEL
#define DIMMER_CONF_SCHED_RF_CHANNEL 26
#endif /* DIMMER_CONF_SCHED_RF_CHANNEL */

#ifndef DIMMER_CONF_SCHED_NUM_CHANNELS
#define DIMMER_CONF_SCHED_NUM_CHANNELS 4
#endif /* DIMMER_CONF_SCHED_NUM_CHANNELS */

#ifndef HOST_ID
#define HOST_ID                                             1
#endif

#ifndef IS_DIMMER_INITIATOR
#define IS_DIMMER_INITIATOR(id) (id == HOST_ID)
#endif

#ifndef DIMMER_FIXED_POINT_SCALE
#define DIMMER_FIXED_POINT_SCALE                                   100
#endif

#ifndef DIMMER_USE_SERIAL_LINE_FOR_NN
#define DIMMER_USE_SERIAL_LINE_FOR_NN                              0
#endif

#ifndef DIMMER_BOOTSTRAP_ROUNDS
#define DIMMER_BOOTSTRAP_ROUNDS                                     10
#endif

#ifndef DIMMER_ADAPTIVITY_PERIOD
#define DIMMER_ADAPTIVITY_PERIOD                                     2
#endif

void dimmer_init(void);
void dimmer_poll(void);
void dimmer_start_dqn_process(void);
void dimmer_poll_dqn_process(void);
uint8_t dimmer_is_source_node(void);
void dimmer_update_local_statistics_post_reception(uint8_t is_assigned_slot);
void dimmer_update_global_statistics_post_reception(unsigned short node_id, uint8_t* glossy_payload);
void dimmer_local_statistics_scale_input(void);
void dimmer_piggyback_local_statistics_in_packet(uint8_t* packet, uint8_t data_len);
void dimmer_reset_input_field(void);
uint8_t dimmer_get_flood_N_parameter(void);
void dimmer_reset_statistics(void);
void dimmer_start_serial_line_process(void);
void dimmer_reset_energy_measurement(void);
void dimmer_execute_command(uint8_t command, uint8_t additional_data);
uint8_t dimmer_PID_controller(uint8_t current_n, dimmer_dqn_compute_type_t input[]);
uint8_t dimmer_rate_controller(uint8_t current_n, dimmer_dqn_compute_type_t input[]);

#endif /* DIMMER_H_ */