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
#include <string.h>
#include "dimmer.h"
#include "dimmer-dqn.h"
#include "dimmer-exp3.h"
#include "glossy.h"
#include "node-id.h"
#include "lwb.h"
#include "debug-print.h"
#include "dimmer-testbed.h"
#if DIMMER_USE_SERIAL_LINE_FOR_NN
  #include "serial-line.h"
  #include <stdlib.h> 
#endif
#include "sys/energest.h"
#include "leds.h"

/*---------------------------------------------------------------------------*/
PROCESS(dimmer_controller_process, "Dimmer Deep-Q Neural Network (DQN)");
#if DIMMER_USE_SERIAL_LINE_FOR_NN
PROCESS(dimmer_serial_line_process, "Dimmer Serial Line Control");
#endif /* DIMMER_USE_SERIAL_LINE_FOR_NN */
/*---------------------------------------------------------------------------*/
static uint8_t dimmer_flood_N_parameter = DIMMER_INITIAL_RETRANSMISSION_PARAM; // [0,10]
static uint16_t dimmer_options = 0;
static uint16_t dimmer_floods_since_last_update = 0; /**< \brief Counter until next DQN call. */
static uint16_t dimmer_lcl_stats_packet_received = 0; /**< \brief Current number of received packets. */
static uint16_t dimmer_lcl_stats_packet_missed = 0; /**< \brief Current number of missed packets. */
static uint32_t dimmer_lcl_stats_radio_on_sum = 0; /**< \brief Amount of radio-on used since last DQN call. */
static dimmer_dqn_compute_type_t dimmer_network_statistics[3*DIMMER_NUM_OF_NODES];
static dimmer_dqn_compute_type_t dimmer_output_probability[3];
static dimmer_dqn_compute_type_t dimmer_dqn_input[2*DIMMER_NUM_OF_INPUTS+DIMMER_ONE_HOT_LENGTH+DIMMER_HISTORY_LENGTH];
static uint16_t dimmer_bootstrap_rounds_before_dqn_execution = DIMMER_BOOTSTRAP_ROUNDS;
static uint8_t execute_local_optimization = 0;
static uint8_t reset_statistics = 0;
static uint8_t dimmer_nb_rounds_since_last_update = 0;
#if DIMMER_EVAL_USE_HANDCRAFTED_CONTROLLER
int16_t pid_I = 0;
#endif
#if DIMMER_USE_SERIAL_LINE_FOR_NN 
static uint16_t log_radiontimes[DIMMER_NUM_OF_NODES+2];
static uint16_t log_reliabilities[DIMMER_NUM_OF_NODES+2];
static uint8_t log_nparams[DIMMER_NUM_OF_NODES+2];
static uint8_t log_cnt = 0;
#endif /* DIMMER_USE_SERIAL_LINE_FOR_NN */
/*---------------------------------------------------------------------------*/
/* BEGIN AUTOMATICALLY GENERATED NEURAL NETWORK */
#undef FIXED_POINT_SCALE
#define FIXED_POINT_SCALE 100
#undef DIMMER_DQN_LAYER0_SIZE
#define DIMMER_DQN_LAYER0_SIZE 30
#undef DIMMER_DQN_LAYER1_SIZE
#define DIMMER_DQN_LAYER1_SIZE 3
#undef DIMMER_DQN_OUTPUT_SIZE
#define DIMMER_DQN_OUTPUT_SIZE 3
#undef DIMMER_DQN_MAX_LAYER_SIZE
#define DIMMER_DQN_MAX_LAYER_SIZE 30
static struct dimmer_dqn_config_struct enn_config = {
//layer sizes
   {30,3,}
};

static struct dimmer_dqn_nn_struct enn_dqn = {
   {NULL, NULL}, // pointers to weights
   {NULL, NULL}, // pointers to biases
// layer 0 - weights
   {-18,-21,56,-64,-69,-83,-61,-6,-12,-74,121,817,1021,1219,1302,1335,1376,1418,1394,1437,-25,371,488,514,629,660,653,613,588,-9,44,734,908,928,835,537,576,538,505,456,414,-747,-71,182,339,722,786,837,899,921,972,-3,551,424,511,541,674,679,605,668,-111,-199,1,7,90,-49,-51,-56,-29,19,-30,-71,209,816,1068,1199,1290,1359,1383,1410,1386,1403,8,305,295,485,599,642,701,712,639,11,52,9,-95,-34,-143,-129,-182,-141,-131,-108,-186,109,780,1043,1157,1236,1318,1307,1393,1376,1374,-26,516,257,390,527,553,630,667,698,-92,-36,59,-18,72,-20,-68,-78,-42,9,-45,-53,149,834,1075,1227,1298,1363,1346,1375,1406,1402,16,486,182,419,514,593,680,723,785,35,40,-10,-43,38,-66,-53,-56,-74,-26,-48,-136,156,811,1063,1194,1283,1306,1324,1366,1420,1419,3,342,489,556,590,639,624,618,514,-10,49,250,223,333,203,60,136,146,166,143,55,87,761,965,1055,1289,1330,1322,1364,1382,1411,26,360,146,510,585,655,731,811,786,57,69,142,116,219,105,-22,48,77,68,33,-9,109,787,1053,1163,1333,1356,1357,1397,1395,1410,-23,458,235,511,583,644,656,746,769,39,45,-13,-111,-23,-103,-129,-118,-88,-121,-94,-146,97,803,1049,1212,1262,1333,1345,1434,1432,1430,29,485,548,459,575,632,636,570,586,0,24,60,18,94,-31,-14,-6,11,27,18,-14,202,869,1103,1204,1308,1308,1391,1399,1397,1402,-7,350,235,455,562,600,663,740,758,41,55,-192,-164,-140,-179,-159,-205,-226,-229,-216,-288,-45,605,861,1023,1173,1226,1207,1262,1295,1314,-21,481,742,368,410,423,401,341,415,-32,-107,-640,-99,-143,-215,-352,-236,-349,-463,-405,-431,-609,-10,254,405,572,644,680,692,715,721,22,346,864,327,-475,-34,304,490,308,34,-130,-9,-40,74,-53,-65,-27,1,8,-40,-63,170,847,1037,1192,1301,1343,1327,1390,1423,1415,-25,426,325,494,555,601,636,700,694,-6,60,51,-16,90,-19,-80,-52,-9,-15,-7,-79,163,793,1035,1210,1293,1340,1350,1425,1380,1430,3,451,195,394,505,592,660,682,762,14,6,-200,-182,-145,-167,-189,-242,-210,-232,-235,-284,-3,690,959,1084,1202,1274,1301,1316,1329,1362,25,496,698,435,522,514,488,392,393,-8,-47,59,9,48,-36,-91,-37,-20,0,-45,-106,166,822,1064,1215,1300,1341,1389,1426,1380,1413,2,464,298,469,563,622,620,657,709,27,21,24,18,83,10,-40,-7,34,41,-16,-59,207,841,1078,1186,1306,1322,1388,1393,1412,1392,1,358,295,485,591,614,703,714,677,3,14,31,-32,49,-17,-11,-22,3,7,14,-44,162,818,1070,1220,1275,1323,1366,1411,1409,1444,-2,408,384,517,561,623,649,653,641,-21,41,-521,-577,-519,-648,-705,-650,-602,-629,-668,-718,-244,297,530,676,781,841,840,889,898,885,-16,530,92,-171,37,395,717,542,158,8,-31,21,-29,71,-58,-72,-70,-32,-3,-35,-103,115,823,1038,1192,1321,1324,1386,1419,1423,1441,-24,529,274,440,555,597,618,699,743,3,11,598,527,574,581,334,276,245,180,164,169,-864,-326,-122,57,376,420,469,505,527,559,-5,495,466,319,400,419,290,202,513,100,214,-463,-469,-365,-487,-447,-444,-496,-481,-488,-532,-102,545,787,894,1004,1048,1127,1148,1138,1134,-8,346,633,384,320,147,424,465,290,-92,-176,410,388,428,318,47,103,139,155,141,117,-177,473,700,842,1146,1149,1224,1269,1280,1246,-6,681,-93,195,322,485,672,857,1096,40,-59,-1,20,94,-30,-26,-32,11,53,38,-63,229,816,1031,1192,1280,1280,1334,1375,1387,1397,15,287,197,482,556,608,635,748,675,-15,-4,11,-33,35,-38,-88,-62,-84,-34,-74,-116,151,811,1032,1193,1270,1356,1357,1403,1430,1428,14,511,411,477,561,618,640,643,717,15,47,173,146,224,88,8,80,50,122,78,20,174,758,996,1166,1285,1349,1389,1381,1393,1409,3,407,126,431,576,640,661,777,794,52,83,7,-55,69,-60,-88,-41,-67,-27,-35,-73,136,810,1045,1202,1296,1362,1385,1430,1439,1442,-20,432,400,478,576,621,647,643,636,36,35,-22,-76,41,-121,-159,-151,-115,-63,-105,-136,78,721,964,1102,1216,1248,1299,1351,1330,1334,-19,476,169,340,436,480,584,702,829,-137,-100,66,-34,106,-33,-60,-30,-47,21,-4,-49,180,853,1092,1236,1336,1368,1400,1403,1399,1403,-16,436,347,468,614,655,674,679,697,37,27,56,-10,47,-15,-75,-70,-20,-25,-2,-109,142,811,1025,1218,1278,1338,1378,1406,1407,1435,-29,496,247,422,544,591,622,699,754,46,28,},
// layer 0 - biases
   {1699,1832,1685,1676,1694,1689,1752,1740,1700,1694,1624,1192,1683,1695,1668,1694,1689,1695,1221,1698,1348,1463,1742,1662,1700,1728,1707,1627,1709,1698,},
// layer 1 - weights
   {784,645,826,907,875,768,824,823,794,822,696,505,847,858,727,844,817,804,828,868,398,724,860,829,828,840,805,941,805,850,866,785,867,842,789,872,819,810,839,807,815,696,854,786,849,813,847,842,632,805,400,835,654,853,813,803,819,852,807,783,808,976,758,868,792,807,691,746,870,726,966,918,801,785,982,806,745,781,907,830,1096,924,702,711,840,713,815,819,769,801,},
// layer 1 - biases
   {1379,1399,1762,},
// activation function per layer
   {DIMMER_ACTIVATION_FN_RELU,DIMMER_ACTIVATION_FN_RELU,},
};


static struct dimmer_exp3_struct dimmer_exp3;

/* END AUTOMATICALLY GENERATED NEURAL NETWORK */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
#define DIMMER_SET_INITIATOR_COMMAND(command) (dimmer_options = ((dimmer_options)&0xff) | ((command)<<8) )
#define DIMMER_GET_INITIATOR_COMMAND(options) ((options)>>8)
#define DIMMER_SET_INITIATOR_ADDITIONAL_DATA(data) (dimmer_options = ((dimmer_options)&0xff00) | (data) )
#define DIMMER_GET_INITIATOR_ADDITIONAL_DATA(options) ((options)&0xff)
/*---------------------------------------------------------------------------*/
#define DIMMER_DECREASE_N_PARAM() \
  {\
    if (dimmer_flood_N_parameter <= DIMMER_MIN_N_PARAM) {\
        dimmer_flood_N_parameter = DIMMER_MIN_N_PARAM;\
      } else {\
        --dimmer_flood_N_parameter;\
      }\
      lwb_set_conf_tx_cnt_data(dimmer_flood_N_parameter);\
      lwb_set_conf_tx_cnt_sched(dimmer_flood_N_parameter);\
  }
/*---------------------------------------------------------------------------*/
#define DIMMER_INCREASE_N_PARAM() \
  {\
    if (dimmer_flood_N_parameter >= DIMMER_MAX_N_PARAM) {\
        dimmer_flood_N_parameter = DIMMER_MAX_N_PARAM;\
      } else {\
        ++dimmer_flood_N_parameter;\
      }\
      lwb_set_conf_tx_cnt_data(dimmer_flood_N_parameter);\
      lwb_set_conf_tx_cnt_sched(dimmer_flood_N_parameter);\
  }
/*---------------------------------------------------------------------------*/
#define DIMMER_SET_N_PARAM(n) \
  {\
    dimmer_flood_N_parameter = n; \
    if (dimmer_flood_N_parameter <= DIMMER_MIN_N_PARAM) {\
        dimmer_flood_N_parameter = DIMMER_MIN_N_PARAM;\
      } else if (dimmer_flood_N_parameter >= DIMMER_MAX_N_PARAM) {\
        dimmer_flood_N_parameter = DIMMER_MAX_N_PARAM;\
      } \
      lwb_set_conf_tx_cnt_data(dimmer_flood_N_parameter);\
      lwb_set_conf_tx_cnt_sched(dimmer_flood_N_parameter);\
  }
/*---------------------------------------------------------------------------*/
 //dimmer_local_statistics_scale_input();
#define DIMMER_EXECUTE_AND_APPLY_DQN() \
  {\
    dimmer_dqn_run(&enn_config, &enn_dqn, dimmer_network_statistics, dimmer_output_probability);\
    if (dimmer_output_probability[0] > dimmer_output_probability[1]\
        && dimmer_output_probability[0] > dimmer_output_probability[2]) {\
      DIMMER_DECREASE_N_PARAM();\
    } else if (dimmer_output_probability[2] > dimmer_output_probability[0]\
                && dimmer_output_probability[2] > dimmer_output_probability[1]) {\
      DIMMER_INCREASE_N_PARAM();\
    }\
  }
/*---------------------------------------------------------------------------*/
 //dimmer_local_statistics_scale_input();
#define DIMMER_EXECUTE_AND_REPORT_DQN() \
  {\
    leds_on(LEDS_RED);\
    dimmer_dqn_select_n_worst_nodes(dimmer_network_statistics, dimmer_dqn_input, DIMMER_NUM_OF_INPUTS);\
    dimmer_dqn_update_one_hot(dimmer_dqn_input, dimmer_flood_N_parameter);\
    dimmer_dqn_run(&enn_config, &enn_dqn, dimmer_dqn_input, dimmer_output_probability);\
    if (dimmer_output_probability[0] > dimmer_output_probability[1]\
        && dimmer_output_probability[0] > dimmer_output_probability[2]) {\
        DIMMER_SET_INITIATOR_COMMAND(DIMMER_COMMAND_USE_INITIATOR_N_PARAMETER);\
        DIMMER_SET_INITIATOR_ADDITIONAL_DATA(dimmer_flood_N_parameter-1);\
    } else if (dimmer_output_probability[2] > dimmer_output_probability[0]\
                && dimmer_output_probability[2] > dimmer_output_probability[1]) {\
      DIMMER_SET_INITIATOR_COMMAND(DIMMER_COMMAND_USE_INITIATOR_N_PARAMETER);\
      DIMMER_SET_INITIATOR_ADDITIONAL_DATA(dimmer_flood_N_parameter+1);\
    } else { \
      DIMMER_SET_INITIATOR_COMMAND(DIMMER_COMMAND_USE_INITIATOR_N_PARAMETER);\
      DIMMER_SET_INITIATOR_ADDITIONAL_DATA(dimmer_flood_N_parameter); \
    } \
    dimmer_dqn_update_historical_values(dimmer_dqn_input);\
    leds_off(LEDS_RED);\
  }
/*---------------------------------------------------------------------------*/
#if KIEL
#define DIMMER_NODE_ID node_id
#define DIMMER_ID(x) x
#elif COOJA
#define DIMMER_NODE_ID node_id
#define DIMMER_ID(x) x
#else
#define DIMMER_NODE_ID node_id
#define DIMMER_ID(x) x
#endif
/*---------------------------------------------------------------------------*/
void dimmer_init(void)
{
  // init DQN process
  dimmer_start_dqn_process();
  // set default LWB N values
  lwb_set_conf_tx_cnt_data(dimmer_flood_N_parameter);
  lwb_set_conf_tx_cnt_sched(dimmer_flood_N_parameter);
  // Init nominal state optimization
  dimmer_exp3_init(&dimmer_exp3);
  // Init history
  int i;
  for (i=2*DIMMER_NUM_OF_INPUTS+DIMMER_ONE_HOT_LENGTH;i<2*DIMMER_NUM_OF_INPUTS+DIMMER_ONE_HOT_LENGTH+DIMMER_HISTORY_LENGTH;++i) {
    dimmer_dqn_input[i] = 100;
  }
 #if DIMMER_USE_SERIAL_LINE_FOR_NN
  dimmer_start_serial_line_process();
 #endif /* DIMMER_USE_SERIAL_LINE_FOR_NN */
}
/*---------------------------------------------------------------------------*/
void dimmer_poll(void)
{
  dimmer_poll_dqn_process();
}
/*---------------------------------------------------------------------------*/
void dimmer_start_dqn_process(void)
{
  process_start(&dimmer_controller_process, NULL);
}
/*---------------------------------------------------------------------------*/
#if DIMMER_USE_SERIAL_LINE_FOR_NN
void dimmer_start_serial_line_process(void)
{
  serial_line_init();
  process_start(&dimmer_serial_line_process, NULL);
}
#endif /* DIMMER_USE_SERIAL_LINE_FOR_NN */
/*---------------------------------------------------------------------------*/
void dimmer_poll_dqn_process(void)
{
  process_poll(&dimmer_controller_process);
}
/*---------------------------------------------------------------------------*/
uint8_t dimmer_is_source_node(void)
{
  /* We can't have more than 20 flows registered, otherwise memory is full */
  #if GRAZ
    if (node_id<=DIMMER_NUM_OF_NODES) return 1;
  #elif KIEL
    return 1;
  #elif FLOCKLAB
    return 1;
  #else
    if (node_id<20) return 1;
  #endif
  return 0;
}
/*---------------------------------------------------------------------------*/
void dimmer_reset_energy_measurement(void) {
  energest_init();
}
/*---------------------------------------------------------------------------*/
void dimmer_update_local_statistics_post_reception(uint8_t is_assigned_slot)
{
  // Packet received during this flood?
  if (glossy_get_rx_cnt()) {
    dimmer_lcl_stats_packet_received++;
  } else {
    if (is_assigned_slot) {
      dimmer_lcl_stats_packet_missed++;
    }
  }
  // Compute average radio-on time, in microseconds.
  // energy used during last flood
  energest_flush();
  // energy since last DQN call (in microsecond)
  uint16_t radioon_this_flood = (uint16_t) (1000000 * (energest_type_time(ENERGEST_TYPE_LISTEN) + energest_type_time(ENERGEST_TYPE_TRANSMIT)) / ENERGEST_SECOND);
  dimmer_lcl_stats_radio_on_sum += radioon_this_flood;
  // reset energest values
  energest_init();

 #if DIMMER_USE_SERIAL_LINE_FOR_NN
  log_radiontimes[log_cnt] = radioon_this_flood;
  log_reliabilities[log_cnt] = (uint16_t) (((uint32_t) dimmer_lcl_stats_packet_received)*10000/((uint32_t)(dimmer_lcl_stats_packet_received+dimmer_lcl_stats_packet_missed)));
  log_nparams[log_cnt] = dimmer_flood_N_parameter;
  log_cnt = (log_cnt+1)%(DIMMER_NUM_OF_NODES+2);
 #endif /* DIMMER_USE_SERIAL_LINE_FOR_NN */
}
/*---------------------------------------------------------------------------*/
void dimmer_local_statistics_scale_input(void)
{
  // get average radio-on time per flood
  unsigned long radio_on_unscaled = dimmer_lcl_stats_radio_on_sum / (unsigned long) (dimmer_lcl_stats_packet_received + dimmer_lcl_stats_packet_missed);
  // Transform radio-on time into [-1,1]*DIMMER_INPUT_SCALE
  dimmer_dqn_compute_type_t radio_on_scaled = ( (dimmer_dqn_compute_type_t) (radio_on_unscaled) * DIMMER_INPUT_SCALE / 1000 * 2 / DIMMER_MAX_RADIO_ON_TIME - DIMMER_INPUT_SCALE);
  // Transform packer reception ratio into [-1,1]*DIMMER_INPUT_SCALE
  dimmer_dqn_compute_type_t packet_ratio_scaled = ((dimmer_dqn_compute_type_t)(dimmer_lcl_stats_packet_received) * 2000) / ((dimmer_dqn_type_t)(dimmer_lcl_stats_packet_received + dimmer_lcl_stats_packet_missed));
  // Important, we only consider reliabilities between 50% and 100%
  packet_ratio_scaled = (MAX(0, packet_ratio_scaled - 1000)-500)/5;
  dimmer_dqn_compute_type_t n_param_scaled = ((dimmer_dqn_compute_type_t) dimmer_flood_N_parameter) * 2 * DIMMER_INPUT_SCALE/DIMMER_MAX_N_PARAM_NORMALIZATION - DIMMER_INPUT_SCALE;
  radio_on_scaled = MIN(100, radio_on_scaled);
  radio_on_scaled = MAX(-100, radio_on_scaled);
  packet_ratio_scaled = MIN(100, packet_ratio_scaled);
  packet_ratio_scaled = MAX(-100, packet_ratio_scaled);
  dimmer_network_statistics[DIMMER_NODE_ID-1] = radio_on_scaled;
  dimmer_network_statistics[DIMMER_NUM_OF_NODES + DIMMER_NODE_ID -1] = packet_ratio_scaled;
  dimmer_network_statistics[2*DIMMER_NUM_OF_NODES + DIMMER_NODE_ID -1] = n_param_scaled;
}
/*---------------------------------------------------------------------------*/
void dimmer_update_global_statistics_post_reception(unsigned short sender_id, uint8_t* glossy_payload)
{
  if (sender_id > 0) {
    dimmer_network_statistics[DIMMER_ID(sender_id) - 1] = (dimmer_dqn_compute_type_t) ((int8_t) glossy_payload[LWB_HEADER_LEN]);
    dimmer_network_statistics[DIMMER_NUM_OF_NODES + DIMMER_ID(sender_id) - 1] = (dimmer_dqn_compute_type_t) ((int8_t) glossy_payload[LWB_HEADER_LEN + 1]);
    dimmer_network_statistics[2*DIMMER_NUM_OF_NODES + DIMMER_ID(sender_id) - 1] = (dimmer_dqn_compute_type_t) ((int8_t) glossy_payload[LWB_HEADER_LEN + 2]);
  }
}
/*---------------------------------------------------------------------------*/
void dimmer_piggyback_local_statistics_in_packet(uint8_t* packet, uint8_t data_len)
{
  int8_t radioon = (int8_t) (dimmer_network_statistics[DIMMER_NODE_ID-1]);
  int8_t reliability = (int8_t) (dimmer_network_statistics[DIMMER_NUM_OF_NODES + DIMMER_NODE_ID-1]);
  int8_t n_param = (int8_t) dimmer_network_statistics[2*DIMMER_NUM_OF_NODES + DIMMER_NODE_ID -1];

  *(packet + LWB_HEADER_LEN) = radioon;
  *(packet + LWB_HEADER_LEN + 1) = reliability;
  *(packet + LWB_HEADER_LEN + 2) = n_param;
}
/*---------------------------------------------------------------------------*/
void dimmer_reset_statistics(void)
{
    // reset all data received from other nodes, except N parameter
    int i;
    for (i=0;i<DIMMER_NUM_OF_NODES;++i) {
      dimmer_network_statistics[i] = 100;
      dimmer_network_statistics[DIMMER_NUM_OF_NODES+i] = -100;
    }
    dimmer_local_statistics_scale_input();
    // reset counter until next update
    dimmer_floods_since_last_update = 0;
    // reset all statistics
    dimmer_lcl_stats_packet_missed = 0;
    dimmer_lcl_stats_packet_received = 0;
    dimmer_lcl_stats_radio_on_sum = 0;
}
/*---------------------------------------------------------------------------*/
void dimmer_execute_command(uint8_t command, uint8_t additional_data)
{
  if (command == DIMMER_COMMAND_NO_CMD_RECEIVED) {
    return;
  } else if (command == DIMMER_COMMAND_APPLY_LOCAL_OPTIMIZATION) {
    execute_local_optimization = 1;
    return;
  } else if (command == DIMMER_COMMAND_EXECUTE_LOCAL_DQN) {
    DIMMER_EXECUTE_AND_APPLY_DQN();
    reset_statistics = 1;
    return;
  } else if (command == DIMMER_COMMAND_EXECUTE_DECREASE_N_PARAMETER) {
    DIMMER_DECREASE_N_PARAM();
    reset_statistics = 1;
    return;
  } else if (command == DIMMER_COMMAND_EXECUTE_KEEP_N_PARAMETER) {
    reset_statistics = 1;
    return;
  } else if (command == DIMMER_COMMAND_EXECUTE_INCREASE_N_PARAMETER) {
    DIMMER_INCREASE_N_PARAM();
    reset_statistics = 1;
    return;
  } else if (command == DIMMER_COMMAND_USE_INITIATOR_N_PARAMETER) {
    DIMMER_SET_N_PARAM(additional_data);
    reset_statistics = 1;
    return;
  }
}
/*---------------------------------------------------------------------------*/
#if DIMMER_USE_SERIAL_LINE_FOR_NN
void dimmer_print_stats_for_gym(void)
{
  #if !GRAZ
  uint8_t i = 0;
  for (i=0; i<log_cnt; ++i) {
      printf("res:%i:0:%u:%u:%i/%i:%i\n",
                                log_nparams[i],
                                log_radiontimes[i],
                                log_reliabilities[i],
                                dimmer_lcl_stats_packet_received,
                                dimmer_lcl_stats_packet_received+dimmer_lcl_stats_packet_missed,
                                lwb_get_last_round_id());
    }
  #endif
  log_cnt=0;
}
#endif /* DIMMER_USE_SERIAL_LINE_FOR_NN */
/*---------------------------------------------------------------------------*/
void dimmer_debug_print_input_vector(void)
{
    uint8_t i = 0;
  printf("inpt: [ ");
    for (i = 0; i < 3*DIMMER_NUM_OF_NODES; ++i) {
      printf("%li, ", (long int) dimmer_network_statistics[i]);
    }
    printf("]\n");
}
/*---------------------------------------------------------------------------*/
void dimmer_debug_print_received_command(void)
{
  printf("dimcmd: rd %u n %u cmd %u (%u)\n", lwb_get_last_round_id(),
                                             dimmer_flood_N_parameter,
                                             DIMMER_GET_INITIATOR_COMMAND(dimmer_options),
                                             DIMMER_GET_INITIATOR_ADDITIONAL_DATA(dimmer_options));
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* dimmer_controller_process: Upon polling, executing the neural network based on the aggregated stats */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dimmer_controller_process, ev, data)
{
  PROCESS_BEGIN();
  // Init DQN
  enn_dqn.weights[0] = enn_dqn.layer0_weights;
  enn_dqn.biases[0] = enn_dqn.layer0_biases;
  enn_dqn.weights[1] = enn_dqn.layer1_weights;
  enn_dqn.biases[1] = enn_dqn.layer1_biases;

  // todo remove
  dimmer_dqn_input[0] = 0;

    /* main Dimmer loop */
  while(1) {
    /* polled by the application at the end of an LWB round */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    // set again local input (to send it to other nodes)
    if (node_id < DIMMER_NUM_OF_NODES) {
      dimmer_local_statistics_scale_input();
    }

    // print Dimmer stats
 #if KIEL
    dimmer_debug_print_input_vector();
  #endif
      printf("Dmr:%u:%i:%li:%li\n", lwb_get_last_round_id(),
                                  dimmer_flood_N_parameter,
                                  dimmer_network_statistics[DIMMER_NODE_ID-1],
                                  dimmer_network_statistics[DIMMER_NUM_OF_NODES+DIMMER_NODE_ID-1]);

 #if DIMMER_USE_SERIAL_LINE_FOR_NN
    dimmer_print_stats_for_gym();
 #endif /* DIMMER_USE_SERIAL_LINE_FOR_NN */

    /* Bootstraping*/
    // Wait a certain amount of rounds before adaptiveness (to force LWB to receive all flows)
    if (dimmer_bootstrap_rounds_before_dqn_execution > 0) {
      --dimmer_bootstrap_rounds_before_dqn_execution;
      dimmer_reset_statistics();
      continue;
    }

/* Adaptivity Policy Control (Local execution) */
    /* read Dimmer command sent by initiator as part of the second schedule */
    reset_statistics = 0;
    dimmer_options = lwb_get_dimmer_options();
    execute_local_optimization = 0;
    dimmer_execute_command(DIMMER_GET_INITIATOR_COMMAND(dimmer_options), DIMMER_GET_INITIATOR_ADDITIONAL_DATA(dimmer_options));
    /* reset command */
    DIMMER_SET_INITIATOR_COMMAND(DIMMER_COMMAND_NO_CMD_RECEIVED);
    DIMMER_SET_INITIATOR_ADDITIONAL_DATA(0x00);
    /* end Adaptivity Policy Control (Local execution) */


    /* Forwarder Selection (Distributed) */
    dimmer_exp3_optimize(&dimmer_exp3,
                         lwb_get_last_round_id(),
                         0,// execute_local_optimization
                         dimmer_network_statistics,
                         dimmer_flood_N_parameter,
                         IS_DIMMER_INITIATOR(node_id));
    if (execute_local_optimization) {
      dimmer_reset_statistics();
      execute_local_optimization = 0;
    }



    /* Adaptivity Policy Control (Central Control) */
    if(IS_DIMMER_INITIATOR(DIMMER_NODE_ID)) {
      if (dimmer_nb_rounds_since_last_update < 1) {
        ++dimmer_nb_rounds_since_last_update;
        DIMMER_SET_INITIATOR_COMMAND(DIMMER_COMMAND_EXECUTE_KEEP_N_PARAMETER);
        DIMMER_SET_INITIATOR_ADDITIONAL_DATA(0x00);
        continue;
      }
      dimmer_nb_rounds_since_last_update = 0;

 #if DIMMER_USE_EMBEDDED_NN // use embedded DQN
      DIMMER_EXECUTE_AND_REPORT_DQN();
 #elif DIMMER_EVAL_USE_HANDCRAFTED_CONTROLLER // use PID controller
      uint8_t new_N = dimmer_PID_controller(dimmer_flood_N_parameter, dimmer_network_statistics);
      DIMMER_SET_INITIATOR_COMMAND(DIMMER_COMMAND_USE_INITIATOR_N_PARAMETER);\
      DIMMER_SET_INITIATOR_ADDITIONAL_DATA(new_N);\
 #else /* DIMMER_USE_EMBEDDED_NN */ // No embedded DQN or PID, just apply local optimization
      /* LWB(3) */
      DIMMER_SET_INITIATOR_COMMAND(DIMMER_COMMAND_USE_INITIATOR_N_PARAMETER);
      DIMMER_SET_INITIATOR_ADDITIONAL_DATA(DIMMER_INITIAL_RETRANSMISSION_PARAM);
 #endif /* DIMMER_USE_EMBEDDED_NN */

      /* Send new command to be executed by all nodes at the end of the next LWB round */
      lwb_set_dimmer_options(dimmer_options);
    } /* end Adaptivity Policy Control (Central Control) */

    
    if (reset_statistics) {
      dimmer_reset_statistics();
    }

  } /* end forever loop */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* dimmer_serial_line_process: Upon event on serial line, read */
/*---------------------------------------------------------------------------*/
#if DIMMER_USE_SERIAL_LINE_FOR_NN
PROCESS_THREAD(dimmer_serial_line_process, ev, data)
{
  PROCESS_BEGIN();
  while (1) {
    PROCESS_YIELD();
    if (ev == serial_line_event_message) {
      // we received something on the serial line
        uint8_t n = atoi((char *)data);
        dimmer_reset_statistics();
        if (n == 101) {
          // can't bootstrap anymore!
          dimmer_flood_N_parameter = DIMMER_INITIAL_RETRANSMISSION_PARAM;
        } else if (n == 102) {
          // do nothing
        } else {
          dimmer_flood_N_parameter = n;
        }
      lwb_set_conf_tx_cnt_data(dimmer_flood_N_parameter);
      lwb_set_conf_tx_cnt_sched(dimmer_flood_N_parameter);
    }
  }
  PROCESS_END();
}
#endif /* DIMMER_USE_SERIAL_LINE_FOR_NN */



#if DIMMER_EVAL_USE_HANDCRAFTED_CONTROLLER
uint8_t dimmer_PID_controller(uint8_t current_n, dimmer_dqn_compute_type_t input[]) {
  // find minimum reliability
  int16_t min_rel = 100;
  uint8_t chosen_n = current_n;
  uint8_t i;
  for (i=0; i < DIMMER_NUM_OF_NODES; ++i) {
    printf("%i ", (int16_t) input[DIMMER_NUM_OF_NODES+i]);
    if (input[DIMMER_NUM_OF_NODES+i] < min_rel) {
      min_rel = (int16_t) input[DIMMER_NUM_OF_NODES+i];
    }
  }
  printf("-> %i \n", min_rel);
  // we want above 99% reliability (Input: -100 = 50%, 100 = 100%) ==> 98 in input field
  int16_t required_rel = 98;

  int16_t e = (required_rel - min_rel);
  pid_I = MIN(5, pid_I+e); // avoid huge bias towards increasing for too long

  int16_t pid = (1*e + (1*pid_I)/4);
  printf("PID: %i\n", pid);
  if (pid > 3) {
    chosen_n = current_n + 1;
  } else if (pid < -3) {
    chosen_n = current_n - 1;
    // add some value to pid_I to avoid fast convergence towards 0
    pid_I += 3;
  } else {
    chosen_n = current_n;
  }

  return chosen_n;
}

uint8_t dimmer_rate_controller(uint8_t current_n, dimmer_dqn_compute_type_t input[]) {
  // find minimum reliability
  uint8_t suffered_losses = 0;
  uint8_t chosen_n = current_n;
  uint8_t i;
  for (i=0; i < DIMMER_NUM_OF_NODES; ++i) {
   if (input[DIMMER_NUM_OF_NODES+i] < 100) {
     suffered_losses = 1;
     break;
   }
  }
  if (suffered_losses) {
    chosen_n = MIN(8, 2*current_n);
  } else {
    if (chosen_n == 1) {
      chosen_n = 1;
    } else {
      chosen_n -= 1;
    }
  }
  return chosen_n;

}
#endif /* DIMMER_EVAL_USE_HANDCRAFTED_CONTROLLER */

#endif /* USE_DIMMER */