/*
 * Copyright (c) 2020 Kiel University.
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
 
/**
 * @brief Dimmer Example Application
 */

#include "contiki.h"
#include "lwb.h"
#include "debug-print.h"
#include "dc-stat.h"
#include "gpio.h"
#include "node-id.h"
#include "glossy.h"
#include "cc2420.h"
#include "leds.h"
#if USE_DIMMER
#include "dimmer.h"
#include "dimmer-testbed.h"
#define ENERGEST_CONF_ON 1
#endif

/*---------------------------------------------------------------------------*/

#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */


/* Packet buffer */
static uint8_t pkt_buffer[LWB_CONF_MAX_DATA_PKT_LEN];
static uint16_t sender_id, pkt_id;
static uint8_t ack_pos;
static uint8_t dimmer_app_acks_sender_id[DIMMER_NUM_ACKS] = {0};
static uint8_t dimmer_app_acks_first_byte[DIMMER_NUM_ACKS] = {0};


/* Create one packet for eval */
void create_one_packet()
{
  if ( lwb_get_send_buffer_state() == 0) {
    memset(pkt_buffer, 0, LWB_CONF_MAX_DATA_PKT_LEN - 3);
    pkt_buffer[0] = (uint8_t) (node_id >> 8);
    pkt_buffer[1] = (uint8_t) (node_id & 0xff);
    pkt_buffer[2] = (uint8_t) (pkt_id >> 8);
    pkt_buffer[3] = (uint8_t) (pkt_id & 0xff);
    #if USE_DIMMER
    uint8_t success = lwb_send_pkt(0xffff, 1, pkt_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN - DIMMER_HEADER_LEN);
    #else /* USE_DIMMER */
    uint8_t success = lwb_send_pkt(0xffff, 1, pkt_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN);
    #endif /* USE_DIMMER */
    if (success) {
      ++pkt_id;
      printf("pkt:%u:%u\n", (uint16_t) ((pkt_buffer[0] << 8) | pkt_buffer[1]), (uint16_t) ((pkt_buffer[2] << 8) | pkt_buffer[3]));
    }
  }
}

/* print received packets to serial line - for eval */
void read_received_packets()
{
  uint16_t pkt_len;
  do {
    pkt_len = lwb_rcv_pkt(pkt_buffer, &sender_id, 0);
    printf("pkt:%u:%u:%u\n",
           (uint16_t) ((pkt_buffer[0] << 8) | pkt_buffer[1]),
           (uint16_t) ((pkt_buffer[2] << 8) | pkt_buffer[3]),
           lwb_get_last_round_id());
    #if DIMMER_USE_ACKS
      if (node_id == HOST_ID) {
        lwb_set_dimmer_ack(ack_pos, (uint8_t)sender_id, pkt_buffer[0]);
        ack_pos = (ack_pos+1) % DIMMER_NUM_ACKS;
      }
    #endif /* DIMMER_USE_ACKS */
  } while(pkt_len);
}

/*---------------------------------------------------------------------------*/
/* lwb_application_process: Application level code, run LWB */
/*---------------------------------------------------------------------------*/
PROCESS(lwb_application_process, "Dimmer Example Application");
AUTOSTART_PROCESSES(&lwb_application_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lwb_application_process, ev, data)
{
  static uint8_t stream_state = 0;

  PROCESS_BEGIN();

  /* Dimmer requires that the node_id starts at 1 and are consecutive
   * We remap the node-id internally (but not the one saved in hardware!)
   * Mapping is in os/net/mac/dimmer/dimmer-testbed.h
   */
  #if USE_DIMMER
    init_mapped_node_id();
    node_id = mapped_node_id;
  #endif
  cc2420_set_txpower(0xff);


#if USE_DIMMER
  dimmer_init();
#endif
  lwb_start(0, &lwb_application_process, node_id == HOST_ID);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
    if(HOST_ID == node_id) {
      /* LWB coordinator */
      read_received_packets();
     /* Acknowledgements are done in read_received_packets() */
    } else {
      /* flow-source */
      /* Print all packets received for evaluation */
      read_received_packets();
      if(stream_state != LWB_STREAM_STATE_ACTIVE) {
        stream_state = lwb_stream_get_state(1);
        if(stream_state == LWB_STREAM_STATE_INACTIVE) {
          /* request a stream */
          lwb_stream_req_t my_stream = { node_id, 0, 1, SOURCE_IPI };
          if(!lwb_request_stream(&my_stream, 0)) {
          }
        }
      } else {
        /* keep the output buffer filled, generate a dummy packet */
        create_one_packet();
        #if DIMMER_USE_ACKS
        lwb_get_dimmer_acks(dimmer_app_acks_sender_id, dimmer_app_acks_first_byte);
        printf("Received ACKS: %u & %u\n", dimmer_app_acks_sender_id[0], dimmer_app_acks_sender_id[1]);
        #endif /* DIMMER_USE_ACKS */
      }

    }
    /* let the debug print process run */
    debug_print_poll();
#if USE_DIMMER
    /* Call Dimmer */
    dimmer_poll();
#endif
  }

  PROCESS_END();
}
