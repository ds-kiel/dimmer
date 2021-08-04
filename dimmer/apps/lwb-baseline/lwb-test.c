/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Reto Da Forno
 */
 
/**
 * @brief Low-Power Wireless Bus Test Application
 */

#include "contiki.h"
#include "lwb.h"
#include "debug-print.h"
#include "dc-stat.h"
#include "gpio.h"
#include "node-id.h"
#include "glossy.h"
#include "cc2420.h"

/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */




static uint8_t pkt_buffer[LWB_CONF_MAX_DATA_PKT_LEN];
static uint16_t pkt_id;



void create_one_packet()
{
  if ( lwb_get_send_buffer_state() == 0) {
    memset(pkt_buffer, 0, LWB_CONF_MAX_DATA_PKT_LEN - 3);
    pkt_buffer[0] = (uint8_t) (node_id >> 8);
    pkt_buffer[1] = (uint8_t) (node_id & 0xff);
    pkt_buffer[2] = (uint8_t) (pkt_id >> 8);
    pkt_buffer[3] = (uint8_t) (pkt_id & 0xff);
    uint8_t success = lwb_send_pkt(0xffff, 1, pkt_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN);
    if (success) {
      ++pkt_id;
      printf("pkt:%u:%u\n", (uint16_t) ((pkt_buffer[0] << 8) | pkt_buffer[1]), (uint16_t) ((pkt_buffer[2] << 8) | pkt_buffer[3]));
    }
  }
}

void read_received_packets()
{
  uint16_t pkt_len;
  do {
    pkt_len = lwb_rcv_pkt(pkt_buffer, 0, 0);
    printf("pkt:%u:%u\n", (uint16_t) ((pkt_buffer[0] << 8) | pkt_buffer[1]), (uint16_t) ((pkt_buffer[2] << 8) | pkt_buffer[3]));
  } while(pkt_len);
}



/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data)
{
  static uint8_t stream_state = 0;

  PROCESS_BEGIN();


  cc2420_set_txpower(0xfb);

  PIN_SEL_I(5, 5);      /* SMCLK output on LED2 (for debugging only) */
  /* init pins */
#ifdef GLOSSY_START_PIN
  PIN_CLR(GLOSSY_START_PIN);
  PIN_CFG_OUT(GLOSSY_START_PIN);
#endif /* GLOSSY_START_PIN */
#ifdef GLOSSY_TX_PIN
  PIN_CLR(GLOSSY_TX_PIN);
  PIN_CFG_OUT(GLOSSY_TX_PIN);
#endif /* GLOSSY_TX_PIN */
#ifdef GLOSSY_RX_PIN
  PIN_CLR(GLOSSY_RX_PIN);
  PIN_CFG_OUT(GLOSSY_RX_PIN);
#endif /* GLOSSY_RX_PIN */
#ifdef GLOSSY_DEBUG_PIN
  PIN_CLR(GLOSSY_DEBUG_PIN);
  PIN_CFG_OUT(GLOSSY_DEBUG_PIN);
#endif /* GLOSSY_DEBUG_PIN */
#ifdef DEBUG_PRINT_CONF_TASK_ACT_PIN
  PIN_CLR(DEBUG_PRINT_CONF_TASK_ACT_PIN);
  PIN_CFG_OUT(DEBUG_PRINT_CONF_TASK_ACT_PIN);
#endif /* DEBUG_PRINT_CONF_TASK_ACT_PIN */
#ifdef APP_TASK_ACT_PIN
  PIN_CLR(APP_TASK_ACT_PIN);
  PIN_CFG_OUT(APP_TASK_ACT_PIN);
#endif /* APP_TASK_ACT_PIN */

  /* start the LWB thread */
  lwb_start(0, &app_process, node_id == HOST_ID);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */

    if(HOST_ID == node_id) {
      /* we are the host: read the received data packets */
      read_received_packets();
      DEBUG_PRINT_INFO("time: %llu, rcvd: %u, FSR: %u, PER: %u, CPU DC: %u, RF DC: %u",
                       lwb_get_timestamp(),
                       0,
                       glossy_get_fsr(),
                       glossy_get_per(),
                       DCSTAT_CPU_DC,
                       DCSTAT_RF_DC);

    } else {
      /* we are a source node */
      if(stream_state != LWB_STREAM_STATE_ACTIVE) {
        stream_state = lwb_stream_get_state(1);
        if(stream_state == LWB_STREAM_STATE_INACTIVE) {
          /* request a stream */
          lwb_stream_req_t my_stream = { node_id, 0, 1, SOURCE_IPI };
          if(!lwb_request_stream(&my_stream, 0)) {
            DEBUG_PRINT_ERROR("stream request failed");
          }
        }
      } else {
        create_one_packet();
        read_received_packets();
      }
      DEBUG_PRINT_INFO("time: %llu, FSR: %u, PER: %u, CPU DC: %u, RF DC: %u",
                       lwb_get_timestamp(),
                       glossy_get_fsr(),
                       glossy_get_per(),
                       DCSTAT_CPU_DC,
                       DCSTAT_RF_DC);
    }
    
    /* let the debug print process run */
    debug_print_poll();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
