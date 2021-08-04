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
#if USE_DIMMER
#include "dimmer.h"
#include "dimmer-testbed.h"
#define ENERGEST_CONF_ON 1
#endif
#include "graz.h"
#include "my2c.h"
#include "isr_compat.h"

#if GRAZ == 0
  #error "This application should be used for Graz only!"
#endif


#define IS_GRAZ_SOURCE_NODE() ((node_id<=DIMMER_NUM_OF_NODES))
#define IS_GRAZ_SINK_NODE() ((node_id==6))
#define GRAZ_TIMEOUT 4
#define NUM_GRAZ_PKT 30


/*---------------------------------------------------------------------------*/

#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */


/* Graz experiment configuration */
config_t cfg __attribute__((section(".testbedConfigSection")));// ={{{1,{208,0,0,0,0,0,0,0},{216,0,0,0,0,0,0,0},4,0,0,0,500,30000}}};
static process_event_t FALLING_EDGE_EV;

// Store more than one Graz eval packet at any time
struct graz_packet {
  uint8_t buf[LWB_CONF_MAX_DATA_PKT_LEN];
  uint8_t acked;
  uint8_t timeout;
};

struct graz_application {
  struct graz_packet pkt_buffer[NUM_GRAZ_PKT];
  uint8_t dummy_buffer[LWB_CONF_MAX_DATA_PKT_LEN];
  uint8_t packet_position; // Position of the last packet added in memory
  uint16_t sender_id; // Used by coordinator to acknowledge the correct sender
  uint8_t new_packet_to_send;
  uint8_t next_ack_position_in_sched; // position in schedule header where next ack will be added
  uint8_t received_acks_sender_id[DIMMER_NUM_ACKS];
  uint8_t received_acks_first_byte[DIMMER_NUM_ACKS];
};

static struct graz_application app;



void generate_dummy_packet() {
  memset(app.dummy_buffer, 0, LWB_CONF_MAX_DATA_PKT_LEN - 3);
  app.dummy_buffer[0] = (uint8_t) (0x1F);
}

#if DIMMER_USE_ACKS
void read_acks() {
   uint8_t i, j;
  // read last ACKs
  lwb_get_dimmer_acks(app.received_acks_sender_id, app.received_acks_first_byte);
  for (i=0;i<DIMMER_NUM_ACKS;++i) {
    // check if some of our pkts were ACKed
    if (node_id == app.received_acks_sender_id[i]) {
      // ack the correct packet
      for (j=0;j<NUM_GRAZ_PKT;++j) {
        if (app.received_acks_first_byte[i] == app.pkt_buffer[j].buf[1]) {
          // ack the pkt
          if (!app.pkt_buffer[j].acked) {
            printf("[GRAZ,%u] Acked: 0x%x\n", lwb_get_last_round_id(), app.received_acks_first_byte[i]);
            app.pkt_buffer[j].acked = 1;
            break; // goto next ack
          }
        }
      }
    }
  }
  // decrease timeouts
  for (i=0;i<NUM_GRAZ_PKT;++i) {
    if (app.pkt_buffer[i].timeout>0)
    app.pkt_buffer[i].timeout--;
  }
}
#else
void read_acks() {}
#endif

#if DIMMER_USE_ACKS
void transmit_packet() {
  uint8_t j, pkt_to_send;
  // Have we generated a new packet since the last call?
  if (app.new_packet_to_send) {
    // Send this packet
    pkt_to_send = app.packet_position;
    app.new_packet_to_send = 0;
    app.pkt_buffer[pkt_to_send].timeout = GRAZ_TIMEOUT;
    printf("[GRAZ,%u] TX New 0x%x [%u]\n",lwb_get_last_round_id(), app.pkt_buffer[pkt_to_send].buf[1], lwb_get_send_buffer_state());
    #if USE_DIMMER
      lwb_send_pkt(0xffff, 1, app.pkt_buffer[pkt_to_send].buf, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN - DIMMER_HEADER_LEN);
    #else /* USE_DIMMER */
      lwb_send_pkt(0xffff, 1, app.pkt_buffer[pkt_to_send].buf, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN);
    #endif /* USE_DIMMER */
    return;
  } else {
    // Send the first non-acked packet in memory
    for (j=0;j<NUM_GRAZ_PKT;++j) {
      // sent the first non-acked pkt, never send twice the same pkt in a row, ack is delayed by one round
      if (!app.pkt_buffer[j].acked) {
        if (app.pkt_buffer[j].timeout==0) {
          pkt_to_send = j;
          app.pkt_buffer[j].timeout = GRAZ_TIMEOUT;
          printf("[GRAZ,%u] re-TX 0x%x [%u]\n",lwb_get_last_round_id(), app.pkt_buffer[pkt_to_send].buf[1], lwb_get_send_buffer_state());
          #if USE_DIMMER
            lwb_send_pkt(0xffff, 1, app.pkt_buffer[pkt_to_send].buf, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN - DIMMER_HEADER_LEN);
          #else /* USE_DIMMER */
            lwb_send_pkt(0xffff, 1, app.pkt_buffer[pkt_to_send].buf, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN);
          #endif /* USE_DIMMER */
          return;
        }
      }
    }
    // If we have reached this point, this means all packets were acked.
    // or that we must wait another round for ack
    if (lwb_get_send_buffer_state() == 0) {
      printf("[GRAZ,%u] .\n", lwb_get_last_round_id());
      generate_dummy_packet();
      #if USE_DIMMER
        lwb_send_pkt(0xffff, 1, app.dummy_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN - DIMMER_HEADER_LEN);
      #else /* USE_DIMMER */
        lwb_send_pkt(0xffff, 1, app.dummy_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN);
      #endif /* USE_DIMMER */
    }
  }
}
#else
void transmit_packet() {
  if (app.new_packet_to_send) {
      #if USE_DIMMER
        lwb_send_pkt(0xffff, 1, app.dummy_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN - DIMMER_HEADER_LEN);
      #else /* USE_DIMMER */
        lwb_send_pkt(0xffff, 1, app.dummy_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN);
      #endif /* USE_DIMMER */
      app.new_packet_to_send = 0;
      return;
  } else {
    if (lwb_get_send_buffer_state() == 0) {
      generate_dummy_packet();
      #if USE_DIMMER
        lwb_send_pkt(0xffff, 1, app.dummy_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN - DIMMER_HEADER_LEN);
      #else /* USE_DIMMER */
        lwb_send_pkt(0xffff, 1, app.dummy_buffer, LWB_CONF_MAX_DATA_PKT_LEN - LWB_HEADER_LEN);
      #endif /* USE_DIMMER */
    }
  }
}
#endif

/* print received packets to serial line - for eval */
void read_received_packets()
{
  uint16_t pkt_len;
  do {
    pkt_len = lwb_rcv_pkt(app.dummy_buffer, &app.sender_id, 0);
      if (IS_GRAZ_SINK_NODE() && app.dummy_buffer[0] == (uint8_t) (0xF1)) {
        // add to ACKS
        #if DIMMER_USE_ACKS
        printf("[GRAZ,%u] Acking %u:0x%x\n", lwb_get_last_round_id(), app.sender_id, app.dummy_buffer[1]);
        lwb_set_dimmer_ack(app.next_ack_position_in_sched, (uint8_t)app.sender_id, app.dummy_buffer[1]);
        app.next_ack_position_in_sched = (app.next_ack_position_in_sched+1) % DIMMER_NUM_ACKS;
        #endif /* DIMMER_USE_ACKS */
        rtimer_ext_clock_t lastTime = LWB_RTIMER_NOW();
	      while ((LWB_RTIMER_NOW() - lastTime) < 660);
        P2OUT |= BV(EVENT_PIN);
        my2c_start();
        my2c_write(0x50<<1);
        my2c_write(cfg.p[0].msg_offsetH);
        my2c_write(cfg.p[0].msg_offsetL);
        int i;
        for(i=0;i<cfg.p[0].msg_length;i++){
            my2c_write(app.dummy_buffer[i+1]);
        }
        my2c_stop();
        rtimer_ext_clock_t current = LWB_RTIMER_NOW();
	      while ((LWB_RTIMER_NOW() - current) < 200){};//165) {};// ~164 cycles (32kHz) are 5ms
        P2OUT &= ~BV(EVENT_PIN);
      }
  } while(pkt_len);
}

/*---------------------------------------------------------------------------*/
/* lwb_application_process: Application level code, run LWB */
/*---------------------------------------------------------------------------*/
PROCESS(lwb_application_process, "Dimmer Application Example");
#if GRAZ
  PROCESS(graz_i2c_process, "Graz I2C task");
  AUTOSTART_PROCESSES(&lwb_application_process,&graz_i2c_process);
#else
  AUTOSTART_PROCESSES(&lwb_application_process);
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lwb_application_process, ev, data)
{
  static uint8_t stream_state = 0;

  PROCESS_BEGIN();

  #if USE_DIMMER
  init_mapped_node_id();
  node_id = mapped_node_id;
  #endif
  cc2420_set_txpower(0xff);
  uint8_t i;
  for (i=0;i<NUM_GRAZ_PKT;++i) {
    app.pkt_buffer[i].acked=1;
  }

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

#if USE_DIMMER
  dimmer_init();
#endif
  lwb_start(0, &lwb_application_process, node_id == HOST_ID);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    //leds_off(LEDS_RED);
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
    //leds_on(LEDS_RED);
    if(HOST_ID == node_id) {
      read_received_packets();
      #if DIMMER_USE_ACKS
      lwb_get_dimmer_acks(app.received_acks_sender_id, app.received_acks_first_byte);
      #endif
    } else {
      /* we are a source node */
      //read_received_packets();
      // Only DIMMER_NUM_OF_NODES can start a flow
      if (IS_GRAZ_SOURCE_NODE()) {
        if(stream_state != LWB_STREAM_STATE_ACTIVE) {
          stream_state = lwb_stream_get_state(1);
          if(stream_state == LWB_STREAM_STATE_INACTIVE) {
            /* request a stream */
            lwb_stream_req_t my_stream = { node_id, 0, 1, SOURCE_IPI };
            if(!lwb_request_stream(&my_stream, 0)) {
            }
          }
        } else {
          // send newest packet, or any non-acked packet, or dummy message
          read_acks();
          if (lwb_get_send_buffer_state() == 0) {
            transmit_packet();
          }
        }
      } else {
        //printf("[GRAZ,%u] Not a source\n", lwb_get_last_round_id());
      }

    }
    /* let the debug print process run */
    debug_print_poll();
#if USE_DIMMER
    /* Call the deep-Q network */
    dimmer_poll();
#endif
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/

ISR(PORT2, __eeprom_isr){
      P2IFG=0;//&=~BV(EVENT_PIN);
      process_post(&graz_i2c_process,FALLING_EDGE_EV,NULL);
    }
  PROCESS_THREAD(graz_i2c_process, ev, data)
  {
    static int i=0;
    PROCESS_BEGIN();
      FALLING_EDGE_EV=process_alloc_event();
      //watchdog_stop();
      my2c_enable();
      my2c_stop();
      if (IS_GRAZ_SOURCE_NODE() && !IS_GRAZ_SINK_NODE()) {
        P2DIR&=~BV(EVENT_PIN);
        P2SEL&=~BV(EVENT_PIN);
        P2IES|= BV(EVENT_PIN);
        P2IFG&= ~BV(EVENT_PIN);
        P2IE |= BV(EVENT_PIN);
        clock_delay(10000);
        while( (P2IN & BIT6) != 0);
        while(1) {
        PROCESS_WAIT_EVENT_UNTIL(ev==FALLING_EDGE_EV);
        // point to the correct memory
        #if DIMMER_USE_ACKS
        app.packet_position = (app.packet_position+1)%NUM_GRAZ_PKT;
        if (!app.pkt_buffer[app.packet_position].acked)
          printf("[GRAZ,%u] Skip 0x%x\n", lwb_get_last_round_id(), app.pkt_buffer[app.packet_position].buf[1]);
        app.pkt_buffer[app.packet_position].acked=0;
        #endif
        my2c_start();
        my2c_write(0x50<<1);
        my2c_write(cfg.p[0].msg_offsetH);
        my2c_write(cfg.p[0].msg_offsetL);
        my2c_stop(); 
        clock_delay(100);
        my2c_start();
        my2c_write((0x50<<1)|1);
        #if DIMMER_USE_ACKS
        memset(app.pkt_buffer[app.packet_position].buf, 0, LWB_CONF_MAX_DATA_PKT_LEN - 3);
        app.pkt_buffer[app.packet_position].buf[0] = 0xF1;
        #else
        memset(app.dummy_buffer, 0, LWB_CONF_MAX_DATA_PKT_LEN - 3);
        app.dummy_buffer[0] = 0xF1;
        #endif
        for(i=0;i<(cfg.p[0].msg_length);i++){
          if(i==((cfg.p[0].msg_length)-1)) {
            #if DIMMER_USE_ACKS
            app.pkt_buffer[app.packet_position].buf[i+1]=my2c_read(0);
            #else
            app.dummy_buffer[i+1]=my2c_read(0);
            #endif
          } else {
            #if DIMMER_USE_ACKS
            app.pkt_buffer[app.packet_position].buf[i+1]=my2c_read(1);
            #else
            app.dummy_buffer[i+1]=my2c_read(1);
            #endif
          }
        }
        my2c_stop();
        printf("[GRAZ,%u] New pkt: 0x%x\n", lwb_get_last_round_id(), app.pkt_buffer[app.packet_position].buf[1]);
        app.new_packet_to_send = 1;
      }
    } else if(IS_GRAZ_SINK_NODE()) {
      P2SEL &= ~BV(EVENT_PIN);
      P2DIR |=  BV(EVENT_PIN);
      P2OUT &= ~BV(EVENT_PIN);
      while(1) {
      PROCESS_WAIT_EVENT_UNTIL(ev==FALLING_EDGE_EV);
      }
    }
    else {
    while(1) {
      PROCESS_WAIT_EVENT_UNTIL(ev==FALLING_EDGE_EV);
      }
    }

    PROCESS_END();
  }