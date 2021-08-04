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

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_


/* Use Dimmer? */
#define USE_DIMMER                                      1
/* Enable additional output used to collect traces, and allow to control the retransmission parameter from the serial line */
#define DIMMER_USE_SERIAL_LINE_FOR_NN                   1
/* Use the pre-traiend embedded DQN as central adaptivity control */
#define DIMMER_USE_EMBEDDED_NN                          0
/* Use the handcrafted PID controller as central adaptivity control */
#define DIMMER_EVAL_USE_HANDCRAFTED_CONTROLLER          0
/* Initial retransmission value to use */
#define DIMMER_INITIAL_RETRANSMISSION_PARAM             3
/* Dimmer requires ENERGEST */
#if USE_DIMMER
#define ENERGEST_CONF_ON                1
/* Without this, Glossy does not work with Energest */
#define FAST_ENERGEST_MODE              1
#endif


/* Define which testbed is used (for node-id mapping) */
#define COOJA                             0
#define FLOCKLAB                          0
#define INDRIYA2                          0
#define KIEL                              1
#define GRAZ                              0


// Testbed defines
#if KIEL
  #define TESTBED                         12
  #define NUM_NODES                       18
  #define RF_CHANNEL                      26
  #define TX_POWER_LEVEL                  0xFF
  #define HOST_ID                         8
  #define SOURCE_IPI                      4    /* seconds */
  #define DIMMER_NUM_OF_NODES          NUM_NODES
  #define LWB_CONF_MAX_DATA_SLOTS         NUM_NODES
  #define LWB_CONF_IN_BUFFER_SIZE         20
  #define DIMMER_USE_ACKS                 1
  #define DIMMER_NUM_ACKS                 4
#elif FLOCKLAB
  #define TESTBED                         3
  #define NUM_NODES                       10
  #define RF_CHANNEL                      26
  #define TX_POWER_LEVEL                  0xFF
  #define HOST_ID                         8
  #define SOURCE_IPI                      4    /* seconds */
  #define DIMMER_NUM_OF_NODES             NUM_NODES
  #define LWB_CONF_MAX_DATA_SLOTS         NUM_NODES
  #define DIMMER_USE_ACKS                 0
#elif COOJA
  #define TESTBED                         0
  #define NUM_NODES                       18
  #define RF_CHANNEL                      26
  #define TX_POWER_LEVEL                  0xFF
  #define HOST_ID                         1
  #define SOURCE_IPI                      4    /* seconds */
  #define DIMMER_NUM_OF_NODES             NUM_NODES
  #define LWB_CONF_IN_BUFFER_SIZE         20
  #define DIMMER_USE_ACKS                 1
  #define DIMMER_NUM_ACKS                 2
#elif GRAZ
  #define TESTBED                         13
  #define NUM_NODES                       55
  #define RF_CHANNEL                      26
  #define TX_POWER_LEVEL                  0xFF
  #define HOST_ID                         6
  #define SOURCE_IPI                      4    /* seconds */
  #define DIMMER_NUM_OF_NODES             15 // needs a smaller number, otherwise we run out of memory!
  #define LWB_CONF_MAX_DATA_SLOTS         DIMMER_NUM_OF_NODES
  #define LWB_CONF_IN_BUFFER_SIZE         20
  #define DIMMER_USE_ACKS                 1
  #define DIMMER_NUM_ACKS                 2
#endif




/*
 * LWB-application specifics, can be used to override LWB parameters
 */

/* For evaluation purposes */
#define LWB_CONF_SCHED_STREAM_REMOVAL_THRES  256 // don't remove streams

#if FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"
  /* set the highest antenna gain if the program runs on FlockLAB */
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #ifdef PLATFORM_SKY
    #define GLOSSY_TX_PIN               FLOCKLAB_LED2
    #define GLOSSY_RX_PIN               FLOCKLAB_LED3
  #else  /* PLATFORM_SKY */
    #define RF_GDO2_PIN                 FLOCKLAB_INT1
  #endif /* PLATFORM_SKY */
  #define LWB_CONF_TASK_ACT_PIN         FLOCKLAB_INT2
  #define DEBUG_PRINT_CONF_TASK_ACT_PIN FLOCKLAB_INT2
  #define APP_TASK_ACT_PIN              FLOCKLAB_INT2
#endif /* FLOCKLAB */

/* platform specific config */
#ifdef PLATFORM_SKY
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            ADC0
    #define GLOSSY_TX_PIN               ADC1
    #define GLOSSY_RX_PIN               ADC2
  #endif /* FLOCKLAB */
  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL
  /* stats */
  #define DCSTAT_CONF_ON                1
#endif

/* LWB configuration */
#define LWB_SCHED_MIN_ENERGY             /* use the minimum energy scheduler */
#define LWB_CONF_SCHED_PERIOD_IDLE      5        /* define the period length */
#define LWB_CONF_SCHED_PERIOD_MIN       2
#define LWB_CONF_SCHED_PERIOD_MAX       15
#define LWB_CONF_OUT_BUFFER_SIZE        4
#define LWB_CONF_MAX_PKT_LEN            30
#define LWB_CONF_MAX_DATA_PKT_LEN       30
#define LWB_CONF_SCHED_T_NO_REQ         5
#define LWB_CONF_MAX_HOPS               7 // not used?
#define LWB_CONF_T_SCHED                (LWB_RTIMER_SECOND / 40)     /* 25ms */
#define LWB_CONF_T_DATA                 (LWB_RTIMER_SECOND / 50)     /* 20ms */
#define LWB_CONF_T_GUARD                (LWB_RTIMER_SECOND / 2000)  /* 0.5ms */
#define LWB_CONF_T_GUARD_1              (LWB_RTIMER_SECOND / 2000)  /* 0.5ms */
#define LWB_CONF_T_GUARD_2              (LWB_RTIMER_SECOND / 1000)    /* 1ms */
#define LWB_CONF_T_GUARD_3              (LWB_RTIMER_SECOND / 500)     /* 2ms */
#define LWB_CONF_T_GAP                  (LWB_RTIMER_SECOND / 250)     /* 4ms */
#define LWB_CONF_T_CONT                 (LWB_RTIMER_SECOND / 125)     /* 8ms */
#define LWB_CONF_TX_CNT_SCHED           3
#define LWB_CONF_TX_CNT_DATA            3
#define LWB_CONF_T_SCHED2_START         (LWB_RTIMER_SECOND)            /* 1s */
#define LWB_CONF_T_SILENT               0            /* disable this feature */
/* Debug */
#define DEBUG_PRINT_CONF_STACK_GUARD    (SRAM_START + SRAM_SIZE - 0x0200)


#endif /* PROJECT_CONF_H_ */
