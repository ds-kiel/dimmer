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
#include <stdio.h>
#include "sys/node-id.h"
#include "dimmer-testbed.h"

// Maps node_id from node-id.h into an ID between [0, MAX_NODE_COUNT_TESTBED]
unsigned short mapped_node_id = 0;

#if TESTBED > 0 /* 0 is cooja or no testbed */
  #if TESTBED == TESTBED_KIEL
	#define MAX_NODE_COUNT_TESTBED 19 // highest possible ID
	const uint16_t mapping[MAX_NODE_COUNT_TESTBED] = {0,1,2,3,4,5,6,7,8,9,\
													  10,14,15,16,17,18,19,20,21};

  #elif TESTBED == TESTBED_FLOCKLAB
    #define MAX_NODE_COUNT_TESTBED 10 // highest possible ID
	const uint16_t mapping[MAX_NODE_COUNT_TESTBED] = {0,1,2,4,8,15,33,3,32,31,}
    
  #elif TESTBED == TESTBED_GRAZ
    #define MAX_NODE_COUNT_TESTBED 49 // highest possible ID
	const uint16_t mapping[MAX_NODE_COUNT_TESTBED] = {0,212,119,207,220,222,202,\
													  200,201,203,204,205,206,\
													  208,209,210,211,213,\
													  214,215,216,217,218,219,\
													  221,223,224,225,226,227,\
													  100,101,102,103,104,105,\
													  106,107,108,109,110,111,112,\
													  113,114,115,116,117,118};
  #endif /* TESTBED_xxx */
#endif /* TESTBED */

#if TESTBED > 0
	//execute mapping of node id to mapped id
	void init_mapped_node_id(){
		unsigned int i;
		//lookup id
		for( i = 0; i < MAX_NODE_COUNT_TESTBED; i++ ){
			if( node_id == mapping[i] ){
				mapped_node_id = i;
				return;
			}
		}
		// no id found in mapping? keep current node_id
		mapped_node_id = node_id;
	}
#else
void init_mapped_node_id(){
	mapped_node_id = node_id;
}
#endif