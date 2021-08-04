import time
import socket
import select
import re
import threading
import io
import random
import pickle
import numpy as np
import gym_dimmer.envs.utils.glossai_utils as glossai_utils
from gym_dimmer.envs.utils.TraceHandler import TraceHandler
from gym_dimmer.envs.utils.TestbedHandler import TestbedHandler


class Dispatcher(threading.Thread):
###############################################################################################################
#                                            Class functions """
###############################################################################################################
    def __init__(self,
                 requested_testbed,
                 use_traces = False,
                 one_agent_per_node = False,
                 use_randomized_order = False, # deprecated
                 use_randomized_value_at_beginning_of_episode = False, # deprecated
                 **kwargs):
        threading.Thread.__init__(self)
        # Dispatcher has been requested to shut down 
        self.should_stop_main_loop = False
        # save inputs
        self.use_randomized_order = use_randomized_order
        self.use_randomized_value_at_beginning_of_episode = use_randomized_value_at_beginning_of_episode
        self.is_one_agent_per_node = one_agent_per_node
        self.use_traces = use_traces
        # testbed name, a string
        self.testbed_name = None
        # network representation, a TraceHandler or TestbedHandler
        self.network = None
        # Dispatcher has detected that the action is incorrect? i.e., the N value requested is not permitted
        self.incorrect_action = False
        # Counter to avoid desync
        self.desync_counter = 0
        # Dispatcher's interface for RL agents
        self.buffer_size = glossai_utils.DISPATCHER_BUFFER_SIZE
        self.dispatcher = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dispatcher.settimeout(60.)
        self.dispatcher.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.dispatcher.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)

        # Catch possible exceptions when reading traces or trying to get testbed's config
        try:
            self.testbed_name = requested_testbed
            if use_traces:
                self.network = TraceHandler(requested_testbed)
            else:
                self.network = TestbedHandler(requested_testbed)
            self.network.start()
        except Exception as e:
            glossai_utils.log_error(e)
        
        # Successfully selected traces or correct testbed
        glossai_utils.log_success("Dispatcher: {} selected: {}".format("traces" if use_traces else "testbed",
                                                                       requested_testbed))

        # Dispatcher.py: Check that trace/testbed selection has been carried
        assert self.testbed_name is not None and self.network is not None

        # Dispatcher's global view of N parameters of all nodes in the network
        self.N_value_for_nodes = [0]*self.network.nb_nodes # Actual N used by the node

        # RL agents are represented as socket clients
        self.RL_agents = None
        if one_agent_per_node:
            # if we have one RL algorithm per node in the network
            self.RL_agents = [None] * self.network.nb_nodes
            # allows switchiong between nodes if execution is one at a time
            self.agent_requested_reset = [False]*self.network.nb_nodes
            # used tp retrieve agent in list
            self.current_RL_agent_relative_id_in_order = -1
            # used to allow suffled order
            self.RL_agents_serving_order = np.array(range(len(self.RL_agents)))

        try:
            self.dispatcher.setblocking(glossai_utils.SOCKET_TIMEOUT_IN_SEC)
            self.dispatcher.bind((glossai_utils.DISPATCHER_ADDRESS, glossai_utils.DISPATCHER_PORT))
            self.dispatcher.listen()
            self.dispatcher.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.dispatcher.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
            glossai_utils.log_success("Dispatcher has started, available at {}:{}".format(glossai_utils.DISPATCHER_ADDRESS, glossai_utils.DISPATCHER_PORT))
        except Exception as e:
            glossai_utils.log_error("Couldn't start the Dispatcher!")
            glossai_utils.log_error("{}".format(e))
            self.dispatcher.close()

    def stop(self):
        # Stop TestbedHandler
        self.network.stop()
        self.should_stop_main_loop = True


###############################################################################################################
#                                            Helper functions
###############################################################################################################

    """Check the validity of N parameter"""
    def _verify_N_parameter(self, N):
        try:
            min_N = glossai_utils.MIN_INDIVIDUAL_N if self.is_one_agent_per_node else glossai_utils.MIN_COMMON_N 
            if N < min_N:
                self.incorrect_action = True
                N = min_N
            elif N > glossai_utils.MAX_N:
                N = glossai_utils.MAX_N
                self.incorrect_action = True
            return N
        except Exception as e:
            print(e)
            self.should_stop_main_loop = True
            return glossai_utils.DEFAULT_N_COMMAND

    def _get_full_desynchronized_state(self, state):
        glossai_utils.log_error("DEPRECATED FUNCTION: _get_full_desynchronized_state")
        for i in range(self.network.nb_nodes):
            state[i] = 1.
            state[self.network.nb_nodes+i] = -1.
        return state

    def _modify_state_to_include_desync(self, state):
        glossai_utils.log_error("DEPRECATED FUNCTION: _modify_state_to_include_desync")
        for i in range(self.network.nb_nodes, 2*self.network.nb_nodes):
            if state[i] < 0.:
                state[i] = -1.
                state[i-self.network.nb_nodes] = 1.
        return state

    def _is_glossy_desynchronized(self, lowest_ratio_packet_received):
        #glossai_utils.log_error("DEPRECATED FUNCTION: _is_glossy_desynchronized")
        if (lowest_ratio_packet_received < 0.5):
            glossai_utils.log_debug("Lowest received ratio: {}".format(lowest_ratio_packet_received))
        return  lowest_ratio_packet_received < 0.4

    def _check_possible_desync(self):
        #glossai_utils.log_warning("_check_possible_desync")
        self.network.wait(glossai_utils.DESYNC_TEST_WAIT_IN_FLOODS)
        current_state, _, ratio_packets_received = self.network.read_state()
        ratio_packets_received = np.array(ratio_packets_received)
        return self._is_glossy_desynchronized(np.min(ratio_packets_received))

    """Security to avoid Glossy desynchronization"""
    def _avoid_desync(self):
        if self._check_possible_desync():
            glossai_utils.log_warning("Reset Glossy and reboostrap!")
            self.network.send_action([glossai_utils.DEFAULT_N_COMMAND]*self.network.nb_nodes)
            self.network.wait(glossai_utils.BOOTSTRAPPING_WAIT_IN_FLOODS) # allows bootstrapping and sync

###############################################################################################################
#                                           RL agent Communication functions
###############################################################################################################

    def _send_to_agent(self, agent, data):
        agent.send(pickle.dumps(data) + glossai_utils.DISPATCHER_COMMAND_SEPARATOR)

    def _accept_RL_agent_connection(self):
        try:
            (connection_socket, (client_address,client_port)) = self.dispatcher.accept()
            connection_socket.settimeout(glossai_utils.SOCKET_TIMEOUT_IN_SEC)
            connection_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            glossai_utils.log_success("New RL agent ({}:{}) connecting to the system".format(client_address, client_port))
            return connection_socket
        except Exception as e:
            glossai_utils.log_error("Couldn't accept RL agent as client")
            glossai_utils.log_error("{}".format(e))
            return None

    def _get_RL_agent_absolute_ID(self):
        if self.is_one_agent_per_node:
            return self.RL_agents_serving_order[self.current_RL_agent_relative_id_in_order]
        else:
            return 0

    def _get_next_RL_agent(self):
        # Reset counter to detect agent timeout
        self.RL_agent_timeout_counter = 0
        if self.is_one_agent_per_node:
            # get next RL agent
            self.current_RL_agent_relative_id_in_order +=1
            if self.current_RL_agent_relative_id_in_order >= len(self.RL_agents_serving_order):
                self.current_RL_agent_relative_id_in_order = 0
                # We can treat agents from 0..nb or use a shuffled order
                if self.use_randomized_order:
                    np.random.shuffle(self.RL_agents_serving_order)
            return self.RL_agents[self._get_RL_agent_absolute_ID()]
        else:
            # only one RL agent in system
            return self.RL_agents

    def _get_RL_agent_requests(self, agent):
        try:
            # if one agent per node and node has requested reset last time, force a reset request
            if self.is_one_agent_per_node:
                if self.agent_requested_reset[self._get_RL_agent_absolute_ID()]:
                    self.agent_requested_reset[self._get_RL_agent_absolute_ID()] = False
                    return [pickle.dumps({"request":glossai_utils.RESET_REQUEST})]
            # Read from socket
            return agent.recv(self.buffer_size).split(glossai_utils.DISPATCHER_COMMAND_SEPARATOR)
        except Exception as e:
            self.should_stop_main_loop = True
            glossai_utils.log_error("Could not receive request from agent {}: {}".format(agent,e))


    def _is_RL_agent_timeout(self):   
        return self.RL_agent_timeout_counter >= glossai_utils.MULTIAGENT_MAXIMUM_TIMEOUT_FOR_AGENT_REQUEST

    def _execute_beginning_of_episode_computation(self):
        # Use a random N command value at the beginning of the episode?
        if self.use_randomized_value_at_beginning_of_episode:
            commands_for_glossy = [glossai_utils.KEEP_N_COMMAND]*self.network.nb_nodes
            commands_for_glossy[self._get_RL_agent_absolute_ID()] = np.random.randint(low=0, high=5)
            self.network.send_action(commands_for_glossy)
###############################################################################################################
#                                           Request & Behaviour functions
###############################################################################################################

    """Execute the logic when a STATE request is received"""
    def _execute_state_request(self, agent, agentID = 0, should_check_desync = True):
        self.network.wait(glossai_utils.STATE_ACQUISITION_WAIT_IN_FLOODS)
        self.incorrect_action = False # todo remove
        state, current_n, ratio_packets_received = self.network.read_state(self.incorrect_action)
        self.incorrect_action = False
        # Save the lowest reliability during this observation
        self.lowest_ratio_received_packets = np.min(ratio_packets_received)

        # check we're not desync
        should_reset = False
        if should_check_desync:
            if self._is_glossy_desynchronized(self.lowest_ratio_received_packets):
                # reliability is low, check for a longer period of time to see if we are really desynchronizing
                if self._check_possible_desync():
                    self.desync_counter += 1
                    state = self._modify_state_to_include_desync(state)
            # if we believe we are desync, reset node N parameter to default and force next agent
            if self.desync_counter > 1: # at least two iterations were almost in desync
                glossai_utils.log_warning("Node {} caused a desync, reset command to default value".format(agentID))
                should_reset = True
                commands_for_glossy = [glossai_utils.KEEP_N_COMMAND]*self.network.nb_nodes
                commands_for_glossy[agentID] = glossai_utils.DEFAULT_N_COMMAND
                self.network.send_action(commands_for_glossy)
                state = self._get_full_desynchronized_state(state)
                self.desync_counter = 0

        # send state to agent
        response = {"state": state, "should_reset": should_reset, "this_agent_N": current_n[agentID]}
        self._send_to_agent(agent, response)

    """Execute the logic when a RESET request is received"""
    def _execute_reset_request(self, agent, agentID = 0):
        # Agent-env request a new episode some time ago, we need to read the current state
        self.network.wait(glossai_utils.STATE_ACQUISITION_WAIT_IN_FLOODS)
        # if we are using traces, we need to select a new subset of traces for the next episode
        if self.use_traces:
            self.network.request_reset()

        state, current_n, _ = self.network.read_state()
        # In a one env, all nodes share the same n
        response = {"state": state, "this_agent_N": current_n[agentID]}
        self._send_to_agent(agent, response)
        return True

    """Execute the logic when a ACTION request is received, single-env case"""
    def _execute_action_request(self, requested_n, agent, agentID=0):
        # todo is the action incorrect?
        if self.incorrect_action:
            self.incorrect_action = True
        verified_n = self._verify_N_parameter(requested_n)
        #print("Dispatcher", requested_n, verified_n)
        if self.is_one_agent_per_node:
            commands_for_glossy = [glossai_utils.KEEP_N_COMMAND]*self.network.nb_nodes
            commands_for_glossy[self._get_RL_agent_absolute_ID()] = verified_n
        else:
            commands_for_glossy = [verified_n]*self.network.nb_nodes
        self.network.send_action(commands_for_glossy)

    def _execute_end_of_episode_computation(self):
        self._avoid_desync()

    def _execute_request(self, current_RL_agent, agentID, request, done_serving_this_RL_agent):
        if request["request"] == glossai_utils.STATE_REQUEST:
            #print("state")
            self._execute_state_request(current_RL_agent, agentID, should_check_desync = False)
        elif request["request"] == glossai_utils.RESET_REQUEST:
            #print("reset")
            done_serving_this_RL_agent |= self._execute_reset_request(current_RL_agent, agentID)
        elif request["request"] == glossai_utils.ACTION_REQUEST:
            #print("action")
            self._execute_action_request(request["value"], current_RL_agent, agentID)
        else:
            glossai_utils.log_warning("Non recognized request: {}, ({})".format(request["request"],
                                                                                request))
        return done_serving_this_RL_agent


###############################################################################################################
#                                                   Main loop
###############################################################################################################

    def run(self):
        # Wait for RL agents connection
        if self.is_one_agent_per_node:
            for i in range(0,self.network.nb_nodes):
                self.RL_agents[i] = self._accept_RL_agent_connection()
                glossai_utils.log_success("RL Agent mapped to node {}".format(i))
        else:
            self.RL_agents = self._accept_RL_agent_connection()
        glossai_utils.log_info("Dispatcher: Starting service loop.")

        # WHILE TRUE
        while not self.should_stop_main_loop:
            done_serving_this_RL_agent = False

            # get next RL agent to serve
            current_RL_agent = self._get_next_RL_agent()
            # Check if we need to update the N value of this node to a random value for learning
            self._execute_beginning_of_episode_computation()

            # Serve this RL agent until end of episode or if agent has timeout
            while not done_serving_this_RL_agent and not self._is_RL_agent_timeout() and not self.should_stop_main_loop:
                # Read all requests received by this RL agent
                #t0 = time.time()
                packed_requests_list = self._get_RL_agent_requests(current_RL_agent)
                if packed_requests_list:
                    for packed_request in packed_requests_list:
                        t1 = time.time()
                        # Avoid empty elements or timeout agent
                        if packed_request ==b'':
                            self.RL_agent_timeout_counter +=1
                            continue
                        self.RL_agent_timeout_counter = 0
                        # Unpack request
                        request = pickle.loads(packed_request)
                        # Switch request type
                        done_serving_this_RL_agent |= self._execute_request(current_RL_agent,
                                                                            self._get_RL_agent_absolute_ID(),
                                                                            request,
                                                                            done_serving_this_RL_agent)
                #    glossai_utils.log_debug("request {} executed in {} ns".format(request["request"], (time.time()-t1)*1000000))
                #glossai_utils.log_debug("All requests in {} ns".format((time.time()-t0)*1000000))
            # Execute end of episode check - desync
            # TODO
            #self._execute_end_of_episode_computation()
