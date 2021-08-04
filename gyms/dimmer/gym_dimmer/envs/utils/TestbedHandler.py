import threading
import time
import socket
from select import select as select
import numpy as np
import gym_dimmer.envs.utils.glossai_utils as glossai_utils
from gym_dimmer.envs.utils.DimmerNode import DimmerNode

class TestbedHandler(threading.Thread):

    def __init__(self, testbed_name):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        # Dispatcher has been requested to shut down 
        self.should_stop_main_loop = False
        # load number of nodes from config
        self.nb_nodes = glossai_utils.TESTBED[testbed_name]['number_of_nodes']
        self.nodes = [DimmerNode() for i in range(self.nb_nodes)]
        self.node_sockets = []
        # Load the nodes' IP address and port from config
        self.ips = glossai_utils.TESTBED[testbed_name]['addresses']
        self.ports = glossai_utils.TESTBED[testbed_name]['ports']
        # Dictionary mapping from ip:port to node id
        self.node_address_to_node_id = dict()
        # Connect to the nodes
        self._connect_to_nodes()
        assert len(self.node_sockets) == self.nb_nodes # We should have as many nodes as in the config
        # N value for each node
        self.N = [0]*self.nb_nodes # Actual N used by the node

    def stop(self):
        self.should_stop_main_loop = True

    def run(self):

        while not self.should_stop_main_loop:
            time.sleep(0.1)
            # Did we receive anything?
            (read_events, _, _) = select(self.node_sockets, [], [])
            for node in read_events:
                ip, prt = node.getpeername()
                with self.lock:
                    self._read_input_from_node(node, self.node_address_to_node_id["{}:{}".format(ip, prt)])

    def read_state(self, incorrect_action = False):
        try:
            self.lock.acquire()
            self.latency = [-1.]*self.nb_nodes
            self.reliability = [-1.]*self.nb_nodes
            self.nb_packets_received_perceived_by_nodes = [0]*self.nb_nodes
            for i in range(self.nb_nodes):
                self._read_state_from_one_node(self.nodes[i], i)
        except Exception as e:
            glossai_utils.log_error("TestbedHandler could not read state from nodes: {}".format(e))
        finally:
            self.lock.release()

        state = self.latency + self.reliability + list(self.N)
        
        if incorrect_action:
            incorrect_action = False
            state = glossai_utils.get_full_desynchronized_state(self.nb_nodes, self.N)

        self.nb_packets_received_perceived_by_nodes = np.array(self.nb_packets_received_perceived_by_nodes)
        ratio_packets_received = self.nb_packets_received_perceived_by_nodes / np.max(self.nb_packets_received_perceived_by_nodes)

        return state, glossai_utils.normalized_N_to_N_values(np.array(self.N)), ratio_packets_received

    def send_action(self, N_array):
        assert len(self.node_sockets) == len(N_array)

        for i in range(0, len(N_array)): # Cannot send negative N
            if N_array[i] < 0:
                N_array[i] = 0

        # We acquire the lock
        
        with self.lock:
            threads = []
            for i in range(0, len(self.node_sockets)):
                threads.append(threading.Thread(target=self.send,
                                                args=(self.node_sockets[i],
                                                      '{}\n'.format(N_array[i]),)))
                threads[-1].start()
            # wait until completion
            continue_waiting = True
            while continue_waiting:
                continue_waiting = False
                for t in threads:
                    if t.isAlive():
                        continue_waiting = True
                        break
            # we reset the statistics of each node
            for node in self.nodes:
                node.reset_data()

    def wait(self, nb_of_floods_to_wait_for):
        # we want that at least 80% of the nodes reported their values 3 times in a row
        nb_of_respondants = 0
        total_time_waited_in_sec = 0

        while(total_time_waited_in_sec < 10):
            nb_of_respondants = 0
            for node in self.nodes:
                if node.get_number_of_floods() > 8:
                    nb_of_respondants += 1
            #if nb_of_respondants >= minimal_supermajority*self.nb_nodes:
            #    consecutive_successful_trials += 1
            #else:
                # we don't want to go back to zero to avoid being stuck?
            #    pass

            #if consecutive_successful_trials >= required_successful_trials:
            #    return
            if nb_of_respondants >= self.nb_nodes:
                return
            else:
                total_time_waited_in_sec += 0.1
                time.sleep(0.1)


    def _wait(self, time_in_s):
        time.sleep(time_in_s)

#############################################################################

    def _read_state_from_one_node(self, node, nodeID):
        latency_measures_from_node, reliability_measures_from_node, n_from_node, nb_packets_received = node.get_data()
        # Might have no data, reason unknown
        if latency_measures_from_node is None:
            self.wait(glossai_utils.STATE_ACQUISITION_WAIT_IN_FLOODS)
            latency_measures_from_node, reliability_measures_from_node, n_from_node, nb_packets_received = node.get_data()

        avg_latency_node = 1.
        if latency_measures_from_node is not None:
            if len(latency_measures_from_node) > 0:
                avg_latency_node = np.nanmean(np.array(latency_measures_from_node))

        avg_reliability_node = -1. # no measure == no packets received
        if reliability_measures_from_node is not None:
            if len(reliability_measures_from_node) > 0:
                # reliability measure is an average by itself, take last measurement
                avg_reliability_node = reliability_measures_from_node[-1]
 
        if nb_packets_received is None:
            nb_packets_received = 0

        #if avg_reliability_node == -1: # No packets received?
        #    avg_latency_node = 1 # Then latency is maxmimal
        self.latency[nodeID] = avg_latency_node
        self.reliability[nodeID] = avg_reliability_node
        self.N[nodeID] = n_from_node
        self.nb_packets_received_perceived_by_nodes[nodeID] = nb_packets_received

    def _connect_to_nodes(self):
        for i in range(self.nb_nodes):
            try:
                self.node_sockets.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
                self.node_sockets[-1].connect((self.ips[i], self.ports[i]))
                self.node_address_to_node_id["{}:{}".format(self.ips[i], self.ports[i])] = i
                # changes made for flocklab to work
                #self.node_address_to_node_id["{}".format(self.ports[i])] = i
            except Exception as e:
                glossai_utils.log_error('Cannot connect to node {}:{}: {}'.format(self.ips[i], self.ports[i], e))
                self.node_sockets.append([])

    def send(self, socket, data):
        success = True
        try:
            socket.send(data.encode())
        except SocketError as e:
            glossai_utils.log_error("Node {} (Send): {}".format(self.socket_port,e))
            success = False
        return success

    def _read_input_from_node(self, socket, node_id):
        try:
            data = socket.recv(glossai_utils.DISPATCHER_BUFFER_SIZE).decode('utf-8')
            for line in data.splitlines():
                self.nodes[node_id].read_and_parse_line(line)
        except Exception as e:
            glossai_utils.log_error("TestbedHandler._read_input_from_node: {}".format(e))