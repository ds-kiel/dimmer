import os
import numpy as np
import gym_dimmer.envs.utils.glossai_utils as glossai_utils

class TraceHandler:

    def __init__(self, testbed_name):
        # load number of nodes from config
        self.nb_nodes = glossai_utils.TESTBED[testbed_name]['number_of_nodes']
        # Load traces path from config
        traces_path = os.path.join(glossai_utils.TRACE_PATH, glossai_utils.TESTBED[testbed_name]['traces_location'])
        # Load traces
        self.measurements = np.load(traces_path, allow_pickle=True)

        self.number_of_traces = self.measurements.shape[0]
        self.max_N_in_traces = self.measurements.shape[1]
        glossai_utils.MAX_N = self.max_N_in_traces-1
        self.max_samples_in_traces = self.measurements.shape[2]
        self.current_trace_file_index = np.random.randint(0, self.number_of_traces)
        self.current_state_index = np.random.randint(0, self.max_samples_in_traces - glossai_utils.ENV_MAX_NUMBER_OF_STEPS -1)
        self.N = 0

    def start(self):
        pass

    def stop(self):
        pass

    def read_state(self, incorrect_action = False):
        if incorrect_action:
            incorrect_action = False
            raise NotImplementedError
            #return glossai_utils.get_full_desynchronized_state(), np.array([self.N]*self.nb_nodes)
        if (self.current_state_index >= self.measurements.shape[2]):
            self.current_state_index = 0
        state = self.measurements[self.current_trace_file_index, self.N, self.current_state_index]
        self.current_state_index +=1
        return state, glossai_utils.normalized_N_to_N_values(state[2*self.nb_nodes:]), np.array(state[self.nb_nodes:2*self.nb_nodes])

    def send_action(self, N_array):
        # Traces only contain identical N for all nodes
        # TraceHandler cannot serve different N
        assert len(np.unique(N_array)) < 3 # (Correct value at position 0, 102 at all other positions)
        if N_array[0] < 100:
            self.N = N_array[0]

    def request_reset(self):
        self.current_state_index = np.random.randint(0, self.max_samples_in_traces - glossai_utils.ENV_MAX_NUMBER_OF_STEPS -1)
        self.current_trace_file_index = np.random.randint(0, self.number_of_traces)

    def wait(self, time_in_s):
        pass