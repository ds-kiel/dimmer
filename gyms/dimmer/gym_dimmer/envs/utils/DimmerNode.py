import numpy as np
import gym_dimmer.envs.utils.glossai_utils as glossai_utils

class DimmerNode:

    def __init__(self):
        # curr_step_results_x contain the result for this node during one step
        # a step might contain more than glossy flood
        self._curr_step_results_latency = []
        self._curr_step_results_reception = []
        self._curr_n = 0
        self.nb_packets_since_last_reset = 0

    def read_and_parse_line(self, line):
        res = self.extract_data_using_regex(line, glossai_utils.REGEX)
        # if the line is correct
        if res:
            if glossai_utils.USE_OLD_GLOSSY:
                self.normalize_and_append_data(res[glossai_utils.REGEX_FIELD_LATENCY_POSITION],
                                           res[glossai_utils.REGEX_FIELD_RELIABILITY_SINCE_LAST_RESET_POSITION],
                                           res[glossai_utils.REGEX_FIELD_N_POSITION],
                                           res[glossai_utils.REGEX_FIELD_NUMBER_OF_PACKETS_RECEIVED_SINCE_BEGINNING_OF_TIME_POSITION])
            else:
                self.normalize_and_append_data(res[glossai_utils.REGEX_FIELD_LATENCY_POSITION]/1000.0,
                                            res[glossai_utils.REGEX_FIELD_RELIABILITY_SINCE_LAST_RESET_POSITION]/10000.0,
                                            res[glossai_utils.REGEX_FIELD_N_POSITION],
                                            res[glossai_utils.REGEX_FIELD_NUMBER_OF_PACKETS_RECEIVED_SINCE_BEGINNING_OF_TIME_POSITION])

    def reset_data(self):
        #glossai_utils.log_debug("DimmerNode: {} floods wasted by data reset".format(len(self._curr_step_results_latency)))
        self._curr_step_results_latency = []
        self._curr_step_results_reception = []
        self.nb_packets_since_last_reset = 0

    def get_data(self):
        latency = None
        reliability = None
        nb_packets_received = 0.
        n = None
        latency = self._curr_step_results_latency
        reliability = self._curr_step_results_reception
        n = self._curr_n
        nb_packets_received = self.nb_packets_since_last_reset

        #glossai_utils.log_debug("DimmerNode: {} floods present when data read".format(len(self._curr_step_results_latency)))
        return latency, reliability, n, nb_packets_received

    def extract_data_using_regex(self, line, regex = glossai_utils.REGEX):
        res = None
        # parse line
        res = regex.findall(line)
        # if the line could be parsed, transform strings into float
        if len(res) > 0:
            res = res[0] # at most one element treated
            res = list(map(float, res))
        return res

    def normalize_and_append_data(self, data_latency, data_reception, data_n, data_nb_packets_received_since_last_reset):
        self._curr_step_results_latency.append(self.normalize_latency(data_latency))
        self._curr_step_results_reception.append(self.normalize_reception(data_reception))
        self._curr_n = self.normalize_N(data_n)
        self.nb_packets_since_last_reset = data_nb_packets_received_since_last_reset

    def normalize_latency(self, data_latency):
        # TelosB implementation gives us a latency in ms.
        # We clip the maximum latency we recognize as glossai_utils.MAX_LATENCY
        # We normalize in [-1,1]
        # normalized_latency = min(1, latency / (MAX_LATENCY/2) -1)
        if data_latency > glossai_utils.MAX_LATENCY:
            return 1.
        return (2*data_latency/glossai_utils.MAX_LATENCY - 1)

    def normalize_reception(self, data_reception):
        # TelosB implementation gives us a reliability in [0,1].
        # IMPORTANT! We trim the result and consider only reliability above 50%!!!
        data_reception = max(data_reception, 0.5)-0.5
        # we are now between [0;0.5]
        # We normalize in [-1,1]
        return (4*data_reception - 1)

    def normalize_N(self, data_n):
        # TelosB implementation gives us the number of retransmission as an integer.
        # We normalize in [-1,1]
        # normalized_n = n / (MAX_N/2) -1
        return (2*data_n/glossai_utils.NORMALIZATION_MAX_N -1)

    def get_number_of_floods(self):
        return len(self._curr_step_results_latency)


