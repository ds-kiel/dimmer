import re
import datetime
from enum import Enum
import numpy as np

#### DISPATCHER ####
## Dispatcher server constants
DISPATCHER_ADDRESS = "localhost"
DISPATCHER_PORT = 25565
DISPATCHER_BUFFER_SIZE = 2048
## Dispatcher requests
RESET_REQUEST = "Reset"
ACTION_REQUEST = "Action"
STATE_REQUEST = "State"
INCORRECT_ACTION_INFO = "incorrect" # agent requested an incorrect action
## Dispatcher timing constants
# In seconds
STATE_ACQUISITION_WAIT_IN_SEC = 1.
DESYNC_TEST_WAIT_IN_SEC = 1.
BOOTSTRAPPING_WAIT_IN_SEC = 5.
SOCKET_TIMEOUT_IN_SEC = 5.
# In number of Glossy floods
STATE_ACQUISITION_WAIT_IN_FLOODS = 8
DESYNC_TEST_WAIT_IN_FLOODS = 8
BOOTSTRAPPING_WAIT_IN_FLOODS = 16
# Timeout counters
MULTIAGENT_MAXIMUM_TIMEOUT_FOR_AGENT_REQUEST = 2

#### GLOSSY ####
## Regular Expression to read output from Glossy node
# res:N:latency(radio_on_time):reliability_since_last_reset:pckts_received_since_last_reset
GLOSSY_REGEX = re.compile(r'res\:(\d+)\:(\d+)\:(\d+.\d+)\:(\d+.\d+):(\d+)')
LWB_REGEX = re.compile(r'res\:(\d+)\:(\d+)\:(\d+)\:(\d+):\d+/\d+\:(\d+)')
REGEX = LWB_REGEX
USE_OLD_GLOSSY = False
REGEX_FIELD_N_POSITION = 0
REGEX_FIELD_LAST_RECEIVED_PACKET_ID_POSITION = 1
REGEX_FIELD_LATENCY_POSITION = 2
REGEX_FIELD_RELIABILITY_SINCE_LAST_RESET_POSITION = 3
REGEX_FIELD_NUMBER_OF_PACKETS_RECEIVED_SINCE_BEGINNING_OF_TIME_POSITION = 4
DISPATCHER_COMMAND_SEPARATOR = b'\n\n\n'
## Node constants
# Maximum latency: 20 ms - 20000 us
# Changing MAX_LATENCY means relogging new traces
MAX_LATENCY = 20
DEFAULT_N = 3
MIN_INDIVIDUAL_N = 0
MIN_COMMON_N = 1
# Maximum number of retransmission per node during one flood
# Changing MAX_N means relogging new traces
MAX_N = 8
NORMALIZATION_MAX_N = 9
# Glossy Control Layer constants
DEFAULT_N_COMMAND = 101 # Recognized by GlossAI control layer as "reset statistics and reset to N = default N (=5)"
KEEP_N_COMMAND = 102 # Recognized by GlossAI control layer as "reset statistics and keep current N"

TRACE_PATH = "../../../../dimmer/RL/traces"

TESTBED = {}
# Nodes 12 && 13 are removed from testbed and considered as jammers
# todo put corrept IPs
TESTBED["kiel"] = {
    'number_of_nodes': 18,
    'addresses': ['192.168.87.226', '192.168.87.227', '192.168.87.228', '192.168.87.229', '192.168.87.230',
                  '192.168.87.231', '192.168.87.232', '192.168.87.233', '192.168.87.234', '192.168.87.235',
                  '192.168.87.239', '192.168.87.240', '192.168.87.241', '192.168.87.242', '192.168.87.243',
                  '192.168.87.244', '192.168.87.245', '192.168.87.246'],
    'ports': [50000]*18,
    'traces_location': 'kiel.npy',
    'model_location': '../models/kiel_jammers_out' # leave _nodeID.pkl OUT
}
TESTBED["kiel_eval"] = {
    'number_of_nodes': 18,
    'addresses': ['192.168.87.226', '192.168.87.227', '192.168.87.228', '192.168.87.229', '192.168.87.230',
                  '192.168.87.231', '192.168.87.232', '192.168.87.233', '192.168.87.234', '192.168.87.235',
                  '192.168.87.239', '192.168.87.240', '192.168.87.241', '192.168.87.242', '192.168.87.243',
                  '192.168.87.244', '192.168.87.245', '192.168.87.246'],
    'ports': [50000]*18,
    'traces_location': 'kiel_eval_nointerf.npy',
    'model_location': '../models/kiel_eval' # leave _nodeID.pkl OUT
}
TESTBED["old-flocklab"] = {
    'number_of_nodes': 27,
    'addresses': ['whymper.ee.ethz.ch']*27,
    'ports': [50101, 50102, 50104, 50108, 50115, 50133, 50103, 50132, 50131, 50128, 50122, 50106, 50116,
                50118, 50127, 50123, 50124, 50110, 50126, 50120, 50119, 50117, 50113, 50125, 50114, 50111, 50107],
    'traces_location': 'flocklab.npy',
    'model_location': '../models/flocklab' # leave _nodeID.pkl OUT
}
TESTBED["cooja"] = {
    'number_of_nodes': 10,
    'addresses': ['127.0.0.1']*10,
    'ports': list(range(60001,60000+10+1)),
    'traces_location': 'cooja.npy',
    'model_location': '../models/cooja/cooja' # leave _nodeID.pkl OUT
}


#### GYM ENVIRONMENT ####
# Environment constants
ENV_MAX_NUMBER_OF_STEPS = 10
class Action(Enum):
    DECREMENT = 0
    NOTHING = 1
    INCREMENT = 2

#### UTILS ####
# possible debug level
LOG_INFO = 0
LOG_WARNING = 1
LOG_ERROR = 2
LOG_SUCCESS = 3
LOG_DEBUG = 4
LOG_COLOR_INFO = "\033[0m"
LOG_COLOR_WARNING = "\x1b[0;33;33m"
LOG_COLOR_ERROR = "\x1b[1;31;31m"
LOG_COLOR_SUCCESS = "\x1b[1;32;32m"
LOG_COLOR_DEBUG = "\x1b[0;36;36m" 
LOG_NORMAL_COLOR_MODE = "\033[0m"

# Minimum level of debug displayed
LOG_MINIMUM_LEVEL = LOG_INFO
# utility functions
def log(prefix, data, debug_level, debug_color = ""):
    if debug_level >= LOG_MINIMUM_LEVEL:
        print('{}[{} {}] {}{}'.format(debug_color, datetime.datetime.now(), prefix, data, LOG_NORMAL_COLOR_MODE))

def log_debug(data):
    log("DEBUG", data, LOG_DEBUG, LOG_COLOR_DEBUG)

def log_info(data):
    log("INFO", data, LOG_INFO, LOG_COLOR_INFO)

def log_warning(data):
    log("WARNING", data, LOG_WARNING, LOG_COLOR_WARNING)

def log_error(data):
    log("ERROR", data, LOG_ERROR, LOG_COLOR_ERROR)

def log_success(data):
    log("SUCCESS", data, LOG_SUCCESS, LOG_COLOR_SUCCESS)


def get_gym_name(is_multiagent, testbed):
    return "Glossy{}N-{}-v0".format("Individual" if is_multiagent else "Common",
                                                    testbed)

def get_model_path(testbed, node_id = -1):
    if node_id < 0:
        return TESTBED[testbed]['model_location'] + '.pkl'
    return TESTBED[testbed]['model_location'] + '_' +  str(node_id) + '.pkl'

def normalized_N_to_N_value(normalized_N):
    return int((normalized_N+1)*(NORMALIZATION_MAX_N/2))

def normalized_N_to_N_values(normalized_Ns):
    return list(map(int, (np.array(normalized_Ns)+1)*(NORMALIZATION_MAX_N/2)))

def get_full_desynchronized_state(nb_nodes, N_values):
    log_warning("Introducing desync values in the state")
    return [1]*nb_nodes + [-1]*nb_nodes + N_values

