import sys
import socket
import pickle
from socket import error as SocketError
import gym_dimmer.envs.utils.glossai_utils as glossai_utils


class GymToDispatcherInterface():

    def __init__(self, testbed, **kwargs):
        self.nb_nodes = 0
        try:
            self.nb_nodes = glossai_utils.TESTBED[testbed]['number_of_nodes']
        except Exception as e:
            print(e)
        assert self.nb_nodes > 0

        self.buffer_size = glossai_utils.DISPATCHER_BUFFER_SIZE


    # Private interface to Dispatcher Server

    def _connect_to_dispatcher(self):
        self.dispatcher = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.dispatcher.setblocking(True)
            self.dispatcher.connect((glossai_utils.DISPATCHER_ADDRESS, glossai_utils.DISPATCHER_PORT))
            self.dispatcher.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.dispatcher.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        except Exception as e:
            print("Object GymToDispatcherInterface could not connect to Dispatcher at {}:{}, error: {}".format(glossai_utils.DISPATCHER_ADDRESS, glossai_utils.DISPATCHER_PORT, e))
            #sys.exit(1)

    def _read(self):
        lines = []
        try:
            requests = self.dispatcher.recv(self.buffer_size).split(glossai_utils.DISPATCHER_COMMAND_SEPARATOR)
            # no request received
            if requests == [b'']:
                return None
            for req in requests:
                if req ==b'':
                    continue
                lines.append(pickle.loads(req))
        except Exception as e:
            glossai_utils.log_error("Object GymToDispatcherInterface could not read from Dispatcher at {}:{}, error: {}".format(glossai_utils.DISPATCHER_ADDRESS, glossai_utils.DISPATCHER_PORT, e))
            sys.exit(1)
        #glossai_utils.log_warning("Received from dispatcher")
        return lines

    def _send(self, data):
        #glossai_utils.log_warning("Sending to dispatcher")
        try:
            self.dispatcher.send(pickle.dumps(data) + glossai_utils.DISPATCHER_COMMAND_SEPARATOR)
        except Exception as e:
            glossai_utils.log_error("Object GymToDispatcherInterface could not send to Dispatcher at {}:{}, error: {}".format(glossai_utils.DISPATCHER_ADDRESS, glossai_utils.DISPATCHER_PORT, e))
            sys.exit(1)

    # Public interface for Gym environment

    def connect(self):
        self._connect_to_dispatcher()

    def request_state(self):
        request = {"request":glossai_utils.STATE_REQUEST}
        self._send(request)
        data_from_dispatcher = []
        while len(data_from_dispatcher) < 1:
            data_from_dispatcher = self._read()
        if data_from_dispatcher is None:
            sys.exit(1)
        state = data_from_dispatcher[0]["state"] # keep only the first response
        is_episode_finished = data_from_dispatcher[0]["should_reset"]
        local_N = data_from_dispatcher[0]["this_agent_N"]
        return state, local_N, is_episode_finished

    def request_reset(self):
        request = {"request":glossai_utils.RESET_REQUEST}
        self._send(request)
        data_from_dispatcher = []
        while len(data_from_dispatcher) < 1:
            data_from_dispatcher = self._read()
        if data_from_dispatcher is None:
            sys.exit(1)
        state = data_from_dispatcher[0]["state"] # keep only the first response
        return state

    def send_action(self, N):
        request = {"request":glossai_utils.ACTION_REQUEST, "value":int(N)}
        self._send(request)