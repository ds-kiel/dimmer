import time
import numpy as np
from gym_dimmer.envs.utils.Dispatcher import Dispatcher
from gym_dimmer.envs.utils.GymToDispatcherInterface import GymToDispatcherInterface

def main():

    testbed = "kiel"
    NB_NODES = 18
    NB_OF_ITERATIONS = 500
    MIN_N = 1
    MAX_N = 8

    dispatcher = Dispatcher(testbed,
                            use_traces = False,
                            one_agent_per_node = False,
                            use_randomized_order = False,
                            use_randomized_value_at_beginning_of_episode = False)
    time.sleep(2)
    #dispatcher.daemon = True
    dispatcher.start()


    dispatcherIntf = GymToDispatcherInterface(testbed)
    dispatcherIntf.connect()

    dispatcherIntf.request_reset()

    states = []
    for i in range(0,MAX_N+1):
        states.append([])

    possibilities = list(range(MIN_N,MAX_N+1))

    for iteration in range(0,NB_OF_ITERATIONS):
        print("Iteration {}/{}".format(iteration, NB_OF_ITERATIONS+1))

        #dispatcherIntf.send_action(3) # reset to "normal", default state
        #time.sleep(5.)

        requested_Ns = np.random.permutation(possibilities)
        for n in requested_Ns:
            print("sending {}".format(n))
            dispatcherIntf.send_action(n) # request a new state
            received_state, n_in_state, _ = dispatcherIntf.request_state()
            print(received_state)
            print(n_in_state)
            states[n].append(np.array(received_state)) # read state and save it at the correct N parameter

    # INJECT N = 0
    for i in range(NB_OF_ITERATIONS):
        states[0].append(np.array([1.]*NB_NODES + [-1.]*NB_NODES + [-1.]*NB_NODES))
        states[1].append(np.array([1.]*NB_NODES + [-1.]*NB_NODES + [-1.]*NB_NODES))
        states[2].append(np.array([1.]*NB_NODES + [-1.]*NB_NODES + [-1.]*NB_NODES))
        states[4].append(np.array([1.]*NB_NODES + [-1.]*NB_NODES + [-1.]*NB_NODES))
        states[6].append(np.array([1.]*NB_NODES + [-1.]*NB_NODES + [-1.]*NB_NODES))
        states[7].append(np.array([1.]*NB_NODES + [-1.]*NB_NODES + [-1.]*NB_NODES))


    states = np.array(states)
    states_shape = states.shape
    states = np.reshape(states, (1,) + states_shape) # reshape into [1, nb of N, number of iterations, 2*number of nodes +1]
    np.save("../traces/{}_sensys_evaluation_trace3".format(testbed), np.array(states))

    print("Traces saved!")
    dispatcher.stop()


if __name__ == '__main__':
    main()
