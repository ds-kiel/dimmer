import sys
import time
import numpy as np
from baselines import deepq
import gym
import gym_dimmer
import gym_dimmer.envs.utils.dimmer_nn
from gym_dimmer.envs.utils.Dispatcher import Dispatcher
import gym_dimmer.envs.utils.glossai_utils as glossai_utils
from baselines.common.models import register

def main(input_size):

    TESTBED = "kiel"
    LEARNING_INSTANCE_ID = f"dqn_eval_kiel_k_10_history_0_reward_constant_30_instance_0"

    dispatcher = Dispatcher(TESTBED,
                            use_traces = True,
                            one_agent_per_node = False,
                            use_randomized_order = False,
                            use_randomized_value_at_beginning_of_episode = False)
    dispatcher.start()

    kwargs = {"testbed":testbed, "k_worst_nodes_len":k_worst_nodes, "history_len":history_size}
    env = gym.make(f"CentralizedControl-v0", **kwargs)
    act, dbg = deepq.learn(env,
                      network='dimmer_deepq_network',
                      total_timesteps=0,
                      load_path="../models/{}/{}/{}.pkl".format(TESTBED, LEARNING_INSTANCE_ID, TESTBED),
                      dueling=False)
    n = 3
    s = 100
    i=0
    obs, _ = env.reset(), False
    while i<1000:
        old_n = n
        old_s = s
        action = int(act(obs)[0])
        #print(dbg['q_values'](obs)[0], action)
        obs, _, _, _ = env.step(action)
        #print(obs)
        s = (np.mean(obs[input_size:2*input_size])+1)*25+50
        n = np.argmax(obs[2*input_size:2*input_size+9])
        print(" {}% || N: {} -> {} || {}%".format(round(old_s,3),
                                                  old_n,
                                                  n,
                                                  round(s,3)))
        i+=1

    dispatcher.stop()

if __name__ == '__main__':
    main(int(sys.argv[1]))
