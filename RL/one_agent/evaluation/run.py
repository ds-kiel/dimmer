import sys
import time
import os
import numpy as np
from baselines import deepq

import gym
import gym_dimmer
import gym_dimmer.envs.utils.dimmer_nn
from gym_dimmer.envs.utils.Dispatcher import Dispatcher
import gym_dimmer.envs.utils.glossai_utils as glossai_utils

from baselines.common.models import register




def run(testbed, instance_name, k_worst_nodes, history_size, reward_K, instance_id):

    PRINT_ADVANCED_STATISTICS = True
    NUMBER_OF_EPISODES = 10
    instance = f"{instance_name}_{testbed}_k_{k_worst_nodes}_history_{history_size}_reward_constant_{reward_K}_instance_{instance_id}"
    testbed += "_eval"

    dispatcher = Dispatcher(testbed,
                            use_traces = True,
                            one_agent_per_node = False,
                            use_randomized_order = False,
                            use_randomized_value_at_beginning_of_episode = False)
    time.sleep(2)
    dispatcher.start()

    kwargs = {"testbed":testbed, "k_worst_nodes_len":k_worst_nodes, "history_len":history_size}
    env = gym.make(f"CentralizedControl-v0", **kwargs)
    act, dbg = deepq.learn(env,
                                   network='dimmer_deepq_network',
                                   total_timesteps=0,
                                   load_path=f"../models/evaluation/{instance}.pkl",
                                   dueling=False)

    for i in range(NUMBER_OF_EPISODES):
        obs, done = env.reset(), False
        episode_rew = 0
        if PRINT_ADVANCED_STATISTICS:
            s = []
            a = []
            n = []
        while not done:
            action = int(act(obs)[0])
            print(dbg["q_values"](obs))
            if PRINT_ADVANCED_STATISTICS:
                s.append(int((np.nanmean(obs[k_worst_nodes:2*k_worst_nodes])+1)*50))
                n.append(np.argmax(obs[2*k_worst_nodes:2*k_worst_nodes+9]))
                a.append(action)
            obs, rew, done, _ = env.step(action)
            episode_rew += rew
        if PRINT_ADVANCED_STATISTICS:
            glossai_utils.log_info("+".join("----" for x in list(range(len(n)+1))))
            glossai_utils.log_info("|".join(" {}  ".format(i) for i in n))
            glossai_utils.log_info("+".join("----" for x in list(range(len(n)+1))))
            glossai_utils.log_info("|".join(" {} ".format(int(i)) for i in s))
            glossai_utils.log_info("+".join("----" for x in list(range(len(n)+1))))
            glossai_utils.log_info("|".join(" +  " if i == 2 else (" -  " if i == 0 else "    ") for i in a))
            glossai_utils.log_info("+".join("----" for x in list(range(len(n)+1))))
        glossai_utils.log_info("Episode {} reward: {}".format(i, episode_rew))
    glossai_utils.log_success("End of execution!")
    dispatcher.stop()

if __name__ == '__main__':
    # params
    testbed =          "kiel"
    instance_name =    "dqn_eval"
    k_worst_nodes =    10
    history_size =     2
    reward_K = 50
    # check all existing instance IDs, try to get a new one
    instance_id = 0
    run(testbed, instance_name, k_worst_nodes, history_size, reward_K, instance_id)
