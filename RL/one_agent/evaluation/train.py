import sys, threading, multiprocessing, time, os

import gym
import gym_dimmer
import gym_dimmer.envs.utils.glossai_utils as glossai_utils
import gym_dimmer.envs.utils.dimmer_nn
from gym_dimmer.envs.utils.Dispatcher import Dispatcher

from baselines import deepq
import numpy as np

from baselines.common.models import register



def train(testbed,
          instance_name,
          k_worst_nodes,
          history_size,
          reward_K,
          instance_id):

    instance = f"{instance_name}_{testbed}_k_{k_worst_nodes}_history_{history_size}_reward_constant_{reward_K}_instance_{instance_id}"


    dispatcher = Dispatcher(testbed,
                            use_traces = True,
                            one_agent_per_node = False,
                            use_randomized_order = False,
                            use_randomized_value_at_beginning_of_episode = False)
    time.sleep(2)
    dispatcher.daemon = True
    dispatcher.start()

    kwargs = {"testbed":testbed, "k_worst_nodes_len":k_worst_nodes, "history_len":history_size, "reward_K":reward_K}
    env = gym.make(f"CentralizedControl-v0", **kwargs)
    act, _ = deepq.learn(
        env,
        network='dimmer_deepq_network',
        lr=5e-4,
        total_timesteps=200000,
        exploration_fraction=0.9,
        exploration_final_eps=0.05,
        print_freq=100,
        discount_factor=0.5,
        dueling=False,
        checkpoint_freq=5000,
    )
    glossai_utils.log_warning("Saving model")
    act.save(f"../models/evaluation/{instance}.pkl")
    glossai_utils.log_success("Saved!")
    dispatcher.stop()



if __name__ == '__main__':
    # params
    testbed =          "kiel"
    instance_name =    "dqn_eval"
    k_worst_nodes =    10
    history_size =     2
    reward_K =        30
    # check all existing instance IDs, try to get a new one
    i = 0
    found = True
    while found:
        found = os.path.exists(f"../models/evaluation/{instance_name}_{testbed}_k_{k_worst_nodes}_history_{history_size}_reward_constant_{reward_K}_instance_{i}.pkl") 
        i+=1
    i -=1
    instance_id = i
    glossai_utils.log_success(f"Training model named ../models/evaluation/{instance_name}_{testbed}_k_{k_worst_nodes}_history_{history_size}_reward_constant_{reward_K}_instance_{i}.pkl")
    train(testbed, instance_name, k_worst_nodes, history_size, reward_K, instance_id)
