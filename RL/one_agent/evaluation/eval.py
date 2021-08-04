import sys
import time
import os
import numpy as np
from baselines import deepq
import tensorflow as tf
import matplotlib.pyplot as plt

import gym
import gym_dimmer
import gym_dimmer.envs.utils.dimmer_nn
from gym_dimmer.envs.utils.Dispatcher import Dispatcher
import gym_dimmer.envs.utils.glossai_utils as glossai_utils

from baselines.common.models import register

PATH_DIMMER = "/Users/valentin/research/dimmer"
PATH_DIMMERDATA = "/Users/valentin/research/dimmer-data"
PATH_MODELS = os.path.join(PATH_DIMMER, "agents/one_agent/models/evaluation")
PATH_SAVED_RESULTS = os.path.join(PATH_DIMMERDATA, "eval_dqn_data")

def normalized_radio_to_radio(rad):
    return (np.array(rad)+1)*10 # in ms

def normalized_reliability_to_reliability(rel):
    return (np.array(rel)+1)*25+50 # in %, we can't get mroe granularity below 50%

def run(testbed,
        instance_name,
        k_worst_nodes,
        history_size,
        instance_id,
        reward_K,
        num_episodes,
        episode_len):

    # Find the correct trained DQN
    instance = f"{instance_name}_{testbed}_k_{k_worst_nodes}_history_{history_size}_reward_constant_{reward_K}_instance_{instance_id}"
    model_path = os.path.join(PATH_MODELS, instance)

    # We are doing evaluation, be sure to use the correct traces!
    testbed += "_eval"

    # Create Dispatcher object
    dispatcher = Dispatcher(testbed,
                            use_traces = True,
                            one_agent_per_node = False,
                            use_randomized_order = False,
                            use_randomized_value_at_beginning_of_episode = False)
    time.sleep(0.1)
    dispatcher.start()

    # Pass arguments to gym and create env
    kwargs = {"testbed":testbed, "k_worst_nodes_len":k_worst_nodes, "history_len":history_size, "reward_K":reward_K}
    env = gym.make(f"CentralizedControl-v0", **kwargs)

    # Load DQN
    tf.compat.v1.reset_default_graph()
    with tf.compat.v1.Session().as_default() as sess:
        act, _ = deepq.learn(env,
                                    network='dimmer_deepq_network',
                                    total_timesteps=0,
                                    load_path=f"{model_path}.pkl",
                                    dueling=False)


        reliability_per_ep = []
        radio_per_ep = []
        n_per_ep = []
        for _ in range(num_episodes):
            reliability_per_step = []
            radio_per_step = []
            n_per_step = []
            obs, done = env.reset(), False
            for _ in range(episode_len):
                action = int(act(obs)[0])
                obs, rew, done, _ = env.step(action)
                # combining per-node results
                reliability_per_step.append(np.min(obs[k_worst_nodes:2*k_worst_nodes]))
                radio_per_step.append(np.mean(obs[:k_worst_nodes]))
                n_per_step.append(np.argmax(obs[2*k_worst_nodes:2*k_worst_nodes+9]))
            # combining per-step results
            reliability_per_ep.append(np.mean(reliability_per_step))
            radio_per_ep.append(np.mean(radio_per_step))
            n_per_ep.append(np.mean(n_per_step))
        dispatcher.stop()

    return reliability_per_ep, radio_per_ep, n_per_ep

def eval_k_worst_nodes():
    # Model params
    testbed =          "kiel"
    instance_name =    "dqn_eval"
    k_worst_nodes =    [1,5,10,15,18]
    instance_id = [0,1,2]
    history_size =     2
    reward_K = 30
    # Eval params
    num_episodes = 100
    episode_len = 30

    reliabilities = np.zeros((len(k_worst_nodes),
                              len(instance_id),
                              num_episodes))
    radioontimes = np.zeros((len(k_worst_nodes),
                             len(instance_id),
                             num_episodes))
    retransmissions = np.zeros((len(k_worst_nodes),
                             len(instance_id),
                             num_episodes))
    k_id = 0
    for k in k_worst_nodes:
        i_id = 0
        for i in instance_id:
            rel,rad, n = run(testbed,
                             instance_name,
                             k,
                             history_size,
                             i,
                             reward_K,
                             num_episodes,
                             episode_len)
            reliabilities[k_id,i_id] = normalized_reliability_to_reliability(rel)
            radioontimes[k_id,i_id] = normalized_radio_to_radio(rad)
            retransmissions[k_id,i_id] = n
            i_id+=1
        k_id+=1

    # we reshape to put all episodes from all instance IDs as an axis
    reliabilities = reliabilities.reshape((len(k_worst_nodes),-1))
    radioontimes = radioontimes.reshape((len(k_worst_nodes),-1))
    retransmissions = retransmissions.reshape((len(k_worst_nodes),-1))
    # we save our data
    np.save(os.path.join(PATH_SAVED_RESULTS, "k_worst_nodes_reliabilities"),
            reliabilities)
    np.save(os.path.join(PATH_SAVED_RESULTS, "k_worst_nodes_radioontimes"),
            radioontimes)
    np.save(os.path.join(PATH_SAVED_RESULTS, "k_worst_nodes_retransmissions"),
            retransmissions)

def eval_history_size():
    # Model params
    testbed =          "kiel"
    instance_name =    "dqn_eval"
    k_worst_nodes =    10
    instance_id = [0,1,2]
    history_size =     [0,1,2,3,4,5]
    reward_K = 30
    # Eval params
    num_episodes = 1000
    episode_len = 2

    reliabilities = np.zeros((len(history_size),
                              len(instance_id),
                              num_episodes))
    radioontimes = np.zeros((len(history_size),
                             len(instance_id),
                             num_episodes))
    retransmissions = np.zeros((len(history_size),
                             len(instance_id),
                             num_episodes))
    h_id = 0
    for h in history_size:
        i_id = 0
        for i in instance_id:
            rel,rad, n = run(testbed,
                             instance_name,
                             k_worst_nodes,
                             h,
                             i,
                             reward_K,
                             num_episodes,
                             episode_len)
            reliabilities[h_id,i_id] = normalized_reliability_to_reliability(rel)
            radioontimes[h_id,i_id] = normalized_radio_to_radio(rad)
            retransmissions[h_id,i_id] = n
            i_id+=1
        h_id+=1

    # we reshape to put all episodes from all instance IDs as an axis
    reliabilities = reliabilities.reshape((len(history_size),-1))
    radioontimes = radioontimes.reshape((len(history_size),-1))
    retransmissions = retransmissions.reshape((len(history_size),-1))

    # we select the best model
    # temporary avg to find the best model
    # re_ = np.nanmean(reliabilities,axis=2).reshape((len(history_size),len(instance_id)))
    # ra_ = np.nanmean(radioontimes,axis=2).reshape((len(history_size),len(instance_id)))
    # rn_ = np.nanmean(retransmissions,axis=2).reshape((len(history_size),len(instance_id)))
    # # we will store the results from the best model here
    # rel_ = np.zeros((len(history_size),
    #                  num_episodes))
    # rad_ = np.zeros((len(history_size),
    #                  num_episodes))
    # ret_ = np.zeros((len(history_size),
    #                  num_episodes))
    # models = np.zeros((len(history_size),),dtype=np.int32)
    # for i in range(len(history_size)):
    #     models[i] = int(np.argmax(re_[i]))
    #     print(f"Best model for reward[K={i}] is instance {models[i]}")
    #     rel_[i] = reliabilities[i,models[i]]
    #     rad_[i] = radioontimes[i,models[i]]
    #     ret_[i] = retransmissions[i,models[i]]

    # we save our data
    np.save(os.path.join(PATH_SAVED_RESULTS, "history_reliabilities2"),
            reliabilities)
    np.save(os.path.join(PATH_SAVED_RESULTS, "history_radioontimes2"),
            radioontimes)
    np.save(os.path.join(PATH_SAVED_RESULTS, "history_retransmissions2"),
            retransmissions)


def compare():
    # Model params
    testbed =          "kiel"
    instance_name =    "dqn_eval"
    k_worst_nodes =    10
    instance_id = [0,1,2]
    history_size =     2
    reward_K = 30
    # Eval params
    num_episodes = 100
    episode_len = 100

    reliabilities = np.zeros((1,
                              len(instance_id),
                              num_episodes))
    radioontimes = np.zeros((1,
                             len(instance_id),
                             num_episodes))
    retransmissions = np.zeros((1,
                             len(instance_id),
                             num_episodes))
    i_id = 0
    for i in instance_id:
        rel,rad, n = run(testbed,
                         instance_name,
                         k_worst_nodes,
                         history_size,
                         i,
                         reward_K,
                         num_episodes,
                         episode_len)
        reliabilities[0,i_id] = normalized_reliability_to_reliability(rel)
        radioontimes[0,i_id] = normalized_radio_to_radio(rad)
        retransmissions[0,i_id] = n
        i_id+=1

    # we select the best model
    # temporary avg to find the best model
    re_ = np.nanmean(reliabilities,axis=2).reshape((1,len(instance_id)))
    ra_ = np.nanmean(radioontimes,axis=2).reshape((1,len(instance_id)))
    rn_ = np.nanmean(retransmissions,axis=2).reshape((1,len(instance_id)))
    # we will store the results from the best model here
    rel_ = np.zeros((1,
                     num_episodes))
    rad_ = np.zeros((1,
                     num_episodes))
    ret_ = np.zeros((1,
                     num_episodes))
    models = np.zeros((1,),dtype=np.int32)
    for i in range(1):
        models[i] = int(np.argmax(re_[i]))
        print(f"Best model is instance {models[i]}")
        rel_[i] = reliabilities[i,models[i]]
        rad_[i] = radioontimes[i,models[i]]
        ret_[i] = retransmissions[i,models[i]]

def eval_reward_K():
    # Model params
    testbed =          "kiel"
    instance_name =    "dqn_eval"
    k_worst_nodes =    10
    instance_id = [0,1,2]
    history_size =     2
    reward_K = [10,20,30,40,50,60,70,80,90]
    # Eval params
    num_episodes = 100
    episode_len = 30

    reliabilities = np.zeros((len(reward_K),
                              len(instance_id),
                              num_episodes))
    radioontimes = np.zeros((len(reward_K),
                             len(instance_id),
                             num_episodes))
    retransmissions = np.zeros((len(reward_K),
                             len(instance_id),
                             num_episodes))
    r_id = 0
    for r in reward_K:
        i_id = 0
        for i in instance_id:
            rel,rad, n = run(testbed,
                             instance_name,
                             k_worst_nodes,
                             history_size,
                             i,
                             r,
                             num_episodes,
                             episode_len)
            reliabilities[r_id,i_id] = normalized_reliability_to_reliability(rel)
            radioontimes[r_id,i_id] = normalized_radio_to_radio(rad)
            retransmissions[r_id,i_id] = n
            i_id+=1
        r_id+=1

    # # we select the best model
    # # temporary avg to find the best model
    # re_ = np.nanmean(reliabilities,axis=2).reshape((len(reward_K),len(instance_id)))
    # ra_ = np.nanmean(radioontimes,axis=2).reshape((len(reward_K),len(instance_id)))
    # rn_ = np.nanmean(retransmissions,axis=2).reshape((len(reward_K),len(instance_id)))
    # # we will store the results from the best model here
    # rel_ = np.zeros((len(reward_K),
    #                  num_episodes))
    # rad_ = np.zeros((len(reward_K),
    #                  num_episodes))
    # ret_ = np.zeros((len(reward_K),
    #                  num_episodes))
    # models = np.zeros((len(reward_K),),dtype=np.int32)
    # for i in range(len(reward_K)):
    #     models[i] = int(np.argmax(re_[i]))
    #     print(f"Best model for reward[K={i}] is instance {models[i]}")
    #     rel_[i] = reliabilities[i,models[i]]
    #     rad_[i] = radioontimes[i,models[i]]
    #     ret_[i] = retransmissions[i,models[i]]

    # we reshape to put all episodes from all instance IDs as an axis
    reliabilities = reliabilities.reshape((len(reward_K),-1))
    radioontimes = radioontimes.reshape((len(reward_K),-1))
    retransmissions = retransmissions.reshape((len(reward_K),-1))

    # we save our data
    np.save(os.path.join(PATH_SAVED_RESULTS, "reward_reliabilities"),
            reliabilities)
    np.save(os.path.join(PATH_SAVED_RESULTS, "reward_radioontimes"),
            radioontimes)
    np.save(os.path.join(PATH_SAVED_RESULTS, "reward_retransmissions"),
            retransmissions)

if __name__ == '__main__':
    #eval_k_worst_nodes()
    eval_history_size()
    #eval_reward_K()
    #compare()