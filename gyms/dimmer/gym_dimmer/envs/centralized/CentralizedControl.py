#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""

"""

import time
import re
import threading

import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
import gym_dimmer.envs.utils.glossai_utils as glossai_utils
from gym_dimmer.envs.utils.GymToDispatcherInterface import GymToDispatcherInterface




################################################################################################
################################################################################################
################################################################################################
################################################################################################
################################################################################################


class CentralizedControl(gym.Env):
    """
    GlossyInCooja: Interfaces with Cooja running Glosy with 10 nodes
    Actions: select the number N of transmissions for each node
    States: For each node, the first time a packet was received
    """

    def __init__(self, testbed, k_worst_nodes_len=10, history_len=0, one_hot_len=9, reward_K=30, **kwargs):
        self.__version__ = "0.0.1"
        print("CentralizedControl - Version {}".format(self.__version__))

        self.dispatcher = GymToDispatcherInterface(testbed)
        self.dispatcher.connect()
        self.history_len = history_len
        self.one_hot_len = one_hot_len
        self.reward_K = reward_K
        self.history = np.zeros(history_len)
        self.use_k_worst =k_worst_nodes_len
        self.nb_nodes_used = k_worst_nodes_len if k_worst_nodes_len > 0 else 18
        self.state = np.zeros(2*self.nb_nodes_used+self.one_hot_len+self.history_len)
        # Action Space
        # {Decrement;Nothing;Increment}
        self.action_space = spaces.Discrete(len(glossai_utils.Action))
        self.N = glossai_utils.DEFAULT_N
        self.is_action_outside_limit = False
        # Observation Space (what the agent perceives):
        self.observation_space = spaces.Box(low=-1,
                                            high=1,
                                            shape=(2*self.nb_nodes_used+self.one_hot_len+self.history_len,),
                                            dtype=np.float32)
        # Reward Space
        # Reward is computed as Sum_i( (NB_SLOTS-O_i)**2 - N_i**2)/NB_NODES
        # i.e., the smaller the time to receive the first packet, the better
        self.reward_range = (-200, 0)
        #Store what the agent tried
        self.curr_episode = -1
        self.curr_step = 0
        #reset overall state and seed
        self.seed()
        self.reset()



    def step(self, action):
        is_done = False
        self.curr_step += 1
        if self.curr_step >= glossai_utils.ENV_MAX_NUMBER_OF_STEPS:
            is_done=True

        # Apply the action given by the agent
        self.take_action(action, is_reset=False)
        # Read the nodes result
        dispatcher_is_done = self.get_state()
        is_done |= dispatcher_is_done
        # Compute the reward
        reward = self.get_reward()

        # return everything
        return self.state, reward, is_done, {}

    def take_action(self, action, is_reset = False):
        """Take an action in this environment.
        action: int, 0: Decrement, 1: Keep, 2: Increment
        is_reset: if True, takes the value of action as new N
        Returns None"""
        self.action = action
        # Decrement / Increment
        if action == 0:
            self.N -=1
        elif action == 1:
            self.N = self.N
        elif action == 2:
            self.N +=1
        if is_reset:
            self.N = action
        self.dispatcher.send_action(self.N)
        return

    def get_reward(self):
        state = self.state

        r = 0
        # THRESHOLD BASED SOLUTION
        avg_rel = np.nanmean(state[self.nb_nodes_used:2*self.nb_nodes_used])
        r = 100 -self.reward_K*(self.N)/8 if avg_rel == 1.0 else 0

        # EXPONENTIAL BASED SOLUTION
        # r += 25*np.exp(np.nanmean(state[self.dispatcher.nb_nodes:2*self.dispatcher.nb_nodes])+1)
        # r -= (np.nanmean(state[0:self.dispatcher.nb_nodes])+1)*40

        return r

    def get_state(self):
        """Get the observation.
        State is written to self.state.
        Returns True if episode is finished, False otherwise."""
        # Update history with average of last step value
        if self.history_len:
            self.history[:-1] = self.history[1:]
            self.history[-1] = 1 if self.state[self.nb_nodes_used] == 1.0 else -1

        # get Dispatcher state
        state_tmp, local_N_tmp, is_done = self.dispatcher.request_state()
        state_tmp = np.array(state_tmp)
        if self.use_k_worst:
            # We need to extract the [self.nb_nodes_used] worst values out of [self.dispatcher.nb_nodes] values
            worst_node_ids = np.argsort(state_tmp[self.dispatcher.nb_nodes:2*self.dispatcher.nb_nodes])
            worst_node_ids = worst_node_ids[:self.nb_nodes_used]
            state_tmp = np.array(state_tmp)
            state = np.zeros(2*self.nb_nodes_used+self.one_hot_len+self.history_len)
            state[0:self.nb_nodes_used] = state_tmp[worst_node_ids]
            state[self.nb_nodes_used:2*self.nb_nodes_used] = state_tmp[self.dispatcher.nb_nodes+worst_node_ids]
        else:
            state[0:2*self.dispatcher.nb_nodes] = state_tmp[0:2*self.dispatcher.nb_nodes]
        
        # One hot N representation
        if self.one_hot_len:
            state[2*self.nb_nodes_used+int(local_N_tmp)] = 1 # evrything else is zero by default
        else:
            if self.use_k_worst:
                state[2*self.nb_nodes_used:3*self.nb_nodes_used] = state_tmp[2*self.dispatcher.nb_nodes+worst_node_ids]
            else:
                state[2*self.dispatcher.nb_nodes:] = state_tmp[2*self.dispatcher.nb_nodes:]
       
        if self.history_len:
            state[2*self.nb_nodes_used+self.one_hot_len:] = self.history

        # print(self.N, local_N_tmp)
        if self.N != local_N_tmp:
            self.N = int(local_N_tmp)
        self.state = state
        return is_done

    def reset(self):
        """
        Reset the state of the environment and returns an initial observation.
        Returns the state written in self.state
        """
        self.curr_episode += 1
        self.curr_step = 0
        self.N = np.random.randint(1, glossai_utils.MAX_N+1)
        #self.N = 3 # todo
        # contact dispatcher
        self.dispatcher.send_action(self.N)
        self.dispatcher.request_reset()
        self.history = np.ones(self.history_len)
        # we need to fill history with correct values
        for _ in range(self.history_len):
            self.get_state()
        return self.state

    def render(self, mode='human', close=False):
        return

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
